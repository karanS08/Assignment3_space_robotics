"""Ice formation detector using colour, texture and the depth camera.

Ice has two looks (blue-grey and brown-beige), so colour alone cannot find it.
A random forest classifies every other pixel from its colour, local texture
and depth features (height above the floor, bumpiness), then pixels are
grouped into boxes. Trained by scripts/ice_train.py.

Needs both camera images. The colour image must be BGR (see artefact_detector).
Fixed for this robot: camera 1.3 m above the floor, 120 degree field of view.

Owner: Karan Sharma
Task: Perception 2 (ice)
"""

import math
import os

import cv2
import numpy as np

from cave_explorer.perception.artefact_detector import Detection, ROVER_COLUMNS, ROVER_ROWS

CAMERA_HEIGHT = 1.3          # [m] above the floor
FOCAL = 360.0 / math.tan(2.0944 / 2)
THRESHOLD = 0.6              # forest probability for a pixel to count as ice
MIN_AREA = 1500              # smallest box area [pixels^2]

_ROWS = np.arange(480, dtype=np.float32)[:, None] * np.ones((1, 720), np.float32)
_model = None


def features(image_bgr, depth):
    """(480, 720, 10) feature image: H, S, V, V and S texture, depth bumpiness and gradient, depth, height, row"""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV).astype(np.float32)
    d = np.where(np.isfinite(depth), depth, 10.0).astype(np.float32)

    def local_std(a, k):
        mean = cv2.blur(a, (k, k))
        return np.sqrt(np.maximum(cv2.blur(a * a, (k, k)) - mean * mean, 0))

    height = CAMERA_HEIGHT - (_ROWS - 240) / FOCAL * d
    gx = cv2.Sobel(d, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(d, cv2.CV_32F, 0, 1, ksize=3)
    return np.dstack([hsv[..., 0], hsv[..., 1], hsv[..., 2], local_std(hsv[..., 2], 7), local_std(hsv[..., 1], 7),
                      local_std(d, 7), np.hypot(gx, gy), d, height, _ROWS]).astype(np.float32)


def model_path(name='ice_forest.npz'):
    here = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'data', name)
    if os.path.exists(here):
        return here
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('cave_explorer'), 'data', name)


def load_model(path=None):
    """The forest as numpy arrays (ice_forest.npz, no scikit-learn needed), or a scikit-learn .joblib file"""
    global _model
    if _model is None or path is not None:
        path = path or model_path()
        if path.endswith('.npz'):
            from cave_explorer.perception.box_classifier import BoxClassifier
            _model = BoxClassifier(path)
        else:
            import joblib
            _model = joblib.load(path)
    return _model


def detect(image_bgr, depth, model=None, threshold=THRESHOLD, min_area=MIN_AREA, step=2):
    """Find ice formations. Returns a list of Detection with kind 'ice'. step: classify every step-th pixel"""
    model = model or load_model()
    f = features(image_bgr, depth)
    h, w = f.shape[:2]

    # Classify every step-th pixel, which is fast enough and loses nothing at these blob sizes
    sub = f[::step, ::step]
    probability = model.predict_proba(sub.reshape(-1, f.shape[2]))[:, 1].reshape(sub.shape[:2])
    mask = cv2.resize((probability > threshold).astype(np.uint8) * 255, (w, h), interpolation=cv2.INTER_NEAREST)
    mask[ROVER_ROWS:, :ROVER_COLUMNS] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((17, 17), np.uint8))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detections = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area >= min_area:
            x, y, bw, bh = cv2.boundingRect(contour)
            detections.append(Detection('ice', x, y, bw, bh, area))
    return detections
