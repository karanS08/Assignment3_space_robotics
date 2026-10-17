"""Second stage for the colour detectors: a trained classifier that accepts or rejects each candidate box.

The colour detector (artefact_detector.py) finds every patch of the right colour. Some are not artefacts: the stop-sign picture on
some walls has green trees the colour of the alien and crystals, and rock and floor sometimes fall in the mushroom and sphere ranges.
This classifier looks at each box and decides from its colour, texture, shape and depth whether it is a real artefact.

The model is a random forest trained offline (scripts/box_classifier_train.py) on boxes from the labelled simulator captures. It is
stored as plain arrays and evaluated with numpy, so the node does not need scikit-learn.

Expects a BGR image and the matching depth image. No ROS dependency.

Owner: Karan Sharma
Task: Perception 2
"""
import os

import cv2
import numpy as np

from cave_explorer.perception.artefact_detector import SPECS, colour_mask

KINDS = ('alien', 'crystals', 'mushroom', 'sphere')
MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'box_classifier.npz')
THRESHOLD = 0.5


def box_features(image_bgr, depth, box, kind):
    """Numbers that describe one candidate box"""
    x, y, w, h = box
    x0, y0 = max(x, 0), max(y, 0)
    crop = image_bgr[y0:y + h, x0:x + w]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    mask = colour_mask(hsv, SPECS[kind]) > 0
    if not mask.any():
        mask = np.ones(mask.shape, bool)
    pixels = hsv[mask].astype(np.float32)
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 60, 150) > 0
    laplacian = cv2.Laplacian(gray, cv2.CV_32F)

    d = np.asarray(depth, dtype=np.float32)
    patch = d[y0:y + h, x0:x + w].ravel()
    patch = np.sort(patch[np.isfinite(patch) & (patch > 0.3) & (patch < 20.0)])
    near = float(np.median(patch[:max(1, int(0.3 * len(patch)))])) if len(patch) else 20.0
    far = float(np.median(patch[int(0.7 * len(patch)):])) if len(patch) else 20.0
    sides = []
    for a, b in ((max(x - w, 0), x0), (x + w, min(x + 2 * w, d.shape[1]))):
        if b - a >= 3:
            s = d[y0:y + h, a:b].ravel()
            s = s[np.isfinite(s) & (s > 0.3)]
            if len(s) > 10:
                sides.append(float(np.median(s)))
    gap = (max(sides) - near) if sides else 0.0

    return np.array([KINDS.index(kind) == k for k in range(len(KINDS))] + [
        w, h, w / float(h), w * h * near * near / 1e4,      # size, and size scaled by distance (about the real size)
        mask.mean(),                                         # share of the box in the colour range
        *pixels.mean(axis=0), *pixels.std(axis=0),           # colour of the matching pixels
        gray.std(), np.abs(laplacian).mean(), edges.mean(),  # texture: tree leaves are busy, artefacts are smooth
        near, far - near, gap,                               # depth: an artefact stands out in front of the wall
        (y + h / 2.0) / image_bgr.shape[0],                  # height in the image
    ], dtype=np.float32)


class BoxClassifier:
    """Random forest stored as arrays: for every tree, its nodes' feature, threshold, children and leaf value"""

    def __init__(self, path=MODEL_PATH):
        m = np.load(path)
        self.feature_, self.threshold_ = m['feature'], m['threshold']
        self.left_, self.right_, self.value_ = m['left'], m['right'], m['value']
        self.roots_ = m['roots']

    def probability(self, features):
        votes = 0.0
        for node in self.roots_:
            while self.left_[node] >= 0:
                node = self.left_[node] if features[self.feature_[node]] <= self.threshold_[node] else self.right_[node]
            votes += self.value_[node]
        return votes / len(self.roots_)

    def predict_proba(self, X):
        """Probability of the positive class for every row of X, all rows at once (same output as scikit-learn)"""
        votes = np.zeros(len(X), np.float32)
        rows = np.arange(len(X))
        for root in self.roots_:
            node = np.full(len(X), root, np.int32)
            inner = self.left_[node] >= 0
            while inner.any():
                n = node[inner]
                go_left = X[rows[inner], self.feature_[n]] <= self.threshold_[n]
                node[inner] = np.where(go_left, self.left_[n], self.right_[n])
                inner = self.left_[node] >= 0
            votes += self.value_[node]
        p = votes / len(self.roots_)
        return np.column_stack([1.0 - p, p])

    def accept(self, image_bgr, depth, detection, threshold=THRESHOLD):
        box = (detection.x, detection.y, detection.width, detection.height)
        return self.probability(box_features(image_bgr, depth, box, detection.kind)) >= threshold
