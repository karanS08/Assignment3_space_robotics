"""Colour-based detector for the green alien, green crystals, blue mushrooms and white sphere.

Standalone: takes an image, returns boxes. Does not depend on ROS, so it can
be scored offline (scripts/detector_eval.py) and called from any node.

IMPORTANT: expects a BGR image, as OpenCV uses. A ROS 'rgb8' image must be
converted first with cv2.cvtColor(image, cv2.COLOR_RGB2BGR); otherwise red and
blue swap and every hue below is wrong.

Owner: Karan Sharma (alien and mushroom detectors)
Task: Perception 2
"""

from dataclasses import dataclass

import cv2
import numpy as np

# OpenCV HSV: hue 0-179, saturation 0-255, value 0-255.
# Ranges measured from simulator captures (scripts/detector_colour_stats.py).
SPECS = {
    # Pale lime: saturation 160-180 in every capture, below the crystals
    'alien': {'lower': (34, 135, 60), 'upper': (47, 192, 255), 'min_area': 120, 'close': 9, 'min_solidity': 0.0},
    # Deep emerald: saturation 204-255
    'crystals': {'lower': (44, 196, 25), 'upper': (68, 255, 255), 'min_area': 200, 'close': 9, 'min_solidity': 0.0},
    # Blue caps, hue 110-112 and saturated; the white gills between caps are bridged by the larger closing kernel
    'mushroom': {'lower': (107, 100, 70), 'upper': (118, 255, 255), 'min_area': 80, 'close': 21, 'min_solidity': 0.0},
    # Pale blue-white ball: hue 105, saturation about 49, bright
    'sphere': {'lower': (102, 35, 120), 'upper': (108, 70, 255), 'min_area': 120, 'close': 5, 'min_solidity': 0.9,
               'min_fill': 0.65, 'aspect': (0.6, 1.7)},
}

# The rover's own body fills the bottom-left of every camera frame
ROVER_ROWS = 335
ROVER_COLUMNS = 335

OPEN_KERNEL = np.ones((3, 3), np.uint8)


@dataclass
class Detection:
    kind: str
    x: int
    y: int
    width: int
    height: int
    score: float          # number of matching pixels

    @property
    def centre_column(self):
        return self.x + self.width / 2.0


def colour_mask(hsv, spec):
    mask = cv2.inRange(hsv, np.array(spec['lower']), np.array(spec['upper']))
    mask[ROVER_ROWS:, :ROVER_COLUMNS] = 0
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, OPEN_KERNEL)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((spec['close'], spec['close']), np.uint8))


def detect(image_bgr, kinds=None):
    """Find artefacts in a BGR image. Returns a list of Detection"""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    detections = []
    for kind, spec in SPECS.items():
        if kinds is not None and kind not in kinds:
            continue
        mask = colour_mask(hsv, spec)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < spec['min_area']:
                continue
            hull_area = cv2.contourArea(cv2.convexHull(contour))
            if hull_area > 0 and area / hull_area < spec['min_solidity']:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if area / float(w * h) < spec.get('min_fill', 0.0):
                continue
            low, high = spec.get('aspect', (0.0, 1e9))
            if not low <= w / float(h) <= high:
                continue
            detections.append(Detection(kind, x, y, w, h, area))
    return detections


def draw(image_bgr, detections):
    """Draw boxes and labels on a copy of the image"""
    colours = {'alien': (0, 255, 0), 'crystals': (0, 200, 120), 'mushroom': (255, 120, 0),
               'sphere': (255, 255, 255), 'ice': (255, 230, 150)}
    out = image_bgr.copy()
    for d in detections:
        colour = colours.get(d.kind, (0, 0, 255))
        cv2.rectangle(out, (d.x, d.y), (d.x + d.width, d.y + d.height), colour, 2)
        cv2.putText(out, d.kind, (d.x, max(d.y - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
    return out
