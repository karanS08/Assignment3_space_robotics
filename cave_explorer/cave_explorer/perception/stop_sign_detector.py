"""Stop-sign detector: a red octagon with white lettering.

The template's stop-sign detector (a Haar cascade) also fires on the framed year labels of the Mars timeline poster on some walls.
A real stop sign is a red octagon, so this detector asks for exactly that:
  1. strongly red pixels (red sits at both ends of the hue scale),
  2. a solid blob whose outline has about eight corners, roughly square and filling about 83 % of its bounding box (an octagon does),
  3. pale lettering inside it (the red has high saturation, the letters and the border have very low saturation).

Expects a BGR image, as OpenCV uses. No ROS dependency.

Owner: Karan Sharma
Task: Perception 2 (stop signs)
"""

import cv2
import numpy as np

from cave_explorer.perception.artefact_detector import Detection, ROVER_COLUMNS, ROVER_ROWS

# OpenCV HSV: hue 0-179. Red wraps around, so there are two ranges.
RED_RANGES = (((0, 110, 60), (8, 255, 255)), ((170, 110, 60), (179, 255, 255)))
MIN_AREA = 120                  # [pixels^2] about 12 x 12; smaller blobs are too far away to judge
ASPECT = (0.40, 1.40)           # width / height; a sign seen from 56 degrees to the side looks about half as wide
FILL = (0.60, 0.95)             # blob area / bounding box area; a regular octagon is 0.83
MIN_SOLIDITY = 0.88             # blob area / convex hull area
CORNERS = (6, 11)               # number of corners of the simplified outline (8 for an octagon, with some slack for noise)
LETTER_SATURATION = 70          # pixels below this saturation count as lettering (measured: red is above 110)
PALE_SHARE = (0.02, 0.45)       # share of the octagon that is lettering or border (measured: about 0.16)

CLOSE = np.ones((5, 5), np.uint8)
OPEN = np.ones((3, 3), np.uint8)


def red_mask(hsv):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for low, high in RED_RANGES:
        mask |= cv2.inRange(hsv, np.array(low), np.array(high))
    mask[ROVER_ROWS:, :ROVER_COLUMNS] = 0
    return cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_CLOSE, CLOSE), cv2.MORPH_OPEN, OPEN)


def check(contour, hsv):
    """Return the reasons a red blob is not a stop sign (an empty list means it is one)"""
    area = cv2.contourArea(contour)
    if area < MIN_AREA:
        return ['too small']
    x, y, w, h = cv2.boundingRect(contour)
    problems = []
    if not ASPECT[0] <= w / float(h) <= ASPECT[1]:
        problems.append('not square')
    if not FILL[0] <= area / float(w * h) <= FILL[1]:
        problems.append('wrong fill')
    hull = cv2.contourArea(cv2.convexHull(contour))
    if hull <= 0 or area / hull < MIN_SOLIDITY:
        problems.append('not solid')
    corners = len(cv2.approxPolyDP(contour, 0.03 * cv2.arcLength(contour, True), True))
    if not CORNERS[0] <= corners <= CORNERS[1]:
        problems.append('wrong number of corners')
    filled = np.zeros((h, w), np.uint8)
    cv2.drawContours(filled, [contour - np.array([x, y])], -1, 255, -1)
    inside = hsv[y:y + h, x:x + w][filled > 0]
    pale = float((inside[:, 1] < LETTER_SATURATION).mean())
    if not PALE_SHARE[0] <= pale <= PALE_SHARE[1]:
        problems.append('no lettering')
    return problems


def detect(image_bgr):
    """Find red octagonal stop signs in a BGR image. Returns a list of Detection with kind 'stop_sign'"""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    contours, _ = cv2.findContours(red_mask(hsv), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for contour in contours:
        if not check(contour, hsv):
            x, y, w, h = cv2.boundingRect(contour)
            out.append(Detection('stop_sign', x, y, w, h, cv2.contourArea(contour)))
    return out
