"""Train and score the ice detector (random forest) on the labelled simulator captures.

Pixel truth comes from projecting the ice mesh into each image and keeping
points whose distance agrees with the depth camera (scripts/ice_truth.py).

Step 1 trains on even-numbered images and scores on odd-numbered ones, so the
reported numbers are on images the forest never saw.
Step 2 trains the final forest on all images and saves it.

Usage: python3 scripts/ice_train.py DATA_DIR [output.joblib]
"""
import json
import os
import sys

import cv2
import joblib
import numpy as np
from sklearn.ensemble import RandomForestClassifier

import ice_truth as it
from offline_cave_map import PACKAGE  # noqa: F401  (puts the package on sys.path)

from cave_explorer.perception import ice_detector

CAMERA_HEIGHT = ice_detector.CAMERA_HEIGHT
FOREST = dict(n_estimators=60, max_depth=14, min_samples_leaf=5, class_weight='balanced', n_jobs=12, random_state=0)


def load(data):
    records = json.load(open(os.path.join(data, 'labels.json')))
    points = it.ice_points(2)
    cases = []
    for r in records:
        image = cv2.imread(os.path.join(data, r['id'] + '.png'))
        depth = np.load(os.path.join(data, r['id'] + '_depth.npy')).astype(np.float32)
        mask = cv2.dilate(it.visible_mask(points, r, depth, CAMERA_HEIGHT).astype(np.uint8), np.ones((5, 5), np.uint8)) > 0
        cases.append((image, depth, ice_detector.features(image, depth), mask))
    return cases


def training_pixels(cases, rng):
    x, y = [], []
    for _, _, f, m in cases:
        valid = np.ones(m.shape, bool)
        valid[ice_detector.ROVER_ROWS:, :ice_detector.ROVER_COLUMNS] = False
        ice = np.argwhere(m & valid)
        other = np.argwhere(~cv2.dilate(m.astype(np.uint8), np.ones((15, 15), np.uint8)).astype(bool) & valid)
        if len(ice):
            s = ice[rng.choice(len(ice), min(len(ice), 500), replace=False)]
            x.append(f[s[:, 0], s[:, 1]])
            y += [1] * len(s)
        s = other[rng.choice(len(other), 250, replace=False)]
        x.append(f[s[:, 0], s[:, 1]])
        y += [0] * len(s)
    return np.concatenate(x), np.array(y)


def score(model, cases, min_area, min_truth_pixels=800):
    shown = found = false_images = false_boxes = 0
    for image, depth, _, m in cases:
        boxes = ice_detector.detect(image, depth, model=model, min_area=min_area)
        visible = m.sum() >= min_truth_pixels
        shown += visible
        hit = stray = False
        for b in boxes:
            inside = m[b.y:b.y + b.height, b.x:b.x + b.width].mean()
            hit |= inside >= 0.35
            if inside < 0.15:
                stray = True
                false_boxes += 1
        found += visible and hit
        false_images += stray
    return shown, found, false_images, false_boxes


def main():
    data = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else None
    rng = np.random.default_rng(0)
    cases = load(data)
    train, test = cases[0::2], cases[1::2]

    forest = RandomForestClassifier(**FOREST).fit(*training_pixels(train, rng))
    print(f'Trained on {len(train)} images, scored on {len(test)} images the forest never saw:')
    for min_area in (1500, 4000):
        shown, found, false_images, false_boxes = score(forest, test, min_area)
        print(f'  min box area {min_area}: ice found in {found} of {shown} images showing it ({100 * found / shown:.0f}%); '
              f'false ice box in {false_images} of {len(test)} images ({100 * false_images / len(test):.0f}%), {false_boxes} boxes')

    if out:
        final = RandomForestClassifier(**FOREST).fit(*training_pixels(cases, rng))
        joblib.dump(final, out, compress=3)
        print(f'Final forest trained on all {len(cases)} images, saved to {out} ({os.path.getsize(out) / 1e6:.1f} MB)')


if __name__ == '__main__':
    main()
