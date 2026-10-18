"""Score artefact detectors against the labelled simulator captures.

A detection counts as correct when its box covers the artefact's true image
column (from the world file) and has the right type. A detection of a type
that has no artefact of that type in view there is a false alarm.

Compares the new detector with the one currently on main (Andrew's green
rule), run exactly as written there, including the way it reads the image.

Usage: python3 scripts/detector_eval.py DATA_DIR [save_examples_dir]
"""
import json
import math
import os
import sys

import cv2
import numpy as np

from offline_cave_map import PACKAGE, artefacts_from_world  # noqa: F401  (PACKAGE puts the package on sys.path)

from cave_explorer.perception import artefact_detector

RANGE_BINS = ((1.0, 3.0), (3.0, 5.0), (5.0, 9.0))
POSSIBLE_RANGE = 12.0        # an artefact of the right type this far ahead makes a box 'not a false alarm'
COLUMN_MARGIN = 25          # a box counts if it is within this many pixels of the true column


def andrew_detector(image_bgr):
    """The detector on main (commit 133e686): green mask, 'Alien' if tall else 'Rock'.

    On the robot the image arrives as rgb8 and is passed to cvtColor as if BGR,
    so it is fed the RGB image here too.
    """
    image = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (50, 100, 100), (80, 255, 255))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for contour in contours:
        if cv2.contourArea(contour) > 2000:
            x, y, w, h = cv2.boundingRect(contour)
            kind = 'Alien' if h / float(w) > 1.3 else 'Rock'
            out.append(artefact_detector.Detection(kind, x, y, w, h, cv2.contourArea(contour)))
    return out


def covers(detection, column):
    return detection.x - COLUMN_MARGIN <= column <= detection.x + detection.width + COLUMN_MARGIN


ARTEFACTS = artefacts_from_world()


def types_ahead(r):
    """Types with an artefact in front of the camera within POSSIBLE_RANGE, even if partly hidden or very close"""
    ahead = set()
    for a in ARTEFACTS:
        dx, dy = a['x'] - r['x'], a['y'] - r['y']
        forward = dx * math.cos(r['yaw']) + dy * math.sin(r['yaw'])
        if forward > -0.5 and math.hypot(dx, dy) <= POSSIBLE_RANGE:
            ahead.add(a['type'])
    return ahead


def score(records, data, detect, type_names):
    """type_names maps a detector label to the true artefact type it claims"""
    stats = {}      # (type, range bin) -> [found, total]
    false_alarms = {}
    images_with_false = {}
    off_label_boxes = {}
    for r in records:
        bgr = cv2.imread(os.path.join(data, r['id'] + '.png'))
        detections = detect(bgr)
        claimed = [(type_names.get(d.kind, d.kind), d) for d in detections]
        for kind in {k for k, _ in claimed}:
            matched = [d for k, d in claimed if k == kind]
            truths = [v for v in r['visible'] if v['type'] == kind]
            # A false alarm is a box of a type that has no artefact ahead of the camera at all
            stray = matched if kind not in types_ahead(r) else []
            false_alarms[kind] = false_alarms.get(kind, 0) + len(stray)
            images_with_false[kind] = images_with_false.get(kind, 0) + (1 if stray else 0)
            off_label = [d for d in matched if not any(covers(d, v['column']) for v in truths)]
            off_label_boxes[kind] = off_label_boxes.get(kind, 0) + len(off_label)
        for v in r['visible']:
            for low, high in RANGE_BINS:
                if low <= v['range'] < high:
                    cell = stats.setdefault((v['type'], (low, high)), [0, 0])
                    cell[1] += 1
                    cell[0] += any(k == v['type'] and covers(d, v['column']) for k, d in claimed)
    return stats, false_alarms, images_with_false, off_label_boxes


def report(title, stats, false_alarms, images_with_false, off_label_boxes, kinds, n_images):
    print(f'\n== {title}')
    print(f'{"type":10s} ' + ' '.join(f'{f"{lo:.0f}-{hi:.0f} m":>12s}' for lo, hi in RANGE_BINS) +
          f' {"all ranges":>12s}  false alarms (images / boxes) | other')
    for kind in kinds:
        cells = [stats.get((kind, b), [0, 0]) for b in RANGE_BINS]
        row = ' '.join(f'{c[0]:>4d}/{c[1]:<3d} {100 * c[0] / max(c[1], 1):3.0f}%' for c in cells)
        found, total = sum(c[0] for c in cells), sum(c[1] for c in cells)
        print(f'{kind:10s} {row} {found:>4d}/{total:<3d} {100 * found / max(total, 1):3.0f}%  '
              f'{images_with_false.get(kind, 0)} / {false_alarms.get(kind, 0)} | boxes off any labelled artefact: {off_label_boxes.get(kind, 0)}')


def main():
    data = sys.argv[1]
    records = json.load(open(os.path.join(data, 'labels.json')))
    kinds = ['alien', 'crystals', 'mushroom', 'sphere']

    new = score(records, data, artefact_detector.detect, {})
    report('new detector (perception/artefact_detector.py)', *new, kinds, len(records))

    old = score(records, data, andrew_detector, {'Alien': 'alien', 'Rock': 'crystals'})
    report("Andrew's detector on main (Alien -> alien, Rock -> crystals)", *old, ['alien', 'crystals'], len(records))

    if len(sys.argv) > 2:
        os.makedirs(sys.argv[2], exist_ok=True)
        picks = [r for r in records if r['visible']][::max(1, len(records) // 24)][:24]
        for r in picks:
            bgr = cv2.imread(os.path.join(data, r['id'] + '.png'))
            shown = artefact_detector.draw(bgr, artefact_detector.detect(bgr))
            for v in r['visible']:
                cv2.putText(shown, f"truth:{v['type']} {v['range']}m", (int(v['column']) - 40, 470 - 18 * r['visible'].index(v)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            cv2.imwrite(os.path.join(sys.argv[2], r['id'] + '.png'), shown)


if __name__ == '__main__':
    main()
