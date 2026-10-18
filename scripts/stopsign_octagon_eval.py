"""Score two stop-sign detectors on the simulator captures made by stop_capture.py: the template's Haar cascade and the red-octagon classifier.

Positive images: those labelled with a stop-sign wall that is at least half inside the picture and within 9 m. A detection is correct when
its box centre lies over the wall (between the wall's left and right image edge, with a small margin).
Negative images: those with no stop-sign wall in view. Any box in them is a false alarm. They include views of every other wall
(this covers the walls with the Mars timeline poster), views around the artefacts, and random views.
The 25 'arc' images are 5 per sign, 28 degrees apart, at -56 to +56 degrees from the sign's front.

Usage: python3 scripts/stopsign_octagon_eval.py DATA_DIR [EXAMPLES_PNG]
"""
import json
import os
import re
import sys

import cv2
import numpy as np

from offline_cave_map import PACKAGE, load_cave_grid
from wall_views import RES, sign_in_direction, visible_stop_walls, walls_from_world

from cave_explorer.perception import stop_sign_detector

MARGIN = 40
RANGE_BINS = ((1.0, 3.0), (3.0, 5.0), (5.0, 9.0))


def cascade_boxes(cascade, image):
    boxes = cascade.detectMultiScale(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), scaleFactor=1.1, minNeighbors=4, minSize=(20, 20))
    return [(int(x), int(y), int(w), int(h)) for x, y, w, h in boxes]


def octagon_boxes(cascade, image):
    return [(d.x, d.y, d.width, d.height) for d in stop_sign_detector.detect(image)]


def covers(box, wall):
    centre = box[0] + box[2] / 2.0
    return wall['left'] - MARGIN <= centre <= wall['right'] + MARGIN


def main():
    data = sys.argv[1]
    records = json.load(open(os.path.join(data, 'labels.json')))
    cascade = cv2.CascadeClassifier(os.path.join(PACKAGE, 'config', 'stop_data.xml'))
    detectors = {'template cascade': cascade_boxes, 'red octagon': octagon_boxes}

    positives, negatives, ambiguous = [], [], []
    for r in records:
        clear = [w for w in r['stop_walls'] if w['inside'] >= 0.5 and w['range'] <= 9.0]
        if clear:
            positives.append((r, clear))
        elif not r['stop_walls']:
            negatives.append(r)
        else:
            ambiguous.append(r)
    arcs = [(r, w) for r, w in positives if r['kind'].startswith('arc')]
    print(f'{len(records)} images: {len(positives)} with a stop sign clearly in view ({len(arcs)} of them the arc photos), '
          f'{len(negatives)} with none, {len(ambiguous)} ambiguous (sign mostly out of the picture, left out)')

    grid, origin = load_cave_grid(RES)
    stop_walls = [w for w in walls_from_world() if w['stop']]
    found = {name: {} for name in detectors}
    shown = {name: [] for name in detectors}
    false_alarms = {name: {} for name in detectors}
    for name, detect in detectors.items():
        for r, walls in positives:
            image = cv2.imread(os.path.join(data, r['id'] + '.png'))
            boxes = detect(cascade, image)
            hit = any(covers(b, w) for b in boxes for w in walls)
            found[name][r['id']] = hit
        for r in negatives:
            image = cv2.imread(os.path.join(data, r['id'] + '.png'))
            # A box is only a false alarm if no real stop sign lies in that direction at any distance
            boxes = [b for b in detect(cascade, image) if not sign_in_direction(stop_walls, grid >= 50, origin, r['x'], r['y'], r['yaw'], b)]
            if boxes:
                false_alarms[name][r['id']] = len(boxes)

    print('\nStop signs found (positives), by distance:')
    print(f'{"detector":18s} ' + ' '.join(f'{f"{lo:.0f}-{hi:.0f} m":>14s}' for lo, hi in RANGE_BINS) + f' {"all":>14s}')
    for name in detectors:
        row = []
        for lo, hi in RANGE_BINS:
            ids = [r['id'] for r, ws in positives if lo <= min(w['range'] for w in ws) < hi]
            row.append(f'{sum(found[name][i] for i in ids)}/{len(ids)}'.rjust(14))
        total = sum(found[name].values())
        print(f'{name:18s} ' + ' '.join(row) + f' {f"{total}/{len(positives)} ({100 * total / max(len(positives), 1):.0f}%)":>14s}')

    print('\nThe 25 arc photos, by angle from the front of the sign (5 signs each):')
    angles = sorted({float(re.search(r'([+-]\d+)', r['kind']).group(1)) for r, _ in arcs})
    print(f'{"detector":18s} ' + ' '.join(f'{f"{a:+.0f} deg":>9s}' for a in angles) + f' {"all":>9s}')
    for name in detectors:
        row = []
        for a in angles:
            ids = [r['id'] for r, _ in arcs if float(re.search(r'([+-]\d+)', r['kind']).group(1)) == a]
            row.append(f'{sum(found[name][i] for i in ids)}/{len(ids)}'.rjust(9))
        arc_total = sum(found[name][r['id']] for r, _ in arcs)
        print(f'{name:18s} ' + ' '.join(row) + f' {f"{arc_total}/{len(arcs)}":>9s}')

    print(f'\nFalse alarms on the {len(negatives)} images with no stop sign in view (a box over a real but distant sign does not count):')
    # Group the images without a stop sign by what they show: the Mars timeline poster (the case that fools the cascade), plain rock walls, artefacts, random places
    posters = [w for w in walls_from_world() if w['poster']]
    groups = {'poster walls': [], 'rock walls': [], 'artefacts': [], 'random': []}
    for r in negatives:
        in_view = [w for w in visible_stop_walls(posters, grid >= 50, origin, r['x'], r['y'], r['yaw']) if w['inside'] >= 0.5 and w['range'] <= 9.0]
        if in_view:
            groups['poster walls'].append(r['id'])
        elif r['kind'] == 'artefact':
            groups['artefacts'].append(r['id'])
        elif r['kind'] == 'random':
            groups['random'].append(r['id'])
        else:
            groups['rock walls'].append(r['id'])
    print(f'{"detector":18s} ' + ' '.join(f'{g:>14s}' for g in groups) + f' {"all":>14s}')
    for name in detectors:
        row = [f'{sum(1 for i in ids if i in false_alarms[name])}/{len(ids)}'.rjust(14) for ids in groups.values()]
        print(f'{name:18s} ' + ' '.join(row) + f' {f"{len(false_alarms[name])}/{len(negatives)}":>14s}')

    if len(sys.argv) > 2:
        tiles = []
        for r, walls in arcs[:15]:
            image = cv2.imread(os.path.join(data, r['id'] + '.png'))
            for box in octagon_boxes(cascade, image):
                cv2.rectangle(image, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (0, 255, 0), 3)
            for box in cascade_boxes(cascade, image):
                cv2.rectangle(image, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 120, 0), 2)
            cv2.putText(image, r['kind'].replace('arc ', ''), (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            tiles.append(cv2.resize(image, (360, 240)))
        false_ids = list(false_alarms['template cascade'])[:5]
        for i in false_ids:
            image = cv2.imread(os.path.join(data, i + '.png'))
            for box in cascade_boxes(cascade, image):
                cv2.rectangle(image, (box[0], box[1]), (box[0] + box[2], box[1] + box[3]), (255, 120, 0), 3)
            cv2.putText(image, 'no stop sign here', (8, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            tiles.append(cv2.resize(image, (360, 240)))
        while len(tiles) % 5:
            tiles.append(np.zeros((240, 360, 3), np.uint8))
        cv2.imwrite(sys.argv[2], np.vstack([np.hstack(tiles[i:i + 5]) for i in range(0, len(tiles), 5)]))


if __name__ == '__main__':
    main()
