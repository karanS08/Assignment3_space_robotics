"""Measure the colour of each artefact type, and of everything else, from labelled captures.

Artefact pixels are taken from the labelled image column (+- COLUMN_HALF_WIDTH)
whose depth is within DEPTH_TOLERANCE of the artefact's true range, so wall
and floor behind or in front of it are left out.

Usage: python3 scripts/detector_colour_stats.py DATA_DIR
"""
import json
import os
import sys

import cv2
import numpy as np

COLUMN_HALF_WIDTH = 70
DEPTH_TOLERANCE = 1.0
ROVER = (slice(335, 480), slice(0, 335))     # the rover's own body, bottom-left of every frame


def main():
    data = sys.argv[1]
    records = json.load(open(os.path.join(data, 'labels.json')))
    samples = {}
    background = []
    for r in records:
        bgr = cv2.imread(os.path.join(data, r['id'] + '.png'))
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        depth = np.load(os.path.join(data, r['id'] + '_depth.npy')).astype(np.float32)
        owned = np.zeros(depth.shape, bool)
        owned[ROVER] = True
        for v in r['visible']:
            columns = np.zeros(depth.shape, bool)
            columns[:, max(0, int(v['column'] - COLUMN_HALF_WIDTH)):int(v['column'] + COLUMN_HALF_WIDTH)] = True
            on_artefact = columns & ~owned & (np.abs(depth - v['range']) < DEPTH_TOLERANCE)
            samples.setdefault(v['type'], []).append(hsv[on_artefact])
            owned |= columns
        background.append(hsv[~owned][::7])

    background = np.concatenate(background)
    print(f'background pixels: {len(background)}')
    print('type        pixels  | H p5 p25 p50 p75 p95 | S p5 p50 | V p5 p50 p95 | % of background pixels inside this type\'s H-S-V box')
    for kind, chunks in sorted(samples.items()):
        px = np.concatenate(chunks)
        h = np.percentile(px[:, 0], [5, 25, 50, 75, 95]).astype(int)
        s = np.percentile(px[:, 1], [5, 50]).astype(int)
        v = np.percentile(px[:, 2], [5, 50, 95]).astype(int)
        inside = ((background[:, 0] >= h[0]) & (background[:, 0] <= h[4]) & (background[:, 1] >= s[0])
                  & (background[:, 2] >= v[0]))
        print(f'{kind:10s} {len(px):7d} | {h} | {s} | {v} | {100 * inside.mean():5.2f} %')


if __name__ == '__main__':
    main()
