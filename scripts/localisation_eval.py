"""Score artefact localisation (Perception 3) on the labelled simulator captures, without the simulator.

For every detection that lands on a visible artefact of the right type, the estimated map position is compared with the artefact's true
position from the world file. Then all sightings are merged into a map of artefacts, which is compared with the real artefacts.

The world file gives each artefact's origin, which is not always its centre (for example, ice formations are up to 5 m wide),
so errors of up to about a metre are partly the label, not the method.

Usage: python3 scripts/localisation_eval.py DATA_DIR [EXTRA_PACKAGE_DIR_WITH_ICE_MODEL]
"""
import json
import math
import os
import sys

import cv2
import numpy as np

from depth_localiser import ArtefactMap, localise
from offline_cave_map import PACKAGE, artefacts_from_world

from cave_explorer.perception import artefact_detector, ice_detector

RANGE_BINS = ((1.0, 3.0), (3.0, 5.0), (5.0, 9.0))
COLUMN_MARGIN = 25


def main():
    data = sys.argv[1]
    records = json.load(open(os.path.join(data, 'labels.json')))
    truth = {a['name']: a for a in artefacts_from_world()}
    ice_model = ice_detector.load_model(os.path.join(PACKAGE, "data", "ice_forest.joblib"))

    errors = {}                                    # (type, range bin) -> list of metres
    sightings = ArtefactMap(merge_radius=2.5)
    all_errors = []
    for r in records:
        image = cv2.imread(os.path.join(data, r['id'] + '.png'))
        depth = np.load(os.path.join(data, r['id'] + '_depth.npy')).astype(np.float32)
        detections = artefact_detector.detect(image) + ice_detector.detect(image, depth, model=ice_model)
        for d in detections:
            place = localise((d.x, d.y, d.width, d.height), depth, r['x'], r['y'], r['yaw'])
            if place is None:
                continue
            sightings.add(d.kind, place[0], place[1])
            for v in r['visible']:
                if v['type'] == d.kind and d.x - COLUMN_MARGIN <= v['column'] <= d.x + d.width + COLUMN_MARGIN:
                    a = truth[v['name']]
                    error = math.hypot(place[0] - a['x'], place[1] - a['y'])
                    all_errors.append(error)
                    for low, high in RANGE_BINS:
                        if low <= v['range'] < high:
                            errors.setdefault((d.kind, (low, high)), []).append(error)

    print('Position error of each sighting that matched a visible artefact of the same type [m]: median / 90th percentile (n)')
    print(f'{"type":10s} ' + ' '.join(f'{f"{lo:.0f}-{hi:.0f} m":>22s}' for lo, hi in RANGE_BINS))
    for kind in ('alien', 'crystals', 'mushroom', 'sphere', 'ice'):
        row = []
        for bin_ in RANGE_BINS:
            e = errors.get((kind, bin_), [])
            row.append(f'{np.median(e):.2f} / {np.percentile(e, 90):.2f} ({len(e)})'.rjust(22) if len(e) else '-'.rjust(22))
        print(f'{kind:10s} ' + ' '.join(row))
    print(f'all matched sightings: median {np.median(all_errors):.2f} m, 90th percentile {np.percentile(all_errors, 90):.2f} m, n = {len(all_errors)}')

    print('\nMerged map: artefacts found against the real ones (a found artefact counts as right if within 3 m of a real one of that type)')
    real_by_type = {}
    for a in truth.values():
        real_by_type.setdefault(a['type'], []).append(a)
    for kind in ('alien', 'crystals', 'mushroom', 'sphere', 'ice'):
        items = [i for i in sightings.items if i['type'] == kind and i['count'] >= 3]
        real = real_by_type.get(kind, [])
        right = sum(1 for i in items if any(math.hypot(i['x'] - a['x'], i['y'] - a['y']) < 3.0 for a in real))
        covered = sum(1 for a in real if any(math.hypot(i['x'] - a['x'], i['y'] - a['y']) < 3.0 for i in items))
        print(f'{kind:10s} real {len(real)} | merged entries seen 3+ times: {len(items)} (right place: {right}, not near any real one: {len(items) - right}) | '
              f'real artefacts covered: {covered}/{len(real)}')


if __name__ == '__main__':
    main()
