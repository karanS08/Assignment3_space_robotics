"""Train the box classifier (perception/box_classifier.py) that rejects false colour detections, such as poster trees.

Candidate boxes come from the colour detector run on three sets of simulator captures:
  - sim_artefacts_final: the rover around every artefact, labelled with which artefacts are in view (labels.json 'visible')
  - sim_stop and sim_poster: the rover in front of the stop-sign and timeline posters; every box there is a negative unless
    a real artefact of the same type is projected into that part of the image (those boxes are left out as unsure)
A box is positive if a visible artefact of its type lies within its columns.

Scored with grouped cross-validation: images from the same 10 m cell of the cave are kept in the same fold, so the score is for
places the model has not seen. The final forest is saved as plain arrays for numpy.

Usage: python3 scripts/box_classifier_train.py [DATASET_ROOT]
"""
import json
import math
import os
import sys

import cv2
import numpy as np
from sklearn.ensemble import RandomForestClassifier
from sklearn.model_selection import GroupKFold

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cave_explorer'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from cave_explorer.perception import artefact_detector                  # noqa: E402
from cave_explorer.perception.box_classifier import KINDS, MODEL_PATH, box_features   # noqa: E402
from offline_cave_map import artefacts_from_world                       # noqa: E402

ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser('~/space_robotics_sim/dataset')
FOCAL = 360.0 / math.tan(math.radians(60.0))
MARGIN = 25


def projected_columns(r, world):
    """Image column of every world artefact in front of the camera within 12 m (line of sight not checked)"""
    cx, cy = r['x'] + 0.5 * math.cos(r['yaw']), r['y'] + 0.5 * math.sin(r['yaw'])
    out = []
    for a in world:
        dx, dy = a['x'] - cx, a['y'] - cy
        forward = dx * math.cos(r['yaw']) + dy * math.sin(r['yaw'])
        left = -dx * math.sin(r['yaw']) + dy * math.cos(r['yaw'])
        if 0.3 < forward and math.hypot(dx, dy) < 12.0:
            out.append((a['type'], 360.0 - FOCAL * left / forward))
    return out


def collect():
    world = artefacts_from_world()
    X, y, groups, kinds = [], [], [], []
    for name in ('sim_artefacts_final', 'sim_stop', 'sim_poster'):
        folder = os.path.join(ROOT, name)
        for r in json.load(open(os.path.join(folder, 'labels.json'))):
            image = cv2.imread(os.path.join(folder, r['id'] + '.png'))
            depth = np.load(os.path.join(folder, r['id'] + '_depth.npy')).astype(np.float32)
            for d in artefact_detector.detect(image):
                inside = lambda col: d.x - MARGIN <= col <= d.x + d.width + MARGIN
                if 'visible' in r:
                    label = any(v['type'] == d.kind and inside(v['column']) for v in r['visible'])
                    if not label and any(t == d.kind and inside(c) for t, c in projected_columns(r, world)):
                        continue                      # an artefact of this type is there but was not labelled visible
                else:
                    if any(t == d.kind and inside(c) for t, c in projected_columns(r, world)):
                        continue
                    label = False
                X.append(box_features(image, depth, (d.x, d.y, d.width, d.height), d.kind))
                y.append(label)
                groups.append(f'{int(r["x"] // 10)},{int(r["y"] // 10)}')
                kinds.append(d.kind)
    return np.array(X), np.array(y), np.array(groups), np.array(kinds)


def export(forest, path):
    feature, threshold, left, right, value, roots = [], [], [], [], [], []
    for est in forest.estimators_:
        t = est.tree_
        base = len(feature)
        roots.append(base)
        feature += list(t.feature)
        threshold += list(t.threshold)
        left += [c + base if c >= 0 else -1 for c in t.children_left]
        right += [c + base if c >= 0 else -1 for c in t.children_right]
        v = t.value[:, 0, :]
        value += list(v[:, 1] / v.sum(axis=1))
    np.savez_compressed(path, feature=np.array(feature, np.int32), threshold=np.array(threshold, np.float32),
                        left=np.array(left, np.int32), right=np.array(right, np.int32),
                        value=np.array(value, np.float32), roots=np.array(roots, np.int32))


def main():
    X, y, groups, kinds = collect()
    print(f'{len(y)} boxes: {y.sum()} real artefacts, {(~y).sum()} false')
    model = lambda: RandomForestClassifier(n_estimators=60, max_depth=8, min_samples_leaf=2, class_weight='balanced',
                                           random_state=0, n_jobs=1)
    p = np.zeros(len(y))
    for train, test in GroupKFold(n_splits=5).split(X, y, groups):
        p[test] = model().fit(X[train], y[train]).predict_proba(X[test])[:, 1]
    print('grouped 5-fold cross-validation, threshold 0.5:')
    print(f'{"type":9s} {"real kept":>14s} {"false removed":>16s}')
    for k in KINDS + ('all',):
        sel = np.ones(len(y), bool) if k == 'all' else kinds == k
        tp, fp = sel & y, sel & ~y
        print(f'{k:9s} {np.sum(p[tp] >= 0.5):5d}/{tp.sum():<4d} {np.mean(p[tp] >= 0.5) * 100:3.0f}%'
              f'  {np.sum(p[fp] < 0.5):5d}/{fp.sum():<4d} {np.mean(p[fp] < 0.5) * 100:3.0f}%')
    export(model().fit(X, y), MODEL_PATH)
    print('saved', MODEL_PATH)


if __name__ == '__main__':
    main()
