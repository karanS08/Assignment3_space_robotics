"""Record labelled camera images for stop-sign detection, from the simulator.

Teleports the rover to many viewpoints and saves the RGB and depth frames. The world file gives the five walls that carry the stop-sign photo,
so each image is labelled with which of them are really in view, and where they appear (the image columns of the wall's two edges).
Views are taken of: the stop-sign walls from both sides and at several distances and offsets; every other wall (this includes the
walls with the Mars timeline poster); the artefacts; and random places.

Needs the simulator running (cave_explorer_startup.launch.py).

For every stop sign there are exactly 5 photos, taken from its front on an arc at -56, -28, 0, +28 and +56 degrees from the sign's
front direction (28 degrees apart, inside a 140 degree cone). The sign is a photo on both faces of a flat panel, readable from one
face and mirrored from the other, so the front side of each sign is chosen by looking at a probe (mode 'probe') and written to
OUTPUT_DIR/front_sides.json as {"wall_97": 1, ...}, where 1 means the side the panel's normal points to and -1 the other.

Usage: python3 scripts/stop_capture.py OUTPUT_DIR [seed] [probe|fill|poster]
Mode 'poster' takes 3 photos (-28, 0, +28 degrees) of each Mars-timeline poster wall, from both sides: hard negative examples for stop-sign detectors.
Mode 'fill' adds only the arc photos that are missing from an existing capture, trying closer viewpoints than usual.
"""
import json
import math
import os
import re
import sys

import cv2
import numpy as np
import rclpy
from scipy import ndimage

from detector_capture import Capture
from offline_cave_map import artefacts_from_world, load_cave_grid
from wall_views import FOCAL, MARGIN, RES, line_clear, project, visible_stop_walls, walls_from_world



def main():
    out = sys.argv[1]
    probe = len(sys.argv) > 3 and sys.argv[3] == 'probe'
    fill = len(sys.argv) > 3 and sys.argv[3] == 'fill'
    poster = len(sys.argv) > 3 and sys.argv[3] == 'poster'
    old_records = json.load(open(os.path.join(out, 'labels.json'))) if fill else []
    have = {(re.search(r'(wall_\d+)', r['kind']).group(1), float(re.search(r'([+-]\d+)', r['kind']).group(1)))
            for r in old_records if r['kind'].startswith('arc')}
    rng = np.random.default_rng(int(sys.argv[2]) if len(sys.argv) > 2 else 0)
    os.makedirs(out, exist_ok=True)
    grid, origin = load_cave_grid(RES)
    wall_cells = grid >= 50
    clearance = ndimage.distance_transform_edt(~wall_cells) * RES
    walls = walls_from_world()
    stop_walls = [w for w in walls if w['stop']]
    print(f'{len(walls)} walls, {len(stop_walls)} carry the stop-sign photo', flush=True)

    def free(x, y):
        col, row = int((x - origin[0]) / RES), int((y - origin[1]) / RES)
        return 0 <= row < wall_cells.shape[0] and 0 <= col < wall_cells.shape[1] and clearance[row, col] >= 0.9

    def view_of_wall(w, side, distance, lateral, jitter):
        x = w['c'][0] + side * w['n'][0] * distance + lateral * w['u'][0]
        y = w['c'][1] + side * w['n'][1] * distance + lateral * w['u'][1]
        if not free(x, y):
            return None
        aim = math.atan2(w['c'][1] - y, w['c'][0] - x) + math.radians(jitter)
        return (x, y, aim)

    views = []
    # 1. The real stop signs: 5 photos each, on an arc in front, 28 degrees apart (or a probe of both sides)
    ARC = (-56.0, -28.0, 0.0, 28.0, 56.0)
    ARC_DISTANCES = (4.0, 3.5, 3.0, 2.5, 2.0) + ((1.6, 1.3, 1.0) if fill else ())   # the first distance at which the viewpoint is free and clear
    if poster:
        for w in walls:
            if not w['poster']:
                continue
            for side in (1, -1):
                for degrees in (-28.0, 0.0, 28.0):
                    angle = math.radians(degrees)
                    fx, fy = side * w['n'][0], side * w['n'][1]
                    dx, dy = fx * math.cos(angle) - fy * math.sin(angle), fx * math.sin(angle) + fy * math.cos(angle)
                    for distance in (4.0, 3.5, 3.0, 2.5, 2.0):
                        x, y = w['c'][0] + distance * dx, w['c'][1] + distance * dy
                        if free(x, y) and line_clear(wall_cells, (x, y), w['c'], origin, stop_short=0.9):
                            views.append((f'poster {w["name"]} {side:+d} {degrees:+.0f} {distance}', (x, y, math.atan2(w['c'][1] - y, w['c'][0] - x))))
                            break
    elif probe:
        for w in stop_walls:
            for side in (1, -1):
                for distance in ARC_DISTANCES:
                    v = view_of_wall(w, side, distance, 0.0, 0.0)
                    if v and line_clear(wall_cells, (v[0], v[1]), w['c'], origin, stop_short=0.9):
                        views.append((f'probe {w["name"]} {side}', v))
                        break
    else:
        front = json.load(open(os.path.join(out, 'front_sides.json')))
        for w in stop_walls:
            side = front[w['name']]
            for degrees in ARC:
                if (w['name'], degrees) in have:
                    continue
                angle = math.radians(degrees)
                fx, fy = side * w['n'][0], side * w['n'][1]
                dx, dy = fx * math.cos(angle) - fy * math.sin(angle), fx * math.sin(angle) + fy * math.cos(angle)
                for distance in ARC_DISTANCES:
                    x, y = w['c'][0] + distance * dx, w['c'][1] + distance * dy
                    if free(x, y) and line_clear(wall_cells, (x, y), w['c'], origin, stop_short=0.9):
                        views.append((f'arc {w["name"]} {degrees:+.0f} {distance}', (x, y, math.atan2(w['c'][1] - y, w['c'][0] - x))))
                        break
                else:
                    print(f'no free viewpoint for {w["name"]} at {degrees:+.0f} degrees', flush=True)
    n_stop = len(views)
    # 2. Every other wall, one or two views each (this includes the poster walls)
    for w in ([] if probe or fill or poster else walls):
        if w['stop']:
            continue
        for _ in range(2):
            for attempt in range(8):
                v = view_of_wall(w, rng.choice([-1, 1]), rng.uniform(2.5, 6.0), rng.uniform(-w['length'] / 3, w['length'] / 3), rng.uniform(-10, 10))
                if v and line_clear(wall_cells, (v[0], v[1]), w['c'], origin, stop_short=0.8):
                    views.append(('other wall', v))
                    break
    # 3. Around the artefacts
    for a in ([] if probe or fill or poster else artefacts_from_world()):
        for _ in range(2):
            angle = rng.uniform(0, 2 * math.pi)
            distance = rng.uniform(2.0, 5.0)
            x, y = a['x'] + distance * math.cos(angle), a['y'] + distance * math.sin(angle)
            if free(x, y):
                views.append(('artefact', (x, y, angle + math.pi + math.radians(rng.uniform(-20, 20)))))
    # 4. Random places
    for _ in range(0 if probe or fill or poster else 300):
        if sum(1 for k, _ in views if k == 'random') >= 40:
            break
        x = rng.uniform(origin[0], origin[0] + wall_cells.shape[1] * RES)
        y = rng.uniform(origin[1], origin[1] + wall_cells.shape[0] * RES)
        if free(x, y):
            views.append(('random', (x, y, rng.uniform(-math.pi, math.pi))))
    print(f'{len(views)} viewpoints: {n_stop} of the stop-sign walls, {sum(1 for k, _ in views if k == "other wall")} of other walls, '
          f'{sum(1 for k, _ in views if k == "artefact")} of artefacts, {sum(1 for k, _ in views if k == "random")} random', flush=True)

    rclpy.init()
    node = Capture()
    records = list(old_records)
    offset = len(old_records)
    for i, (kind, (x, y, yaw)) in enumerate(views):
        node.teleport(x, y, yaw)
        rgb, depth = node.grab()
        name = f'{i + offset:04d}'
        cv2.imwrite(os.path.join(out, name + '.png'), rgb)
        np.save(os.path.join(out, name + '_depth.npy'), depth.astype(np.float16))
        records.append({'id': name, 'kind': kind, 'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 3),
                        'stop_walls': visible_stop_walls(stop_walls, wall_cells, origin, x, y, yaw)})
        if i % 25 == 0:
            print(f'{i}/{len(views)}', flush=True)
            json.dump(records, open(os.path.join(out, 'labels.json'), 'w'))
    json.dump(records, open(os.path.join(out, 'labels.json'), 'w'))
    positives = sum(1 for r in records if r['stop_walls'])
    print(f'done: {len(records)} images, {positives} with a stop-sign wall in view, {len(records) - positives} without', flush=True)


if __name__ == '__main__':
    main()
