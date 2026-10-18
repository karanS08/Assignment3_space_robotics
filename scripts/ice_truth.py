"""Where the ice is really visible in a captured image.

Projects the ice mesh from the world into the camera and keeps the points
whose distance agrees with the depth image, so wall-hidden parts are dropped.
Used only to label and score; the detector never sees it.
"""
import math
import os
import re

import numpy as np

from offline_cave_map import PACKAGE

IMAGE_WIDTH, IMAGE_HEIGHT = 720, 480
FOCAL = (IMAGE_WIDTH / 2) / math.tan(2.0944 / 2)
CAMERA_FORWARD = 0.5
DEPTH_TOLERANCE = 0.35


def _rotation(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    return (np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]) @ np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
            @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))


def ice_points(stride=3):
    """World-frame points on every ice formation in the cave (every `stride`-th mesh vertex)"""
    obj = os.path.join(PACKAGE, 'worlds', 'models', 'artifacts', 'ice_castles_ny__ice_formations',
                       'ice_castles_ny__ice_formations.obj')
    vertices = np.array([[float(t) for t in line.split()[1:4]] for line in open(obj) if line.startswith('v ')])[::stride]
    sdf = open(os.path.join(PACKAGE, 'worlds', 'mars_cave.sdf')).read()
    points = []
    for block in re.split(r'(?=<model name="artifact_)', sdf):
        m = re.match(r'<model name="artifact_\d+">\s*<pose>([^<]+)</pose>', block)
        if m and 'ice_castles' in block.split('</model>')[0]:
            x, y, z, roll, pitch, yaw = map(float, m.group(1).split())
            points.append((_rotation(roll, pitch, yaw) @ (0.8 * vertices).T).T + [x, y, z])
    return np.concatenate(points)


def project(points, x, y, yaw, camera_height):
    """Pixel column, row and range of world points for a rover at (x, y, yaw); only points in front of the camera"""
    cam = np.array([x + CAMERA_FORWARD * math.cos(yaw), y + CAMERA_FORWARD * math.sin(yaw), camera_height])
    d = points - cam
    forward = d[:, 0] * math.cos(yaw) + d[:, 1] * math.sin(yaw)
    left = -d[:, 0] * math.sin(yaw) + d[:, 1] * math.cos(yaw)
    keep = forward > 0.3
    forward, left, up = forward[keep], left[keep], d[keep, 2]
    col = IMAGE_WIDTH / 2 - FOCAL * left / forward
    row = IMAGE_HEIGHT / 2 - FOCAL * up / forward
    inside = (col >= 0) & (col < IMAGE_WIDTH) & (row >= 0) & (row < IMAGE_HEIGHT)
    return col[inside].astype(int), row[inside].astype(int), forward[inside]


def visible_mask(points, record, depth, camera_height):
    """Boolean image: True where the ice surface is seen (point range matches the depth image)"""
    col, row, rng = project(points, record['x'], record['y'], record['yaw'], camera_height)
    mask = np.zeros(depth.shape, bool)
    ok = np.abs(depth[row, col] - rng) < DEPTH_TOLERANCE
    mask[row[ok], col[ok]] = True
    return mask
