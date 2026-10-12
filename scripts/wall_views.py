"""Helpers for working out which walls are in view of the camera (no ROS needed).

Every wall in the world file is a flat panel with one texture. Five carry the stop-sign photo, seven carry the Mars timeline poster
(texture wall07.jpg, the one that fools the template's stop-sign detector), and the rest are plain rock.
"""
import math
import os
import re

import numpy as np

from offline_cave_map import PACKAGE

RES = 0.3
FOCAL = 360.0 / math.tan(2.0944 / 2)
CAMERA_FORWARD = 0.5
MARGIN = 40


def walls_from_world():
    """All walls: name, centre, direction along the wall, normal, length, and whether it carries the stop-sign photo"""
    sdf = open(os.path.join(PACKAGE, 'worlds', 'mars_cave.sdf')).read()
    walls = []
    for block in re.split(r'(?=<model name="wall_)', sdf):
        m = re.match(r'<model name="(wall_\d+)">\s*<pose>([^<]+)</pose>', block)
        if not m:
            continue
        body = block.split('</model>')[0]
        size = re.search(r'<box>\s*<size>([^<]+)</size>', body)
        x, y, _, _, _, yaw = map(float, m.group(2).split())
        sx, sy, _ = map(float, size.group(1).split())
        length = sx if abs(sx - 4) > 1e-6 else sy
        walls.append({'name': m.group(1), 'c': (x, y), 'u': (math.cos(yaw), math.sin(yaw)), 'n': (-math.sin(yaw), math.cos(yaw)),
                      'length': length, 'stop': 'stop_sign' in body,
                      'poster': 'wall07.jpg' in body})
    return walls


def line_clear(wall, a, b, origin, stop_short=0.0):
    """No wall cell between a and b (the line may stop a little short of b, where b is itself a wall)"""
    length = math.hypot(b[0] - a[0], b[1] - a[1])
    end = max(0.0, 1.0 - stop_short / max(length, 1e-6))
    for t in np.linspace(0.0, end, max(2, int(length / (RES / 2)))):
        col = int((a[0] + t * (b[0] - a[0]) - origin[0]) / RES)
        row = int((a[1] + t * (b[1] - a[1]) - origin[1]) / RES)
        if not (0 <= row < wall.shape[0] and 0 <= col < wall.shape[1]) or wall[row, col]:
            return False
    return True


def project(point, x, y, yaw):
    """Image column and range of a world point for the camera, or None if it is behind the camera"""
    cam = (x + CAMERA_FORWARD * math.cos(yaw), y + CAMERA_FORWARD * math.sin(yaw))
    dx, dy = point[0] - cam[0], point[1] - cam[1]
    forward = dx * math.cos(yaw) + dy * math.sin(yaw)
    left = -dx * math.sin(yaw) + dy * math.cos(yaw)
    if forward <= 0.2:
        return None
    return 360.0 - FOCAL * left / forward, math.hypot(dx, dy), cam


def visible_stop_walls(stop_walls, grid_wall, origin, x, y, yaw):
    out = []
    for w in stop_walls:
        centre = project(w['c'], x, y, yaw)
        if centre is None or not 1.0 <= centre[1] <= 10.0:
            continue
        if not line_clear(grid_wall, centre[2], w['c'], origin, stop_short=0.9):
            continue
        edges = []
        for sign in (-1, 1):
            end = (w['c'][0] + sign * w['u'][0] * w['length'] / 2, w['c'][1] + sign * w['u'][1] * w['length'] / 2)
            p = project(end, x, y, yaw)
            edges.append(p[0] if p else (-1000.0 if sign < 0 else 1720.0))
        left, right = min(edges), max(edges)
        if right < -MARGIN or left > 720 + MARGIN or (right - left) < 15:
            continue
        # How much of the wall is inside the picture, as a share of its width
        inside = (min(right, 720.0) - max(left, 0.0)) / max(right - left, 1.0)
        out.append({'wall': w['name'], 'range': round(centre[1], 2), 'left': round(left, 1), 'right': round(right, 1),
                    'inside': round(max(inside, 0.0), 2)})
    return out


def sign_in_direction(stop_walls, grid_wall, origin, x, y, yaw, box, max_range=60.0):
    """True if a real stop-sign wall lies in the direction of an image box, in line of sight, at any distance up to max_range.

    A detection that fits this is a real sign that is simply far away or only partly in view, not a false alarm.
    """
    centre = box[0] + box[2] / 2.0
    for w in stop_walls:
        p = project(w['c'], x, y, yaw)
        if p is None or p[1] > max_range or not line_clear(grid_wall, p[2], w['c'], origin, stop_short=0.9):
            continue
        columns = []
        for sign in (-1, 1):
            end = (w['c'][0] + sign * w['u'][0] * w['length'] / 2, w['c'][1] + sign * w['u'][1] * w['length'] / 2)
            q = project(end, x, y, yaw)
            columns.append(q[0] if q else (-1000.0 if sign < 0 else 1720.0))
        if min(columns) - MARGIN <= centre <= max(columns) + MARGIN:
            return True
    return False
