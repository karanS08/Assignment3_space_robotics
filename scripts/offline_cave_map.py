"""Build an occupancy grid of the cave from worlds/mars_cave.sdf, for offline checks without ROS."""
import math
import os
import re
import sys

import cv2
import numpy as np

PACKAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'cave_explorer')
sys.path.insert(0, PACKAGE)


def load_cave_grid(resolution=0.3, margin=6.0):
    """Rasterise the cave walls. Returns (grid, origin_xy): grid is 0 free, 100 wall; row = y, col = x"""
    sdf = open(os.path.join(PACKAGE, 'worlds', 'mars_cave.sdf')).read()

    # Each wall is a thin 4 m tall panel: centre pose (x, y, yaw) and a length
    walls = []
    for m in re.finditer(r'<model name="wall_\d+">\s*<pose>([^<]+)</pose>.*?<size>([^<]+)</size>', sdf, re.S):
        x, y, _, _, _, yaw = map(float, m.group(1).split())
        size_x, size_y, _ = map(float, m.group(2).split())
        walls.append((x, y, yaw, size_x if abs(size_x - 4) > 1e-6 else size_y))

    origin_x = min(w[0] for w in walls) - margin
    origin_y = min(w[1] for w in walls) - margin
    width = int((max(w[0] for w in walls) + margin - origin_x) / resolution)
    height = int((max(w[1] for w in walls) + margin - origin_y) / resolution)

    image = np.zeros((height, width), np.uint8)
    for x, y, yaw, length in walls:
        dx, dy = length / 2 * math.cos(yaw), length / 2 * math.sin(yaw)
        a = (int((x - dx - origin_x) / resolution), int((y - dy - origin_y) / resolution))
        b = (int((x + dx - origin_x) / resolution), int((y + dy - origin_y) / resolution))
        cv2.line(image, a, b, 255, 1)

    grid = np.zeros((height, width), np.int8)
    grid[image > 0] = 100
    return grid, (origin_x, origin_y)


def artefacts_from_world():
    sdf = open(os.path.join(PACKAGE, 'worlds', 'mars_cave.sdf')).read()
    found = []
    for m in re.finditer(r'<model name="(artifact_\d+)">\s*<pose>([^<]+)</pose>.*?<uri>[^<]*/artifacts/([^/]+)/', sdf, re.S):
        x, y, z = (float(v) for v in m.group(2).split()[:3])
        kind = {'green_alien': 'alien', 'green_crystals_game_ready_asset': 'crystals',
                'white_sphere': 'sphere', 'mushrooms_blue': 'mushroom',
                'ice_castles_ny__ice_formations': 'ice'}.get(m.group(3), m.group(3))
        found.append({'name': m.group(1), 'type': kind, 'x': x, 'y': y})
    return found
