"""Record labelled camera images of the cave artefacts from the simulator.

Teleports the rover around every artefact at several distances and angles,
and saves the RGB and depth frames. The world file gives the true artefact
positions, so each frame is labelled with which artefact types are really in
view. Labels are used only to build and score detectors; the detectors
themselves see only the camera image.

Needs the simulator running (cave_explorer_startup.launch.py).

Usage: python3 scripts/detector_capture.py OUTPUT_DIR [seed]
"""
import json
import math
import os
import re
import subprocess
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from scipy import ndimage
from sensor_msgs.msg import Image

from offline_cave_map import PACKAGE, artefacts_from_world, load_cave_grid

RES = 0.3
IMAGE_WIDTH, IMAGE_HEIGHT = 720, 480
HFOV = 2.0944
FOCAL = (IMAGE_WIDTH / 2) / math.tan(HFOV / 2)
CAMERA_FORWARD = 0.5            # camera is 0.5 m ahead of the rover centre
MIN_WALL_CLEARANCE = 0.9        # [m]
VISIBLE_RANGE = (1.0, 9.0)      # depth camera sees 0.4 - 10 m


class Capture(Node):
    def __init__(self):
        super().__init__('detector_capture')
        self.bridge = CvBridge()
        self.rgb = None
        self.depth = None
        self.create_subscription(Image, 'camera/image', lambda m: setattr(self, 'rgb', m), 1)
        self.create_subscription(Image, 'camera/depth/image', lambda m: setattr(self, 'depth', m), 1)

    def teleport(self, x, y, yaw):
        request = (f'name: "mars_explorer", position: {{x: {x}, y: {y}, z: 0.4}}, '
                   f'orientation: {{x: 0, y: 0, z: {math.sin(yaw / 2)}, w: {math.cos(yaw / 2)}}}')
        subprocess.run(['ign', 'service', '-s', '/world/mars_cave/set_pose', '--reqtype', 'ignition.msgs.Pose',
                        '--reptype', 'ignition.msgs.Boolean', '--timeout', '3000', '--req', request],
                       capture_output=True, check=True)

    def grab(self, settle=1.0):
        """Fresh RGB and depth frames, taken after the rover has settled"""
        self.rgb = self.depth = None
        end = time.time() + settle
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.rgb = self.depth = None
        end = time.time() + 5.0
        while (self.rgb is None or self.depth is None) and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        rgb = self.bridge.imgmsg_to_cv2(self.rgb, 'passthrough')
        depth = self.bridge.imgmsg_to_cv2(self.depth, 'passthrough')
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), depth


def line_clear(wall, a, b, origin):
    """No wall cell between world points a and b"""
    steps = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (RES / 2)))
    for t in np.linspace(0.0, 1.0, steps):
        col = int((a[0] + t * (b[0] - a[0]) - origin[0]) / RES)
        row = int((a[1] + t * (b[1] - a[1]) - origin[1]) / RES)
        if not (0 <= row < wall.shape[0] and 0 <= col < wall.shape[1]) or wall[row, col]:
            return False
    return True


def visible_artefacts(artefacts, wall, origin, x, y, yaw):
    """Artefacts in front of the camera, in range, with a clear line of sight. Gives column and range"""
    cam = (x + CAMERA_FORWARD * math.cos(yaw), y + CAMERA_FORWARD * math.sin(yaw))
    visible = []
    for a in artefacts:
        dx, dy = a['x'] - cam[0], a['y'] - cam[1]
        forward = dx * math.cos(yaw) + dy * math.sin(yaw)
        left = -dx * math.sin(yaw) + dy * math.cos(yaw)
        distance = math.hypot(dx, dy)
        if forward <= 0.2 or not VISIBLE_RANGE[0] <= distance <= VISIBLE_RANGE[1]:
            continue
        column = IMAGE_WIDTH / 2 - FOCAL * left / forward
        if not 20 <= column <= IMAGE_WIDTH - 20:
            continue
        if not line_clear(wall, cam, (a['x'], a['y']), origin):
            continue
        visible.append({'name': a['name'], 'type': a['type'], 'column': round(column, 1), 'range': round(distance, 2)})
    return visible


def main():
    out = sys.argv[1]
    rng = np.random.default_rng(int(sys.argv[2]) if len(sys.argv) > 2 else 0)
    os.makedirs(out, exist_ok=True)

    grid, origin = load_cave_grid(RES)
    wall = grid >= 50
    clearance = ndimage.distance_transform_edt(~wall) * RES
    artefacts = artefacts_from_world()

    def free(x, y):
        col, row = int((x - origin[0]) / RES), int((y - origin[1]) / RES)
        return 0 <= row < wall.shape[0] and 0 <= col < wall.shape[1] and clearance[row, col] >= MIN_WALL_CLEARANCE

    # Viewpoints: around every artefact at several distances, angles and aim offsets
    views = []
    for a in artefacts:
        for distance in (2.0, 3.0, 4.5, 6.5):
            for approach in np.radians(np.arange(0, 360, 40) + rng.uniform(0, 40)):
                x = a['x'] + distance * math.cos(approach)
                y = a['y'] + distance * math.sin(approach)
                if not free(x, y):
                    continue
                aim = math.pi + approach + math.radians(rng.choice([-30, -15, 0, 15, 30]))
                views.append((x, y, aim))

    # Background-only viewpoints: free places where no artefact is in view
    attempts = 0
    background = 0
    while background < 60 and attempts < 5000:
        attempts += 1
        x = rng.uniform(origin[0], origin[0] + wall.shape[1] * RES)
        y = rng.uniform(origin[1], origin[1] + wall.shape[0] * RES)
        yaw = rng.uniform(-math.pi, math.pi)
        if free(x, y) and not visible_artefacts(artefacts, wall, origin, x, y, yaw):
            views.append((x, y, yaw))
            background += 1

    order = rng.permutation(len(views))
    views = [views[i] for i in order][:int(os.environ.get('MAX_VIEWS', 400))]
    print(f'{len(views)} viewpoints ({background} background)', flush=True)

    rclpy.init()
    node = Capture()
    records = []
    for i, (x, y, yaw) in enumerate(views):
        node.teleport(x, y, yaw)
        rgb, depth = node.grab()
        name = f'{i:04d}'
        cv2.imwrite(os.path.join(out, name + '.png'), rgb)
        np.save(os.path.join(out, name + '_depth.npy'), depth.astype(np.float16))
        records.append({'id': name, 'x': round(x, 2), 'y': round(y, 2), 'yaw': round(yaw, 3),
                        'visible': visible_artefacts(artefacts, wall, origin, x, y, yaw)})
        if i % 25 == 0:
            print(f'{i}/{len(views)}', flush=True)
            json.dump(records, open(os.path.join(out, 'labels.json'), 'w'))
    json.dump(records, open(os.path.join(out, 'labels.json'), 'w'))
    counts = {}
    for r in records:
        for v in r['visible']:
            counts[v['type']] = counts.get(v['type'], 0) + 1
    print('images with each type in view:', counts, '| none in view:', sum(1 for r in records if not r['visible']))


if __name__ == '__main__':
    main()
