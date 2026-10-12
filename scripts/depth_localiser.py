"""Where in the cave is a detected artefact? (Perception 3, stand-in written by Karan while the Perception part was not yet shared.)

For a detection box, the depth camera gives how far away the artefact is and the box's column gives the direction, so the position
in the camera frame follows, and the robot's pose turns it into a map position. Sightings of the same artefact are merged by
type and distance.

No ROS dependency. Copied next to the detectors in the package when it is wired in.
"""
import math

import numpy as np

FOCAL = 360.0 / math.tan(2.0944 / 2)        # pixels; 720 px wide image, 120 degree field of view
CAMERA_FORWARD = 0.5                        # [m] the camera is 0.5 m ahead of the robot centre
DEPTH_RANGE = (0.5, 9.9)                    # [m] values outside this are not trusted
NEAREST_FRACTION = 0.3                      # use the nearest 30 % of depth readings in the box: the artefact, not the wall behind it


def box_range(depth, box):
    """Distance to the artefact in a box (x, y, width, height), or None if there is no usable depth reading"""
    x, y, w, h = box
    patch = np.asarray(depth, dtype=np.float32)[max(y, 0):y + h, max(x, 0):x + w].ravel()
    patch = patch[np.isfinite(patch) & (patch > DEPTH_RANGE[0]) & (patch < DEPTH_RANGE[1])]
    if len(patch) < 20:
        return None
    patch.sort()
    return float(np.median(patch[:max(10, int(NEAREST_FRACTION * len(patch)))]))


def localise(box, depth, robot_x, robot_y, robot_yaw):
    """Map position (x, y) of the artefact in a box, and its range from the camera, or None"""
    distance = box_range(depth, box)
    if distance is None:
        return None
    column = box[0] + box[2] / 2.0
    left = distance * (360.0 - column) / FOCAL
    cam_x = robot_x + CAMERA_FORWARD * math.cos(robot_yaw)
    cam_y = robot_y + CAMERA_FORWARD * math.sin(robot_yaw)
    x = cam_x + distance * math.cos(robot_yaw) - left * math.sin(robot_yaw)
    y = cam_y + distance * math.sin(robot_yaw) + left * math.cos(robot_yaw)
    return x, y, distance


class ArtefactMap:
    """Merges repeated sightings: a new sighting joins the nearest artefact of the same type within merge_radius."""

    def __init__(self, merge_radius=2.5):
        self.merge_radius = merge_radius
        self.items = []     # dicts: type, x, y, count

    def add(self, kind, x, y):
        best = None
        for item in self.items:
            if item['type'] == kind:
                gap = math.hypot(item['x'] - x, item['y'] - y)
                if gap < self.merge_radius and (best is None or gap < best[0]):
                    best = (gap, item)
        if best is None:
            self.items.append({'type': kind, 'x': x, 'y': y, 'count': 1})
        else:
            item = best[1]
            item['x'] = (item['x'] * item['count'] + x) / (item['count'] + 1)
            item['y'] = (item['y'] * item['count'] + y) / (item['count'] + 1)
            item['count'] += 1
