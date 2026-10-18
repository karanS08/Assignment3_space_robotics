"""Estimate map-frame artefact positions and merge repeated detections.

Owner: Andrew Than
Task: Perception 3

Method: the bounding-box centre gives the bearing of the artefact, the depth image gives its distance, and the robot pose
turns both into a map position. A new position within 2 m of a known artefact of the same type is merged into it with a
running average; otherwise it is a new artefact.

Corrected from the first version (commit 4f1b435), which was a set of fragments pasted from cave_explorer.py:
- depth was read at the single centre pixel, which often hits the wall behind a thin artefact; now the median of the
  nearest 30 % of depth readings in the box is used
- the bearing grew linearly with the pixel offset, which is up to about 11 degrees wrong for a 120 degree lens; now atan
- the depth camera measures forward (planar) distance, not distance along the ray; it is now used as the forward distance
- the merge was (old + new) / 2, so the latest detection always counted for half; now a running average with a count
- artefacts of different types could merge into one marker; now only the same type merges
- the camera is 0.5 m ahead of the robot centre; this offset is now included

No ROS dependency, so it can be tested without the simulator.
"""
import math

import numpy as np

CAMERA_FOV = math.radians(120.0)    # horizontal field of view of the simulated camera
CAMERA_OFFSET = 0.5                 # camera position ahead of the robot centre (m)
MIN_DEPTH, MAX_DEPTH = 0.5, 9.9     # depth readings outside this range are invalid (m)
MERGE_DISTANCE = 2.0                # detections closer than this are the same artefact (m)
MERGE_DISTANCES = {'crystals': 3.0, 'ice': 5.0}  # a crystal cluster is several metres wide, an ice formation about 5 m


def box_depth(depth_image, box):
    """Forward distance in metres to the object in box (x, y, w, h), or None if the box has too few valid readings.

    The box also contains rock behind the artefact, so the nearest 30 % of readings are used.
    """
    x, y, w, h = box
    patch = np.asarray(depth_image, dtype=np.float32)[max(y, 0):y + h, max(x, 0):x + w].ravel()
    patch = patch[np.isfinite(patch) & (patch > MIN_DEPTH) & (patch < MAX_DEPTH)]
    if len(patch) < 20:
        return None
    patch.sort()
    return float(np.median(patch[:max(10, int(0.3 * len(patch)))]))


def pixel_bearing(centre_x, image_width, fov=CAMERA_FOV):
    """Angle of the pixel column from the camera axis in radians, positive to the left."""
    focal = (image_width / 2.0) / math.tan(fov / 2.0)
    return math.atan2(image_width / 2.0 - centre_x, focal)


def artefact_map_position(robot_x, robot_y, robot_theta, bearing, forward):
    """Map (x, y) of an artefact at the given bearing and forward (planar) distance from the camera."""
    camera_x = robot_x + CAMERA_OFFSET * math.cos(robot_theta)
    camera_y = robot_y + CAMERA_OFFSET * math.sin(robot_theta)
    left = forward * math.tan(bearing)
    return (camera_x + forward * math.cos(robot_theta) - left * math.sin(robot_theta),
            camera_y + forward * math.sin(robot_theta) + left * math.cos(robot_theta))


def localise(robot_x, robot_y, robot_theta, box, depth_image):
    """Map (x, y) of the artefact in box, or None if its depth cannot be measured."""
    forward = box_depth(depth_image, box)
    if forward is None:
        return None
    bearing = pixel_bearing(box[0] + box[2] / 2.0, depth_image.shape[1])
    return artefact_map_position(robot_x, robot_y, robot_theta, bearing, forward)


class ArtefactMap:
    """Known artefacts, one entry per physical artefact: {'type', 'x', 'y', 'count'}."""

    def __init__(self, merge_distance=MERGE_DISTANCE, merge_distances=MERGE_DISTANCES):
        self.merge_distance_ = merge_distance
        self.merge_distances_ = merge_distances
        self.artefacts_ = []

    def add(self, artefact_type, x, y):
        """Add one detection; return the index of the artefact it was merged into, or of the new artefact."""
        for i, known in enumerate(self.artefacts_):
            limit = self.merge_distances_.get(artefact_type, self.merge_distance_)
            if known['type'] == artefact_type and math.hypot(x - known['x'], y - known['y']) < limit:
                known['count'] += 1
                known['x'] += (x - known['x']) / known['count']    # running average of all detections
                known['y'] += (y - known['y']) / known['count']
                return i
        self.artefacts_.append({'type': artefact_type, 'x': x, 'y': y, 'count': 1})
        return len(self.artefacts_) - 1
