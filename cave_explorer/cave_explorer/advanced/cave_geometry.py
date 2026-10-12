"""Online distance-transform analysis: widest accessible area and narrow passages.

Pure numpy/OpenCV functions with no ROS dependency, so they can be tested
offline and reused by other parts (e.g. clearance_at() for goal validation).
The ROS wrapper is in cave_geometry_node.py.

Owner: Karan Sharma
Task: Advanced 2
"""

import math
from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy import ndimage

OCCUPIED_THRESHOLD = 50  # OccupancyGrid values are 0..100, -1 for unknown


@dataclass
class WideArea:
    x: float        # map frame [m]
    y: float        # map frame [m]
    radius: float   # largest circle of confirmed free space centred here [m]


@dataclass
class Passage:
    x: float            # map frame [m]
    y: float            # map frame [m]
    width: float        # wall-to-wall distance across the passage [m]
    wall_a: tuple       # (x, y) of the closest wall point on one side
    wall_b: tuple       # (x, y) of the closest wall point on the other side
    is_saddle: bool     # Hessian of the distance transform has eigenvalues of opposite sign
    kind: str           # 'doorway' (widens on both sides) or 'corridor' (widens on one side)


@dataclass
class GeometryResult:
    clearance: np.ndarray        # distance to nearest wall [m], per cell
    accessible: np.ndarray       # bool, free cells the robot can reach
    ridge: np.ndarray            # bool, medial axis of the accessible space
    widest: WideArea = None
    passages: list = field(default_factory=list)   # narrowest first


def cell_to_world(row, col, resolution, origin_xy):
    """Centre of a grid cell in the map frame"""
    return (origin_xy[0] + (col + 0.5) * resolution,
            origin_xy[1] + (row + 0.5) * resolution)


def world_to_cell(x, y, resolution, origin_xy):
    return (int(math.floor((y - origin_xy[1]) / resolution)),
            int(math.floor((x - origin_xy[0]) / resolution)))


def distance_transforms(grid, resolution):
    """Distance from each cell to the nearest wall, and to the nearest wall or unknown cell [m]"""
    occupied = grid >= OCCUPIED_THRESHOLD
    free = (grid >= 0) & ~occupied

    clearance, nearest_wall = ndimage.distance_transform_edt(~occupied, return_indices=True)
    confirmed = ndimage.distance_transform_edt(free)

    return clearance * resolution, confirmed * resolution, nearest_wall, free, occupied


def clearance_at(grid, resolution, origin_xy, x, y):
    """Distance from a map-frame point to the nearest wall [m]; 0 if not in known free space"""
    row, col = world_to_cell(x, y, resolution, origin_xy)
    if not (0 <= row < grid.shape[0] and 0 <= col < grid.shape[1]):
        return 0.0
    clearance, _, _, free, _ = distance_transforms(grid, resolution)
    return float(clearance[row, col]) if free[row, col] else 0.0


def accessible_region(free, clearance, robot_radius, robot_cell=None):
    """Free cells with enough clearance for the robot, connected to the robot's cell"""
    traversable = free & (clearance >= robot_radius)
    if not traversable.any():
        return traversable

    _, labels = cv2.connectedComponents(traversable.astype(np.uint8), connectivity=4)

    if robot_cell is None:
        # No pose available: use the largest region
        sizes = np.bincount(labels.ravel())
        sizes[0] = 0
        return labels == sizes.argmax()

    # The robot's own cell may be marked occupied or too tight, so use the nearest traversable cell
    rows, cols = np.nonzero(traversable)
    nearest = np.argmin((rows - robot_cell[0])**2 + (cols - robot_cell[1])**2)
    return labels == labels[rows[nearest], cols[nearest]]


def ridge_of(clearance, accessible, resolution, gradient_threshold=0.8):
    """Medial axis of the free space, from the gradient of the distance transform.

    Away from the medial axis the distance transform has gradient magnitude 1.
    On it, the nearest wall switches sides, the gradient is discontinuous and
    its finite-difference estimate drops towards 0.
    """
    d_row, d_col = np.gradient(clearance, resolution)
    return accessible & (np.hypot(d_row, d_col) < gradient_threshold)


def hessian_is_saddle(clearance, resolution, row, col):
    """True if the smoothed distance transform curves up one way and down the other at this cell"""
    half = 4
    r0, r1 = max(row - half, 0), min(row + half + 1, clearance.shape[0])
    c0, c1 = max(col - half, 0), min(col + half + 1, clearance.shape[1])
    patch = ndimage.gaussian_filter(clearance[r0:r1, c0:c1], sigma=1.0)

    d_row, d_col = np.gradient(patch, resolution)
    d_row_row, d_row_col = np.gradient(d_row, resolution)
    _, d_col_col = np.gradient(d_col, resolution)

    r, c = row - r0, col - c0
    hessian = np.array([[d_row_row[r, c], d_row_col[r, c]],
                        [d_row_col[r, c], d_col_col[r, c]]])
    eigenvalues = np.linalg.eigvalsh(hessian)
    return bool(eigenvalues[0] < 0.0 < eigenvalues[1])


def opposite_wall(occupied, cell, wall_cell, search_radius):
    """Closest wall cell on the far side of 'cell' from 'wall_cell', or None"""
    row, col = cell
    r0, r1 = max(row - search_radius, 0), min(row + search_radius + 1, occupied.shape[0])
    c0, c1 = max(col - search_radius, 0), min(col + search_radius + 1, occupied.shape[1])
    rows, cols = np.nonzero(occupied[r0:r1, c0:c1])
    if len(rows) == 0:
        return None

    offsets = np.stack([rows + r0 - row, cols + c0 - col], axis=1).astype(float)
    distances = np.linalg.norm(offsets, axis=1)
    to_wall = np.array([wall_cell[0] - row, wall_cell[1] - col], dtype=float)
    cosines = offsets @ to_wall / (distances * np.linalg.norm(to_wall) + 1e-9)

    # More than ~120 degrees away from the first wall point
    far_side = cosines < -0.5
    if not far_side.any():
        return None
    best = np.argmin(np.where(far_side, distances, np.inf))
    return (int(rows[best] + r0), int(cols[best] + c0))


def find_passages(clearance, nearest_wall, occupied, accessible, ridge, resolution, origin_xy,
                  widening_ratio=1.5, min_region_area=3.0, max_passages=5,
                  separation=3.0, window_radius=5.0, max_candidates=200):
    """Find bottlenecks: local minima of clearance along the medial axis that
    separate the nearby free space into two regions, at least one of them wider.

    Each candidate is tested by cutting the free space along the line between
    its two closest wall points (the critical line) inside a local window.
    """
    if not ridge.any():
        return []

    # Local minima of clearance along the ridge
    ridge_clearance = np.where(ridge, clearance, np.inf)
    window = 2 * int(round(1.0 / resolution)) + 1
    is_minimum = ridge & (ridge_clearance == ndimage.minimum_filter(ridge_clearance, size=window))
    rows, cols = np.nonzero(is_minimum)
    order = np.argsort(clearance[rows, cols])[:max_candidates]

    min_region_cells = min_region_area / resolution**2
    window_cells = int(round(window_radius / resolution))
    passages = []

    for row, col in zip(rows[order], cols[order]):
        x, y = cell_to_world(row, col, resolution, origin_xy)
        if any(math.hypot(x - p.x, y - p.y) < separation for p in passages):
            continue

        wall_a = (int(nearest_wall[0, row, col]), int(nearest_wall[1, row, col]))
        if wall_a == (row, col):
            continue
        search_radius = int(math.ceil(clearance[row, col] / resolution)) + 3
        wall_b = opposite_wall(occupied, (row, col), wall_a, search_radius)
        if wall_b is None:
            continue

        # Cut the local free space along the critical line
        r0, r1 = max(row - window_cells, 0), min(row + window_cells + 1, accessible.shape[0])
        c0, c1 = max(col - window_cells, 0), min(col + window_cells + 1, accessible.shape[1])
        local = accessible[r0:r1, c0:c1].astype(np.uint8)
        centre = (col - c0, row - r0)
        cv2.line(local, (wall_a[1] - c0, wall_a[0] - r0), centre, 0, 1)
        cv2.line(local, centre, (wall_b[1] - c0, wall_b[0] - r0), 0, 1)
        _, labels = cv2.connectedComponents(local, connectivity=4)

        # The regions touching the cut on either side, stepping along the passage direction
        across = np.array([wall_b[0] - wall_a[0], wall_b[1] - wall_a[1]], dtype=float)
        along = np.array([-across[1], across[0]]) / (np.linalg.norm(across) + 1e-9)
        sides = []
        for sign in (1.0, -1.0):
            label = 0
            for step in range(1, 5):
                r = int(round(row - r0 + sign * step * along[0]))
                c = int(round(col - c0 + sign * step * along[1]))
                if 0 <= r < labels.shape[0] and 0 <= c < labels.shape[1] and labels[r, c] > 0:
                    label = labels[r, c]
                    break
            sides.append(label)

        if 0 in sides or sides[0] == sides[1]:
            continue
        areas = np.bincount(labels.ravel())
        if min(areas[sides[0]], areas[sides[1]]) < min_region_cells:
            continue

        local_clearance = clearance[r0:r1, c0:c1]
        widest_each_side = ndimage.maximum(local_clearance, labels, index=sides)
        widens = [w >= widening_ratio * clearance[row, col] for w in widest_each_side]
        if not any(widens):
            continue

        point_a = cell_to_world(wall_a[0], wall_a[1], resolution, origin_xy)
        point_b = cell_to_world(wall_b[0], wall_b[1], resolution, origin_xy)
        passages.append(Passage(
            x=x, y=y,
            width=math.hypot(point_a[0] - point_b[0], point_a[1] - point_b[1]),
            wall_a=point_a, wall_b=point_b,
            is_saddle=hessian_is_saddle(clearance, resolution, row, col),
            kind='doorway' if all(widens) else 'corridor'))

        if len(passages) >= max_passages:
            break

    return sorted(passages, key=lambda p: p.width)


def analyse(grid, resolution, origin_xy, robot_xy=None, robot_radius=0.5, **passage_params):
    """Analyse an occupancy grid (2D int array, row = y, col = x)"""
    clearance, confirmed, nearest_wall, free, occupied = distance_transforms(grid, resolution)

    robot_cell = None if robot_xy is None else world_to_cell(robot_xy[0], robot_xy[1], resolution, origin_xy)
    accessible = accessible_region(free, clearance, robot_radius, robot_cell)
    ridge = ridge_of(clearance, accessible, resolution)
    result = GeometryResult(clearance=clearance, accessible=accessible, ridge=ridge)

    if not accessible.any():
        return result

    # Widest area: largest circle of confirmed free space, so unexplored space is not counted
    row, col = np.unravel_index(np.argmax(np.where(accessible, confirmed, -1.0)), confirmed.shape)
    x, y = cell_to_world(row, col, resolution, origin_xy)
    result.widest = WideArea(x=x, y=y, radius=float(confirmed[row, col]))

    result.passages = find_passages(clearance, nearest_wall, occupied, accessible, ridge,
                                    resolution, origin_xy, **passage_params)
    return result
