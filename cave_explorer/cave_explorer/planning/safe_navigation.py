"""Safer goal choosing and stuck detection for exploration (Planning 1).

Written by Karan Sharma, using the ideas that kept the Advanced 6 sampling robot moving in the simulator:
  - only choose goals in open space that the robot can really reach,
  - keep goals well away from walls and unseen space,
  - choose by the length of the real path through free space, not the straight line,
  - remember goals that failed, and detect a robot that has stopped making progress.
It has no ROS dependency, so it can be tested without the simulator.
"""

import math

import cv2
import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra

from cave_explorer.advanced import cave_geometry

ROBOT_RADIUS = 0.5          # [m] the robot must fit
GOAL_CLEARANCE = 1.2        # [m] a goal must be this far from walls and unseen space
MIN_CLUSTER_CELLS = 5       # a frontier shorter than this (about 1.5 m) is ignored
BLACKLIST_RADIUS = 3.0      # [m] stay away from goals that failed
SEARCH_RADIUS = 3.0         # [m] look this far from a frontier for a safe goal
STUCK_TIME = 15.0           # [s] without moving this far, the robot counts as stuck
STUCK_DISTANCE = 0.5        # [m]


def travel_distances(accessible, start_cell, resolution):
    """Shortest path length [m] from start_cell to every accessible cell, through accessible cells only (inf elsewhere)"""
    height, width = accessible.shape
    index = np.full(accessible.shape, -1)
    index[accessible] = np.arange(accessible.sum())
    sources, targets, weights = [], [], []
    for d_row, d_col in ((0, 1), (1, 0), (1, 1), (1, -1)):
        a = index[:height - d_row, max(0, -d_col):width - max(0, d_col)]
        b = index[d_row:, max(0, d_col):width - max(0, -d_col)]
        connected = (a >= 0) & (b >= 0)
        sources.append(a[connected])
        targets.append(b[connected])
        weights.append(np.full(connected.sum(), resolution * np.hypot(d_row, d_col)))
    n = int(accessible.sum())
    graph = coo_matrix((np.concatenate(weights), (np.concatenate(sources), np.concatenate(targets))), shape=(n, n))
    distances = np.full(accessible.shape, np.inf)
    distances[accessible] = dijkstra(graph, directed=False, indices=index[start_cell])
    return distances


def choose_frontier(map_msg, robot_xy, blacklist, recent_goals=()):
    """Pick the next place to explore.

    A frontier is a free cell next to space that has not been seen. Neighbouring frontier cells are grouped, and for each
    group a safe goal is found inside known space nearby. The reachable goal with the shortest path wins, with a small
    preference for longer frontiers.

    Returns None if there is nothing safe and reachable left, otherwise a dict with:
      goal (x, y), heading, path (metres), size (metres of frontier), candidates (list of (x, y, size, path)).
    """
    info = map_msg.info
    resolution = info.resolution
    origin = (info.origin.position.x, info.origin.position.y)
    grid = np.array(map_msg.data, dtype=np.int8).reshape(info.height, info.width)

    clearance, confirmed, _, free, _ = cave_geometry.distance_transforms(grid, resolution)
    robot_cell = cave_geometry.world_to_cell(robot_xy[0], robot_xy[1], resolution, origin)
    accessible = cave_geometry.accessible_region(free, clearance, ROBOT_RADIUS, robot_cell)
    if not accessible.any():
        return None

    # Frontier cells: accessible cells with an unseen cell directly beside them
    unknown = grid < 0
    beside_unknown = np.zeros_like(unknown)
    beside_unknown[1:, :] |= unknown[:-1, :]
    beside_unknown[:-1, :] |= unknown[1:, :]
    beside_unknown[:, 1:] |= unknown[:, :-1]
    beside_unknown[:, :-1] |= unknown[:, 1:]
    frontier = accessible & beside_unknown
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(frontier.astype(np.uint8), connectivity=8)

    rows, cols = np.nonzero(accessible)
    nearest = np.argmin((rows - robot_cell[0]) ** 2 + (cols - robot_cell[1]) ** 2)
    path = travel_distances(accessible, (rows[nearest], cols[nearest]), resolution)

    # Cells that are safe to stand on: far from walls and unseen space, and reachable
    safe = accessible & (confirmed >= GOAL_CLEARANCE) & np.isfinite(path)
    safe_rows, safe_cols = np.nonzero(safe)
    if len(safe_rows) == 0:
        return None
    safe_xy = np.stack(cave_geometry.cell_to_world(safe_rows, safe_cols, resolution, origin), axis=1)

    avoid = [tuple(p) for p in blacklist] + [tuple(p) for p in recent_goals]
    candidates = []
    for label in range(1, count):
        size_cells = stats[label, cv2.CC_STAT_AREA]
        if size_cells < MIN_CLUSTER_CELLS:
            continue
        centre = cave_geometry.cell_to_world(centroids[label][1], centroids[label][0], resolution, origin)
        gaps = np.hypot(safe_xy[:, 0] - centre[0], safe_xy[:, 1] - centre[1])
        close = np.nonzero(gaps <= SEARCH_RADIUS)[0]
        if len(close) == 0:
            continue                                   # no safe place to stand near this frontier
        best = close[np.argmin(gaps[close])]
        goal = (float(safe_xy[best, 0]), float(safe_xy[best, 1]))
        if any(math.hypot(goal[0] - a, goal[1] - b) < BLACKLIST_RADIUS for a, b in avoid):
            continue
        travel = float(path[safe_rows[best], safe_cols[best]])
        candidates.append((goal[0], goal[1], size_cells * resolution, travel, centre))

    if not candidates:
        return None
    best = min(candidates, key=lambda c: c[3] - 2.0 * min(c[2], 5.0))
    heading = math.atan2(best[4][1] - best[1], best[4][0] - best[0])
    return {'goal': (best[0], best[1]), 'heading': heading, 'path': best[3], 'size': best[2],
            'candidates': [(c[0], c[1], c[2], c[3]) for c in candidates]}


class StuckWatchdog:
    """Says when the robot has not moved far enough for long enough while it has a goal."""

    def __init__(self):
        self.reference = None
        self.since = None

    def reset(self, xy, now):
        self.reference = xy
        self.since = now

    def is_stuck(self, xy, now):
        if self.reference is None:
            self.reset(xy, now)
            return False
        if math.hypot(xy[0] - self.reference[0], xy[1] - self.reference[1]) > STUCK_DISTANCE:
            self.reset(xy, now)
            return False
        return now - self.since > STUCK_TIME
