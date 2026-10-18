"""Gaussian Process model of the measurement field and adaptive choice of sampling locations.

No ROS dependency. The sampler never sees the measurement field itself: it is
given measurements one at a time through add_sample() and decides where to go
next from those measurements and the current map only.
The ROS wrapper is in adaptive_sampling_node.py.

Owner: Karan Sharma
Task: Advanced 6
"""

import warnings

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import RBF, ConstantKernel, WhiteKernel

from cave_explorer.advanced import cave_geometry

STRATEGIES = ('adaptive', 'random')


def travel_distances(accessible, start_cell, resolution):
    """Shortest path length through accessible cells from start_cell to every cell [m].

    Unreachable and inaccessible cells are inf.
    """
    height, width = accessible.shape
    index = np.full(accessible.shape, -1)
    index[accessible] = np.arange(accessible.sum())

    # 8-connected grid graph
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


class AdaptiveSampler:
    def __init__(self, strategy='adaptive', candidate_spacing=1.0, min_sample_spacing=4.0,
                 goal_clearance=1.5, travel_weight=0.3, robot_radius=0.5,
                 blacklist_spacing=4.0, seed=None):
        """
        strategy: 'adaptive' goes where the GP is most uncertain, discounted by travel distance.
                  'random' is the baseline: a uniformly random reachable candidate.
        candidate_spacing: spacing of the candidate goal locations [m]
        min_sample_spacing: never sample closer than this to an existing sample [m]
        goal_clearance: minimum distance from a goal to walls and unknown space, so Nav2 can reach it [m]
        travel_weight: how strongly travel distance is penalised, 0 = ignore distance
        blacklist_spacing: never choose a goal closer than this to one that could not be reached [m]
        """
        if strategy not in STRATEGIES:
            raise ValueError(f'Unknown strategy {strategy}, expected one of {STRATEGIES}')

        self.strategy_ = strategy
        self.candidate_spacing_ = candidate_spacing
        self.min_sample_spacing_ = min_sample_spacing
        self.goal_clearance_ = goal_clearance
        self.travel_weight_ = travel_weight
        self.robot_radius_ = robot_radius
        self.blacklist_spacing_ = blacklist_spacing
        self.rng_ = np.random.default_rng(seed)

        self.sample_xy_ = []        # [x, y] of each measurement
        self.sample_values_ = []    # measured value at each location
        self.blacklist_xy_ = []     # goals that could not be reached
        self.gp_ = None

    def add_sample(self, x, y, value):
        """Record a measurement taken by the robot and update the GP"""
        self.sample_xy_.append([x, y])
        self.sample_values_.append(value)
        self.fit()

    def blacklist(self, x, y):
        """Do not choose goals near this location again"""
        self.blacklist_xy_.append([x, y])

    def fit(self):
        # Signal variance * smoothness over metres + measurement noise
        kernel = (ConstantKernel(1.0, (1e-2, 1e2)) * RBF(10.0, (5.0, 60.0))
                  + WhiteKernel(0.1, (1e-3, 1.0)))

        # With few samples the hyperparameters cannot be estimated reliably: the fit tends to
        # pick a very short length scale, which makes everywhere look equally uncertain.
        # Keep the prior ones until there is enough data
        optimizer = 'fmin_l_bfgs_b' if len(self.sample_values_) >= 8 else None

        self.gp_ = GaussianProcessRegressor(kernel=kernel, normalize_y=True, optimizer=optimizer,
                                            n_restarts_optimizer=2,
                                            random_state=int(self.rng_.integers(2**31)))
        with warnings.catch_warnings():
            # Hyperparameters landing on their bounds is expected early on
            warnings.simplefilter('ignore')
            self.gp_.fit(np.array(self.sample_xy_), np.array(self.sample_values_))

    def predict(self, xy):
        """GP mean and standard deviation at an (N, 2) array of map-frame points"""
        xy = np.atleast_2d(xy)
        if self.gp_ is None:
            return np.zeros(len(xy)), np.ones(len(xy))
        return self.gp_.predict(xy, return_std=True)

    def predict_over(self, accessible, resolution, origin_xy):
        """GP mean and standard deviation as grids, nan outside the accessible cells"""
        rows, cols = np.nonzero(accessible)
        xy = np.stack(cave_geometry.cell_to_world(rows, cols, resolution, origin_xy), axis=1)
        mean = np.full(accessible.shape, np.nan)
        std = np.full(accessible.shape, np.nan)
        if len(xy) > 0:
            mean[rows, cols], std[rows, cols] = self.predict(xy)
        return mean, std

    def candidates(self, grid, resolution, origin_xy, robot_xy):
        """Reachable goal locations in the current map: (N, 2) points and travel distance to each [m]"""
        clearance, confirmed, _, free, _ = cave_geometry.distance_transforms(grid, resolution)
        robot_cell = cave_geometry.world_to_cell(robot_xy[0], robot_xy[1], resolution, origin_xy)
        accessible = cave_geometry.accessible_region(free, clearance, self.robot_radius_, robot_cell)
        if not accessible.any():
            return np.empty((0, 2)), np.empty(0)

        # Travel distance from the accessible cell nearest to the robot
        rows, cols = np.nonzero(accessible)
        nearest = np.argmin((rows - robot_cell[0])**2 + (cols - robot_cell[1])**2)
        distances = travel_distances(accessible, (rows[nearest], cols[nearest]), resolution)

        # Regularly spaced cells with enough clearance to be a goal. Clearance is measured to
        # walls and to unknown space, so a goal is never next to a wall that is not mapped yet
        stride = max(1, int(round(self.candidate_spacing_ / resolution)))
        on_lattice = np.zeros_like(accessible)
        on_lattice[::stride, ::stride] = True
        rows, cols = np.nonzero(accessible & on_lattice & (confirmed >= self.goal_clearance_))
        xy = np.stack(cave_geometry.cell_to_world(rows, cols, resolution, origin_xy), axis=1)
        distances = distances[rows, cols]

        # Not too close to an existing sample or a goal that failed before
        keep = np.isfinite(distances)
        for points, spacing in ((self.sample_xy_, self.min_sample_spacing_),
                                (self.blacklist_xy_, self.blacklist_spacing_)):
            if len(points) > 0 and len(xy) > 0:
                gaps = np.linalg.norm(xy[:, None, :] - np.array(points)[None, :, :], axis=2)
                keep &= gaps.min(axis=1) >= spacing

        return xy[keep], distances[keep]

    def choose_next(self, grid, resolution, origin_xy, robot_xy):
        """Next location to sample as (x, y), or None if there is nowhere left to go"""
        xy, distances = self.candidates(grid, resolution, origin_xy, robot_xy)
        if len(xy) == 0:
            return None

        if self.strategy_ == 'random':
            best = self.rng_.integers(len(xy))
        else:
            # Most uncertain location, discounted by how far away it is
            _, std = self.predict(xy)
            score = (std / max(std.max(), 1e-9)
                     - self.travel_weight_ * distances / max(distances.max(), 1e-9))
            best = np.argmax(score)

        return (float(xy[best, 0]), float(xy[best, 1]))
