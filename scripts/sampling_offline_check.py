"""Offline comparison of adaptive and random sampling (Advanced 6), no ROS needed.

Simulates the robot sampling the Mars Odyssey water field in the cave. The map
is revealed around each place the robot visits, the robot moves instantly to
each chosen goal, and travel is the shortest path length through free space.
Ground truth is used only to score the reconstruction after each sample.

Usage: python3 scripts/sampling_offline_check.py [runs] [samples] [output_dir]
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from offline_cave_map import PACKAGE, load_cave_grid

from cave_explorer.advanced import cave_geometry
from cave_explorer.advanced.adaptive_sampling import AdaptiveSampler, travel_distances
from cave_explorer.advanced.measurement_field import MeasurementField

RES = 0.3
REVEAL_RADIUS = 12.0   # map known within this distance of every visited location [m]
START = (0.0, 0.0)
TABLE = os.path.join(PACKAGE, 'data', 'mars_odyssey_grs', 'h2o_sr_5x5.tab')

runs = int(sys.argv[1]) if len(sys.argv) > 1 else 10
num_samples = int(sys.argv[2]) if len(sys.argv) > 2 else 40
output_dir = sys.argv[3] if len(sys.argv) > 3 else '.'

full_grid, origin = load_cave_grid(RES)
height, width = full_grid.shape
cell_y, cell_x = np.mgrid[0:height, 0:width]
world_x = origin[0] + (cell_x + 0.5) * RES
world_y = origin[1] + (cell_y + 0.5) * RES

# Evaluation region: everywhere the robot could ever reach
clearance, _, _, free, _ = cave_geometry.distance_transforms(full_grid, RES)
full_accessible = cave_geometry.accessible_region(
    free, clearance, 0.5, cave_geometry.world_to_cell(*START, RES, origin))

accessible_rows, accessible_cols = np.nonzero(full_accessible)


def nearest_accessible(xy):
    row, col = cave_geometry.world_to_cell(xy[0], xy[1], RES, origin)
    nearest = np.argmin((accessible_rows - row)**2 + (accessible_cols - col)**2)
    return (accessible_rows[nearest], accessible_cols[nearest])


def simulate(strategy, seed):
    field = MeasurementField(TABLE, seed=seed)
    sampler = AdaptiveSampler(strategy=strategy, seed=seed)
    truth = field.ground_truth(world_x, world_y)

    robot = START
    known = np.hypot(world_x - robot[0], world_y - robot[1]) <= REVEAL_RADIUS
    travelled = 0.0
    rmse, distance = [], []

    for _ in range(num_samples):
        sampler.add_sample(robot[0], robot[1], field.measure(*robot))

        # Score against ground truth (evaluation only; the sampler never sees this)
        mean, _ = sampler.predict_over(full_accessible, RES, origin)
        rmse.append(np.sqrt(np.nanmean((mean - truth)[full_accessible]**2)))
        distance.append(travelled)

        grid = np.where(known, full_grid, -1)
        goal = sampler.choose_next(grid, RES, origin, robot)
        if goal is None:
            break

        travelled += travel_distances(full_accessible, nearest_accessible(robot), RES)[nearest_accessible(goal)]

        robot = goal
        known |= np.hypot(world_x - robot[0], world_y - robot[1]) <= REVEAL_RADIUS

    return np.array(rmse), np.array(distance), sampler, truth


results = {}
for strategy in ('adaptive', 'random'):
    results[strategy] = [simulate(strategy, seed) for seed in range(runs)]
    print(strategy, 'done')

# Table of RMSE after a given number of samples, and total distance
print(f'\n{runs} runs each, {num_samples} samples, RMSE in weight % H2O')
print('samples   ' + '   '.join(f'{s:>18}' for s in results))
for n in (5, 10, 20, 30, num_samples):
    row = []
    for strategy in results:
        values = [r[0][n - 1] for r in results[strategy] if len(r[0]) >= n]
        row.append(f'{np.mean(values):.3f} +- {np.std(values):.3f}'.rjust(18))
    print(f'{n:>7}   ' + '   '.join(row))
row = [f'{np.mean([r[1][-1] for r in results[s]]):.0f} m'.rjust(18) for s in results]
print('travel    ' + '   '.join(row))

colours = {'adaptive': 'tab:red', 'random': 'tab:blue'}
figure, axes = plt.subplots(1, 2, figsize=(12, 4.5))

# RMSE against number of samples
for strategy, runs_ in results.items():
    length = min(len(r[0]) for r in runs_)
    rmse = np.array([r[0][:length] for r in runs_])
    n = np.arange(1, length + 1)
    axes[0].plot(n, rmse.mean(axis=0), color=colours[strategy], label=strategy)
    axes[0].fill_between(n, rmse.mean(axis=0) - rmse.std(axis=0), rmse.mean(axis=0) + rmse.std(axis=0),
                         color=colours[strategy], alpha=0.2)
axes[0].set_xlabel('Number of samples')
axes[0].set_ylabel('RMSE of reconstructed field [wt % H2O]')
axes[0].set_title(f'Reconstruction error vs samples (mean and std of {runs} runs)')
axes[0].legend()
axes[0].grid(alpha=0.3)

# RMSE against distance travelled
for strategy, runs_ in results.items():
    common = np.linspace(0, min(r[1][-1] for r in runs_), 100)
    rmse = np.array([np.interp(common, r[1], r[0]) for r in runs_])
    axes[1].plot(common, rmse.mean(axis=0), color=colours[strategy], label=strategy)
    axes[1].fill_between(common, rmse.mean(axis=0) - rmse.std(axis=0), rmse.mean(axis=0) + rmse.std(axis=0),
                         color=colours[strategy], alpha=0.2)
axes[1].set_xlabel('Distance travelled [m]')
axes[1].set_ylabel('RMSE of reconstructed field [wt % H2O]')
axes[1].set_title('Reconstruction error vs distance travelled')
axes[1].legend()
axes[1].grid(alpha=0.3)
figure.tight_layout()
figure.savefig(os.path.join(output_dir, 'advanced6_offline_rmse.png'), dpi=130)

# Maps from the first run of each strategy
extent = [origin[0], origin[0] + width * RES, origin[1], origin[1] + height * RES]
truth = np.where(full_accessible, results['adaptive'][0][3], np.nan)
figure, axes = plt.subplots(2, 3, figsize=(15, 8.5))
for row, strategy in enumerate(results):
    sampler = results[strategy][0][2]
    mean, std = sampler.predict_over(full_accessible, RES, origin)
    samples = np.array(sampler.sample_xy_)
    panels = ((truth, 'Ground truth (NASA GRS)', 'viridis', (np.nanmin(truth), np.nanmax(truth))),
              (mean, f'GP mean, {strategy}', 'viridis', (np.nanmin(truth), np.nanmax(truth))),
              (std, f'GP standard deviation, {strategy}', 'magma', (0, None)))
    for axis, (image, title, cmap, limits) in zip(axes[row], panels):
        shown = axis.imshow(image, origin='lower', extent=extent, cmap=cmap, vmin=limits[0], vmax=limits[1])
        axis.contour(world_x, world_y, full_grid, levels=[50], colors='k', linewidths=0.5)
        if 'truth' not in title:
            axis.plot(samples[:, 0], samples[:, 1], '-', color='white', linewidth=0.6, alpha=0.7)
            axis.scatter(samples[:, 0], samples[:, 1], c='white', edgecolors='k', s=22, zorder=3)
        axis.set_title(title)
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
        figure.colorbar(shown, ax=axis, fraction=0.04, label='wt % H2O')
figure.tight_layout()
figure.savefig(os.path.join(output_dir, 'advanced6_offline_maps.png'), dpi=110)
print('\nSaved advanced6_offline_rmse.png and advanced6_offline_maps.png to', os.path.abspath(output_dir))
