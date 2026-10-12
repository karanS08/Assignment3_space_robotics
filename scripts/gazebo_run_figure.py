"""Figure for the report: what the rover measured in one Gazebo run and what it learned from it (Advanced 6).

The Gaussian Process is re-fitted here from the samples written to the run's CSV file, with the same
sampler code the rover used, then compared with the true NASA field.

Usage: python3 scripts/gazebo_run_figure.py RUN.csv OUTPUT.png
"""
import csv
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from offline_cave_map import PACKAGE, load_cave_grid

from cave_explorer.advanced import cave_geometry
from cave_explorer.advanced.adaptive_sampling import AdaptiveSampler
from cave_explorer.advanced.measurement_field import MeasurementField

TABLE = os.path.join(PACKAGE, 'data', 'mars_odyssey_grs', 'h2o_sr_5x5.tab')
rows = list(csv.DictReader(open(sys.argv[1])))
x = np.array([float(r['x']) for r in rows])
y = np.array([float(r['y']) for r in rows])
measured = np.array([float(r['h2o_wt_percent']) for r in rows])

field = MeasurementField(TABLE)
true_at_samples = np.array([float(field.ground_truth(a, b)) for a, b in zip(x, y)])

grid, origin = load_cave_grid(0.3)
clearance, _, _, free, _ = cave_geometry.distance_transforms(grid, 0.3)
reach = cave_geometry.accessible_region(free, clearance, 0.5, cave_geometry.world_to_cell(0.0, 0.0, 0.3, origin))
sampler = AdaptiveSampler(strategy='adaptive', seed=0)
for a, b, v in zip(x, y, measured):
    sampler.add_sample(a, b, v)
mean, std = sampler.predict_over(reach, 0.3, origin)

height, width = grid.shape
cx = origin[0] + (np.arange(width) + 0.5) * 0.3
cy = origin[1] + (np.arange(height) + 0.5) * 0.3
xs, ys = np.meshgrid(cx, cy)
truth = np.where(reach, field.ground_truth(xs, ys), np.nan)
extent = [cx[0] - 0.15, cx[-1] + 0.15, cy[0] - 0.15, cy[-1] + 0.15]
low, high = 2.2, 5.1

figure, grid_axes = plt.subplots(2, 2, figsize=(12, 10))
axes = grid_axes.ravel()


def draw_map(axis, image, title, cmap, vmin, vmax, label):
    shown = axis.imshow(image, origin='lower', extent=extent, cmap=cmap, vmin=vmin, vmax=vmax)
    axis.contour(xs, ys, grid >= 50, levels=[0.5], colors='k', linewidths=0.5)
    axis.set_facecolor('#dddddd')
    axis.set_title(title, fontsize=10)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    figure.colorbar(shown, ax=axis, fraction=0.045, label=label)


draw_map(axes[0], truth, '1. True NASA water field\nwith the rover\'s 30 sample spots, in order', 'viridis', low, high, 'water [% of soil weight]')
axes[0].plot(x, y, '-', color='white', linewidth=0.8, alpha=0.8)
axes[0].scatter(x, y, c=measured, cmap='viridis', vmin=low, vmax=high, edgecolors='k', s=45, zorder=3)
for i in (0, 9, 19, 29):
    axes[0].annotate(str(i + 1), (x[i], y[i]), xytext=(4, 4), textcoords='offset points', fontsize=9, color='red', weight='bold')
draw_map(axes[1], mean, '2. What the rover worked out\n(estimate from its 30 measurements)', 'viridis', low, high, 'water [% of soil weight]')
draw_map(axes[2], std, '3. How unsure it still is\n(dark = sure, bright = unsure)', 'magma', 0, None, 'uncertainty [% of soil weight]')

lim = [2.0, 5.3]
axes[3].plot(lim, lim, color='grey', linestyle='--', label='perfect reading')
axes[3].scatter(true_at_samples, measured, c='tab:red', edgecolors='k')
axes[3].set_xlim(lim)
axes[3].set_ylim(lim)
axes[3].set_aspect('equal')
axes[3].set_xlabel('true value at that spot [%]')
axes[3].set_ylabel('value the rover measured [%]')
axes[3].set_title('4. Each reading is the true value\nplus instrument noise', fontsize=10)
axes[3].legend(fontsize=8)
axes[3].grid(alpha=0.3)
figure.tight_layout()
figure.savefig(sys.argv[2], dpi=135)

error = measured - true_at_samples
print(f'samples {len(x)}; reading minus truth: mean {error.mean():+.2f}, std {error.std():.2f}, largest {np.abs(error).max():.2f}')
print('estimate RMSE over cave: %.3f' % np.sqrt(np.nanmean((mean - truth)[reach] ** 2)))
print('first eight samples (x, y, lat, lon, truth, measured):')
for i in range(8):
    lat, lon = field.to_lat_lon(x[i], y[i])
    print(f'  {i + 1} & {x[i]:.1f} & {y[i]:.1f} & {float(lat):.1f} & {float(lon):.1f} & {true_at_samples[i]:.2f} & {measured[i]:.2f}')
