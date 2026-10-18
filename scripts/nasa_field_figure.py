"""Figure for the report: how the NASA water map is placed on the cave map (Advanced 6).

Usage: python3 scripts/nasa_field_figure.py OUTPUT.png
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

from offline_cave_map import PACKAGE, load_cave_grid

from cave_explorer.advanced import cave_geometry
from cave_explorer.advanced.measurement_field import (DEFAULT_LAT_RANGE, DEFAULT_LON_RANGE, DEFAULT_X_RANGE,
                                                       DEFAULT_Y_RANGE, MeasurementField)

TABLE = os.path.join(PACKAGE, 'data', 'mars_odyssey_grs', 'h2o_sr_5x5.tab')
table = np.loadtxt(TABLE)
lats, lons = np.unique(table[:, 0]), np.unique(table[:, 1])
world = np.full((len(lats), len(lons)), np.nan)
for lat, lon, value, _, _ in table:
    if value < 9999:
        world[np.searchsorted(lats, lat), np.searchsorted(lons, lon)] = value

low, high = 1.6, 7.4          # whole planet
patch_low, patch_high = 2.2, 5.1   # the patch only, so its variation is easy to see
figure = plt.figure(figsize=(11.5, 10.5))
grid_spec = figure.add_gridspec(2, 2, height_ratios=[0.85, 1.15])
axes = [figure.add_subplot(grid_spec[0, :]), figure.add_subplot(grid_spec[1, 0]), figure.add_subplot(grid_spec[1, 1])]

# 1. The whole NASA map, with the patch that was used
im = axes[0].imshow(world, origin='lower', extent=[0, 360, -90, 90], cmap='viridis', vmin=low, vmax=high, aspect='auto')
axes[0].add_patch(Rectangle((DEFAULT_LON_RANGE[0] - 2.5, DEFAULT_LAT_RANGE[0] - 2.5),
                            DEFAULT_LON_RANGE[1] - DEFAULT_LON_RANGE[0] + 5, DEFAULT_LAT_RANGE[1] - DEFAULT_LAT_RANGE[0] + 5,
                            fill=False, edgecolor='red', linewidth=2.5))
axes[0].plot(239.9, -8.4, marker='*', color='white', markeredgecolor='black', markersize=13)
axes[0].annotate('Arsia Mons', (239.9, -8.4), xytext=(262, -62), color='white', fontsize=9,
                 arrowprops=dict(arrowstyle='-', color='white'))
axes[0].text(180, 75, 'grey = no data near the poles', color='white', ha='center', fontsize=8)
axes[0].set_facecolor('#555555')
axes[0].set_title('1. NASA map of water in Martian soil\n(red box = the patch we used)')
axes[0].set_xlabel('longitude [degrees east]')
axes[0].set_ylabel('latitude [degrees]')
figure.colorbar(im, ax=axes[0], fraction=0.03, pad=0.02, label='water [% of soil weight]')

# 2. The patch: 12 x 12 squares, one number each
patch_lat = [l for l in lats if DEFAULT_LAT_RANGE[0] <= l <= DEFAULT_LAT_RANGE[1]]
patch_lon = [l for l in lons if DEFAULT_LON_RANGE[0] <= l <= DEFAULT_LON_RANGE[1]]
patch = world[np.ix_([np.searchsorted(lats, l) for l in patch_lat], [np.searchsorted(lons, l) for l in patch_lon])]
axes[1].imshow(patch, origin='lower', extent=[patch_lon[0] - 2.5, patch_lon[-1] + 2.5, patch_lat[0] - 2.5, patch_lat[-1] + 2.5],
               cmap='viridis', vmin=patch_low, vmax=patch_high, aspect='equal')
for i, lat in enumerate(patch_lat):
    for j, lon in enumerate(patch_lon):
        axes[1].text(lon, lat, f'{patch[i, j]:.1f}', ha='center', va='center', fontsize=7.5, color='white')
axes[1].set_title(f'2. The patch: {len(patch_lat)} x {len(patch_lon)} squares,\none number per square (about 300 km wide)')
axes[1].set_xlabel('longitude [degrees east]')
axes[1].set_ylabel('latitude [degrees]')

# 3. The same numbers laid over the cave, in metres
grid, origin = load_cave_grid(0.3)
field = MeasurementField(TABLE, seed=0)
clearance, _, _, free, _ = cave_geometry.distance_transforms(grid, 0.3)
reach = cave_geometry.accessible_region(free, clearance, 0.5, cave_geometry.world_to_cell(0.0, 0.0, 0.3, origin))
rows, cols = grid.shape
x = origin[0] + (np.arange(cols) + 0.5) * 0.3
y = origin[1] + (np.arange(rows) + 0.5) * 0.3
xs, ys = np.meshgrid(x, y)
truth = np.where(reach, field.ground_truth(xs, ys), np.nan)
axes[2].set_facecolor('#dddddd')
im3 = axes[2].imshow(truth, origin='lower', extent=[x[0] - 0.15, x[-1] + 0.15, y[0] - 0.15, y[-1] + 0.15],
                     cmap='viridis', vmin=patch_low, vmax=patch_high)
axes[2].contour(xs, ys, grid >= 50, levels=[0.5], colors='k', linewidths=0.6)
axes[2].add_patch(Rectangle((DEFAULT_X_RANGE[0], DEFAULT_Y_RANGE[0]), DEFAULT_X_RANGE[1] - DEFAULT_X_RANGE[0],
                            DEFAULT_Y_RANGE[1] - DEFAULT_Y_RANGE[0], fill=False, edgecolor='red', linewidth=2.5))
axes[2].set_title('3. The patch stretched over the cave map\n(colour only where the rover can drive)')
axes[2].set_xlabel('x [m] in the map')
axes[2].set_ylabel('y [m] in the map')
figure.colorbar(im3, ax=axes[2], fraction=0.05, label='water [% of soil weight], panels 2 and 3')

figure.tight_layout()
figure.savefig(sys.argv[1], dpi=140)
print('saved', sys.argv[1])
