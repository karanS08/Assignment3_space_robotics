"""Draw the cave geometry analysis of a real SLAM map saved from a Gazebo run (Advanced 2 evidence).

Usage: python3 scripts/geometry_slam_figure.py SLAM_MAP.npz OUTPUT.png
The .npz holds the occupancy grid ('map', -1 unknown / 0 free / 100 wall), its 'res' and origin ('ox', 'oy').
"""
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from offline_cave_map import PACKAGE  # noqa: F401  (puts the package on sys.path)

from cave_explorer.advanced import cave_geometry


def main():
    data = np.load(sys.argv[1])
    grid, res, origin = data['map'], float(data['res']), (float(data['ox']), float(data['oy']))
    result = cave_geometry.analyse(grid, res, origin, robot_xy=(0.0, 0.0))
    extent = [origin[0], origin[0] + grid.shape[1] * res, origin[1], origin[1] + grid.shape[0] * res]

    figure, axes = plt.subplots(1, 2, figsize=(13, 5.2))
    shown = np.where(grid < 0, 0.5, np.where(grid >= 50, 1.0, 0.0))
    axes[0].imshow(shown, origin='lower', extent=extent, cmap='gray_r', vmin=0, vmax=1.4)
    axes[0].set_title('SLAM occupancy map from the Gazebo run\n(white free, grey unknown, black wall)')

    clearance = np.where(result.accessible, result.clearance, np.nan)
    image = axes[1].imshow(clearance, origin='lower', extent=extent, cmap='viridis')
    figure.colorbar(image, ax=axes[1], fraction=0.04, label='distance to nearest wall [m]')
    rows, cols = np.nonzero(result.ridge)
    x, y = cave_geometry.cell_to_world(rows, cols, res, origin)
    axes[1].scatter(x, y, s=1.5, c='gold', label='medial axis (ridge)')
    if result.widest is not None:
        axes[1].add_patch(plt.Circle((result.widest.x, result.widest.y), result.widest.radius, fill=False,
                                     color='cyan', linewidth=2.2, label=f'widest area, radius {result.widest.radius:.1f} m'))
    for i, p in enumerate(result.passages):
        axes[1].plot([p.wall_a[0], p.wall_b[0]], [p.wall_a[1], p.wall_b[1]], color='red', linewidth=3 if i == 0 else 2)
        axes[1].annotate(f'{p.width:.1f} m', (p.x, p.y), color='red', fontsize=9, xytext=(4, 4), textcoords='offset points')
    axes[1].plot([], [], color='red', linewidth=3, label='narrow passages (width)')
    axes[1].legend(loc='upper right', fontsize=8)
    axes[1].set_title('Clearance, medial axis, widest area and narrow passages')
    for axis in axes:
        axis.set_xlabel('x [m]')
        axis.set_ylabel('y [m]')
    figure.tight_layout()
    figure.savefig(sys.argv[2], dpi=140)
    if result.widest is not None:
        print(f'widest area: radius {result.widest.radius:.2f} m at [{result.widest.x:.1f}, {result.widest.y:.1f}]')
    for p in result.passages:
        print(f'passage at [{p.x:.1f}, {p.y:.1f}] width {p.width:.2f} m {p.kind} saddle={p.is_saddle}')
    print(f'accessible area {result.accessible.sum() * res * res:.0f} m^2')


if __name__ == '__main__':
    main()
