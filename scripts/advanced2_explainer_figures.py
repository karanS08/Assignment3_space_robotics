"""Step-by-step pictures of the cave geometry analysis (Advanced 2), on the real SLAM map of a Gazebo run.

Usage: python3 scripts/advanced2_explainer_figures.py SLAM_MAP.npz RUN.csv OUTPUT_DIR
Makes: advanced2_steps.png, advanced2_cut_test.png, advanced2_growth.png
"""
import csv
import math
import os
import sys

import cv2
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import numpy as np
from scipy import ndimage

from offline_cave_map import PACKAGE  # noqa: F401  (puts the package on sys.path)

from cave_explorer.advanced import cave_geometry as cg

plt.rcParams.update({'font.size': 11, 'axes.titlesize': 11.5, 'axes.labelsize': 11})
data = np.load(sys.argv[1])
grid, RES, origin = data['map'], float(data['res']), (float(data['ox']), float(data['oy']))
out_dir = sys.argv[3]
H, W = grid.shape
extent = [origin[0], origin[0] + W * RES, origin[1], origin[1] + H * RES]
WALL_COLOUR, UNKNOWN_COLOUR = '#202020', '#9a9a9a'


def show_walls(axis, g=grid, ext=extent):
    layers = np.where(g < 0, 1, np.where(g >= 50, 2, 0))
    axis.imshow(layers, origin='lower', extent=ext, cmap=ListedColormap(['white', UNKNOWN_COLOUR, WALL_COLOUR]), vmin=0, vmax=2)


def labels(axis, title):
    axis.set_title(title, fontsize=10)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')


# ------------------------------------------------------------------ steps
res = cg.analyse(grid, RES, (origin[0], origin[1]), robot_xy=(0.0, 0.0))
clearance, confirmed, nearest_wall, free, occupied = cg.distance_transforms(grid, RES)
figure, axes = plt.subplots(2, 3, figsize=(15, 9.6))
axes = axes.ravel()

show_walls(axes[0])
labels(axes[0], '1. The map the rover has built\n(white = free, grey = not seen yet, black = wall)')

shown = axes[1].imshow(np.where(free, clearance, np.nan), origin='lower', extent=extent, cmap='viridis')
axes[1].contour(*np.meshgrid(origin[0] + (np.arange(W) + .5) * RES, origin[1] + (np.arange(H) + .5) * RES), occupied, levels=[.5], colors='k', linewidths=.5)
figure.colorbar(shown, ax=axes[1], fraction=0.045, label='distance to the nearest wall [m]')
labels(axes[1], '2. For every free spot: how far is the nearest wall?\n(bright = far from walls, dark = close to a wall)')

fit = np.where(free, np.where(clearance >= 0.5, 1, 2), 0)
axes[2].imshow(fit, origin='lower', extent=extent, cmap=ListedColormap(['#bbbbbb', '#8fd18f', '#f0a0a0']), vmin=0, vmax=2)
axes[2].imshow(np.where(res.accessible, 1, np.nan), origin='lower', extent=extent, cmap=ListedColormap(['#2e9e2e']), alpha=.55)
axes[2].plot([], [], 's', color='#2e9e2e', label='rover fits (0.5 m or more from a wall)')
axes[2].plot([], [], 's', color='#f0a0a0', label='too close to a wall')
axes[2].legend(loc='upper right', fontsize=7)
labels(axes[2], '3. Where can the rover actually go?\n(the rover is about 0.5 m wide on each side)')

show_walls(axes[3])
rows, cols = np.nonzero(res.ridge)
x, y = cg.cell_to_world(rows, cols, RES, origin)
axes[3].scatter(x, y, s=2.5, c='gold', edgecolors='none')
labels(axes[3], '4. The centre-line of the cave (gold)\n(the spots that are as far as possible from both walls)')

shown = axes[4].imshow(np.where(res.accessible, confirmed, np.nan), origin='lower', extent=extent, cmap='viridis')
axes[4].add_patch(plt.Circle((res.widest.x, res.widest.y), res.widest.radius, fill=False, color='cyan', linewidth=2.5))
axes[4].plot(res.widest.x, res.widest.y, '+', color='cyan', markersize=12, markeredgewidth=2)
axes[4].annotate(f'widest area:\nradius {res.widest.radius:.1f} m', (res.widest.x, res.widest.y), xytext=(14, 8), textcoords='offset points',
                 color='white', fontsize=9, weight='bold', bbox=dict(boxstyle='round', facecolor='black', alpha=.5))
figure.colorbar(shown, ax=axes[4], fraction=0.045, label='room around this spot [m]')
labels(axes[4], '5. The widest open area\n(the biggest circle that fits without touching a wall)')

show_walls(axes[5])
axes[5].scatter(x, y, s=1.2, c='gold', edgecolors='none')
for i, p in enumerate(res.passages):
    axes[5].plot([p.wall_a[0], p.wall_b[0]], [p.wall_a[1], p.wall_b[1]], color='red', linewidth=3.5 if i == 0 else 2.2)
    axes[5].annotate(f'{p.width:.1f} m', (p.x, p.y), xytext=(5, 5), textcoords='offset points', color='red', fontsize=9, weight='bold')
labels(axes[5], '6. Narrow spots (red lines, with the gap width)\n(thickest line = the narrowest)')
figure.tight_layout()
figure.savefig(os.path.join(out_dir, 'advanced2_steps.png'), dpi=120)
plt.close(figure)

# ------------------------------------------------------------------ cut test, replicated from cave_geometry.find_passages
window = 2 * int(round(1.0 / RES)) + 1
ridge_clear = np.where(res.ridge, clearance, np.inf)
minima = res.ridge & (ridge_clear == ndimage.minimum_filter(ridge_clear, size=window))
mr, mc = np.nonzero(minima)
order = np.argsort(clearance[mr, mc])
half = int(round(5.0 / RES))
min_cells = 3.0 / RES ** 2


def cut_test(row, col):
    wa = (int(nearest_wall[0, row, col]), int(nearest_wall[1, row, col]))
    if wa == (row, col):
        return None
    wb = cg.opposite_wall(occupied, (row, col), wa, int(math.ceil(clearance[row, col] / RES)) + 3)
    if wb is None:
        return None
    r0, r1 = max(row - half, 0), min(row + half + 1, H)
    c0, c1 = max(col - half, 0), min(col + half + 1, W)
    local = res.accessible[r0:r1, c0:c1].astype(np.uint8)
    cv2.line(local, (wa[1] - c0, wa[0] - r0), (col - c0, row - r0), 0, 1)
    cv2.line(local, (col - c0, row - r0), (wb[1] - c0, wb[0] - r0), 0, 1)
    _, lab = cv2.connectedComponents(local, connectivity=4)
    across = np.array([wb[0] - wa[0], wb[1] - wa[1]], float)
    along = np.array([-across[1], across[0]]) / (np.linalg.norm(across) + 1e-9)
    sides = []
    for sign in (1.0, -1.0):
        label = 0
        for step in range(1, 5):
            r, c = int(round(row - r0 + sign * step * along[0])), int(round(col - c0 + sign * step * along[1]))
            if 0 <= r < lab.shape[0] and 0 <= c < lab.shape[1] and lab[r, c] > 0:
                label = lab[r, c]
                break
        sides.append(label)
    box = (r0, r1, c0, c1)
    info = dict(row=row, col=col, wa=wa, wb=wb, lab=lab, sides=sides, box=box, ok=False, areas=(0, 0), widest=(0, 0))
    if 0 in sides or sides[0] == sides[1]:
        info['reason'] = 'the cut does not split the free space in two'
        return info
    counts = np.bincount(lab.ravel())
    info['areas'] = tuple(counts[s] * RES ** 2 for s in sides)
    local_clear = clearance[r0:r1, c0:c1]
    info['widest'] = tuple(float(ndimage.maximum(local_clear, lab, index=s)) for s in sides)
    if min(counts[sides[0]], counts[sides[1]]) < min_cells:
        info['reason'] = 'one side would be tiny (under 3 square metres)'
        return info
    if not any(w >= 1.5 * clearance[row, col] for w in info['widest']):
        info['reason'] = 'neither side opens out to 1.5 times the gap'
        return info
    info['ok'] = True
    info['reason'] = 'a real narrow spot'
    return info


accepted = rejected = None
for i in order:
    row, col = mr[i], mc[i]
    info = cut_test(row, col)
    if info is None:
        continue
    px, py = cg.cell_to_world(row, col, RES, origin)
    if info['ok'] and accepted is None and any(math.hypot(px - p.x, py - p.y) < 1.0 for p in res.passages):
        accepted = info
    if (not info['ok'] and rejected is None and all(math.hypot(px - p.x, py - p.y) > 3.0 for p in res.passages)
            and 5 < px < 50 and 5 < py < 42):
        rejected = info
    if accepted and rejected:
        break

figure, axes = plt.subplots(2, 2, figsize=(11, 10.5))
for r, (info, name) in enumerate(((accepted, 'ACCEPTED'), (rejected, 'REJECTED'))):
    r0, r1, c0, c1 = info['box']
    ext = [origin[0] + c0 * RES, origin[0] + c1 * RES, origin[1] + r0 * RES, origin[1] + r1 * RES]
    show_walls(axes[r, 0], grid[r0:r1, c0:c1], ext)
    wa = cg.cell_to_world(*info['wa'], RES, origin)
    wb = cg.cell_to_world(*info['wb'], RES, origin)
    cx, cy = cg.cell_to_world(info['row'], info['col'], RES, origin)
    gap = math.hypot(wa[0] - wb[0], wa[1] - wb[1])
    axes[r, 0].plot([wa[0], wb[0]], [wa[1], wb[1]], color='red', linewidth=3)
    axes[r, 0].plot([wa[0], wb[0]], [wa[1], wb[1]], 'o', color='red')
    axes[r, 0].plot(cx, cy, '*', color='gold', markeredgecolor='k', markersize=15)
    labels(axes[r, 0], f'{"A" if r == 0 else "B"}1. A spot on the centre-line (star) with its two\nnearest walls; the red line is the cut ({gap:.1f} m)')
    lab = info['lab']
    sides = info['sides']
    colour = np.zeros(lab.shape + (3,))
    colour[:] = 0.95
    palette = {sides[0]: (0.2, 0.45, 0.9), sides[1]: (0.95, 0.6, 0.1)}
    for label, rgb in palette.items():
        if label > 0:
            colour[lab == label] = rgb
    colour[(lab == 0)] = (0.95, 0.95, 0.95)
    colour[occupied[r0:r1, c0:c1]] = (0.15, 0.15, 0.15)
    axes[r, 1].imshow(colour, origin='lower', extent=ext)
    axes[r, 1].plot([wa[0], wb[0]], [wa[1], wb[1]], color='red', linewidth=3)
    if info['ok']:
        text = (f'Two separate areas after the cut:\nblue {info["areas"][0]:.0f} m² (opens to {info["widest"][0]:.1f} m clear)\n'
                f'orange {info["areas"][1]:.0f} m² (opens to {info["widest"][1]:.1f} m clear)')
    else:
        text = 'The cut leaves one connected area:\nnothing is separated'
    verdict = f'{name}: {info["reason"]}'
    axes[r, 1].text(0.02, 0.98 if r else 0.02, text, transform=axes[r, 1].transAxes, fontsize=8.5, va='top' if r else 'bottom',
                    bbox=dict(facecolor='white', alpha=.85))
    labels(axes[r, 1], f'{"A" if r == 0 else "B"}2. Cut the free space along the red line\n{verdict}')
figure.tight_layout()
figure.savefig(os.path.join(out_dir, 'advanced2_cut_test.png'), dpi=120)
plt.close(figure)
print('accepted example at', [round(v, 1) for v in cg.cell_to_world(accepted['row'], accepted['col'], RES, origin)], '| rejected example at',
      [round(v, 1) for v in cg.cell_to_world(rejected['row'], rejected['col'], RES, origin)], '|', rejected['reason'])

# ------------------------------------------------------------------ growth as the rover explores
rows_csv = list(csv.DictReader(open(sys.argv[2])))
visited = np.array([[float(r['x']), float(r['y'])] for r in rows_csv])
cx_all = origin[0] + (np.arange(W) + .5) * RES
cy_all = origin[1] + (np.arange(H) + .5) * RES
gx, gy = np.meshgrid(cx_all, cy_all)
figure, grid_axes = plt.subplots(2, 2, figsize=(12.5, 10.5))
axes = grid_axes.ravel()
print('growth:')
history = []
for count in range(1, len(visited) + 1):
    seen = np.zeros(grid.shape, bool)
    for vx, vy in visited[:count]:
        seen |= np.hypot(gx - vx, gy - vy) <= 12.0
    partial = np.where(seen, grid, -1).astype(np.int8)
    r = cg.analyse(partial, RES, origin, robot_xy=(0.0, 0.0))
    history.append((count, r.widest.radius, r.accessible.sum() * RES * RES))
    if count in (1, 3, 8):
        axis = axes[(1, 3, 8).index(count)]
        show_walls(axis, partial)
        axis.contour(gx, gy, (partial >= 50), levels=[.5], colors='k', linewidths=.5)
        axis.imshow(np.where(r.accessible, 1, np.nan), origin='lower', extent=extent, cmap=ListedColormap(['#2e9e2e']), alpha=.25)
        axis.add_patch(plt.Circle((r.widest.x, r.widest.y), r.widest.radius, fill=False, color='cyan', linewidth=2.5))
        for i, p in enumerate(r.passages):
            axis.plot([p.wall_a[0], p.wall_b[0]], [p.wall_a[1], p.wall_b[1]], color='red', linewidth=3 if i == 0 else 2)
        axis.plot(visited[:count, 0], visited[:count, 1], 'o', color='magenta', markersize=5)
        narrow = f'narrowest gap {r.passages[0].width:.1f} m' if r.passages else 'no narrow spot found yet'
        labels(axis, f'After {"the first" if count == 1 else count} stop{"" if count == 1 else "s"}: widest area radius {r.widest.radius:.1f} m\n'
                     f'free space {r.accessible.sum() * RES * RES:.0f} m\u00b2; {len(r.passages)} narrow spot(s); {narrow}')
        print(f'  after {count} stops: widest {r.widest.radius:.2f} m at [{r.widest.x:.1f}, {r.widest.y:.1f}], {len(r.passages)} passages')
stops = [h[0] for h in history]
axes[3].plot(stops, [h[1] for h in history], color='tab:cyan', linewidth=2.5, label='widest open area: radius [m]')
axes[3].set_xlabel('Number of stops the rover has made')
axes[3].set_ylabel('radius of the widest area [m]', color='tab:cyan')
twin = axes[3].twinx()
twin.plot(stops, [h[2] for h in history], color='tab:green', linewidth=2.5, label='known free space [square metres]')
twin.set_ylabel('free space the rover knows [m\u00b2]', color='tab:green')
for mark in (1, 3, 8):
    axes[3].axvline(mark, color='grey', linestyle=':')
axes[3].set_title('The findings grow as the map grows\n(dotted lines = the three pictures)')
axes[3].grid(alpha=.3)
figure.tight_layout()
figure.savefig(os.path.join(out_dir, 'advanced2_growth.png'), dpi=115)
