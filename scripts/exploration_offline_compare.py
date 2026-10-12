"""Compare two ways of choosing exploration goals, without the simulator (Planning 1).

The cave is the map that slam_toolbox built in a Gazebo run. The robot starts at (0, 0), sees by casting rays (10 m range,
stopped by walls), moves to each chosen goal, and looks again. Two goal-choosing rules are compared:
  original  - Christina's rule: the free cell next to unseen space that is nearest in a straight line
  safe      - safe_navigation.choose_frontier: clustered frontiers, goals kept away from walls, chosen by path length
A goal counts as bad if the robot could not get there: it is inside the robot's safety margin of a wall, or unreachable.
A goal counts as tight if it is reachable but less than 1.0 m from a wall (the kind of place where the robot got wedged).

Usage: python3 scripts/exploration_offline_compare.py SLAM_MAP.npz SAFE_NAVIGATION_PACKAGE_DIR OUTPUT.png
"""
import math
import sys
from types import SimpleNamespace

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

SAFE_PACKAGE = sys.argv[2]
sys.path.insert(0, SAFE_PACKAGE)
from cave_explorer.advanced import cave_geometry as cg
from cave_explorer.planning import safe_navigation as sn

data = np.load(sys.argv[1])
truth = data['map'].astype(np.int8)
truth[truth < 0] = 100                     # the inside of wall islands was never seen; treat it as wall
RES, ORIGIN = float(data['res']), (float(data['ox']), float(data['oy']))
H, W = truth.shape
RANGE = 10.0
START = (0.0, 0.0)

clearance_t, _, _, free_t, _ = cg.distance_transforms(truth, RES)
reach_t = cg.accessible_region(free_t, clearance_t, 0.5, cg.world_to_cell(*START, RES, ORIGIN))
total_free = float(free_t.sum())


def look(known, xy):
    """Reveal what a 10 m laser sees from xy: along each ray until a wall"""
    for angle in np.radians(np.arange(0, 360, 1.0)):
        for step in np.arange(0.0, RANGE, RES * 0.7):
            x, y = xy[0] + step * math.cos(angle), xy[1] + step * math.sin(angle)
            row, col = cg.world_to_cell(x, y, RES, ORIGIN)
            if not (0 <= row < H and 0 <= col < W):
                break
            known[row, col] = True
            if truth[row, col] >= 50:
                break


def as_message(known):
    grid = np.where(known, truth, -1).astype(np.int8)
    return SimpleNamespace(data=grid.ravel().tolist(),
                           info=SimpleNamespace(resolution=RES, width=W, height=H,
                                                origin=SimpleNamespace(position=SimpleNamespace(x=ORIGIN[0], y=ORIGIN[1]))))


def original_choice(known, xy, visited):
    """Christina's rule: nearest free cell (straight line) that has an unseen 4-neighbour, 1.5 m from earlier goals"""
    grid = np.where(known, truth, -1)
    best, best_distance = None, 1e9
    rows, cols = np.nonzero(grid == 0)
    for row, col in zip(rows, cols):
        if row < 1 or col < 1 or row >= H - 1 or col >= W - 1:
            continue
        if -1 not in (grid[row, col - 1], grid[row, col + 1], grid[row - 1, col], grid[row + 1, col]):
            continue
        x, y = cg.cell_to_world(row, col, RES, ORIGIN)
        if any(math.hypot(x - a, y - b) < 1.5 for a, b in visited):
            continue
        distance = math.hypot(x - xy[0], y - xy[1])
        if distance < best_distance:
            best, best_distance = (x, y), distance
    return best


def run(strategy, max_steps=400):
    known = np.zeros(truth.shape, bool)
    xy = START
    look(known, xy)
    visited, blacklist = [], []
    travelled, curve, goals = 0.0, [(0.0, float((known & free_t).sum()) / total_free)], []
    bad = tight = 0
    for _ in range(max_steps):
        if strategy == 'original':
            goal = original_choice(known, xy, visited)
        else:
            choice = sn.choose_frontier(as_message(known), xy, blacklist, visited[-8:])
            goal = choice['goal'] if choice else None
        if goal is None:
            break
        visited.append(goal)
        row, col = cg.world_to_cell(goal[0], goal[1], RES, ORIGIN)
        in_bounds = 0 <= row < H and 0 <= col < W
        feasible = in_bounds and reach_t[row, col]
        if not feasible:
            bad += 1
            blacklist.append(goal)                 # a real robot would waste time here, then give up on the goal
            goals.append((goal, 'bad'))
            continue
        if clearance_t[row, col] < 1.0:
            tight += 1
            goals.append((goal, 'tight'))
        else:
            goals.append((goal, 'ok'))
        start_cell = cg.world_to_cell(xy[0], xy[1], RES, ORIGIN)
        rr, cc = np.nonzero(reach_t)
        near = np.argmin((rr - start_cell[0]) ** 2 + (cc - start_cell[1]) ** 2)
        travelled += float(sn.travel_distances(reach_t, (rr[near], cc[near]), RES)[row, col])
        xy = goal
        look(known, xy)
        curve.append((travelled, float((known & free_t).sum()) / total_free))
    return dict(curve=np.array(curve), goals=goals, bad=bad, tight=tight, steps=len(goals), distance=travelled)


results = {name: run(name) for name in ('original', 'safe')}
for name, r in results.items():
    final = r['curve'][-1, 1]
    reach95 = np.nonzero(r['curve'][:, 1] >= 0.95)[0]
    to95 = f'{r["curve"][reach95[0], 0]:.0f} m' if len(reach95) else 'never'
    print(f'{name:9s} goals {r["steps"]:3d} | bad goals {r["bad"]:2d} | tight goals (<1 m from a wall) {r["tight"]:2d} | '
          f'distance {r["distance"]:5.0f} m | final coverage {100 * final:4.1f}% | distance to 95%: {to95}')

figure, axes = plt.subplots(1, 3, figsize=(17, 5.4))
colours = {'original': 'tab:blue', 'safe': 'tab:red'}
for name, r in results.items():
    axes[0].plot(r['curve'][:, 0], 100 * r['curve'][:, 1], color=colours[name], linewidth=2.2,
                 label={'original': 'original rule (nearest straight-line frontier)', 'safe': 'safe rule (clusters, clearance, path length)'}[name])
axes[0].set(xlabel='Distance driven [m]', ylabel='Cave seen [% of free area]', title='How fast the cave gets mapped')
axes[0].legend(fontsize=8, loc='lower right')
axes[0].grid(alpha=.3)
extent = [ORIGIN[0], ORIGIN[0] + W * RES, ORIGIN[1], ORIGIN[1] + H * RES]
for axis, name in zip(axes[1:], ('original', 'safe')):
    axis.imshow(np.where(truth >= 50, 0.2, 1.0), origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
    r = results[name]
    for (x, y), kind in r['goals']:
        axis.plot(x, y, 'o', markersize=7, color={'ok': 'tab:green', 'tight': 'orange', 'bad': 'red'}[kind], markeredgecolor='k')
    axis.set_title(f'{name}: {r["steps"]} goals ({r["tight"]} tight, {r["bad"]} bad)', fontsize=10)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
axes[1].plot([], [], 'o', color='tab:green', markeredgecolor='k', label='fine')
axes[1].plot([], [], 'o', color='orange', markeredgecolor='k', label='tight: under 1 m from a wall')
axes[1].plot([], [], 'o', color='red', markeredgecolor='k', label='bad: not reachable')
axes[1].legend(fontsize=8, loc='upper right')
figure.tight_layout()
figure.savefig(sys.argv[3], dpi=120)
