"""Compare recorded simulator runs of the exploration code (Planning 1 to 3).

Reads, for each run folder: timeline.csv (position and mapped area every 5 s, from run_recorder.py), node.log, final_map.npz and
an optional interventions.txt listing any manual help.

Usage: python3 scripts/planning_run_analysis.py REFERENCE_MAP.npz OUTPUT.png NAME=RUN_DIR [NAME=RUN_DIR ...]
"""
import csv
import math
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from offline_cave_map import PACKAGE

reference = np.load(sys.argv[1])
RES = float(reference['res'])
REFERENCE_FREE = float(((reference['map'] >= 0) & (reference['map'] < 50)).sum()) * RES * RES   # free area of a fully explored map


def stop_signs():
    sdf = open(os.path.join(PACKAGE, 'worlds', 'mars_cave.sdf')).read()
    out = []
    for block in re.split(r'(?=<model name=")', sdf):
        m = re.match(r'<model name="([^"]+)">\s*<pose>([^<]+)</pose>', block)
        if m and 'stop' in block.split('</model>')[0].lower():
            p = [float(v) for v in m.group(2).split()]
            out.append((p[0], p[1]))
    return out


SIGNS = stop_signs()


def analyse(folder):
    rows = list(csv.DictReader(open(os.path.join(folder, 'timeline.csv'))))
    t = np.array([float(r['seconds']) for r in rows])
    xy = np.array([[float(r['x']), float(r['y'])] for r in rows])
    free = np.array([float(r['free_area_m2']) for r in rows])
    step = np.hypot(*np.diff(xy, axis=0).T)
    distance = float(np.nansum(step))

    # Stuck: the robot moved less than 0.5 m in 15 s (three 5 s samples)
    moved = np.hypot(*(xy[3:] - xy[:-3]).T)
    # Ignore the idle time at the end, after the code has finished exploring and has nothing left to do
    active = t[3:] <= t[1:][np.hypot(*np.diff(xy, axis=0).T) > 0.3][-1]
    moved = moved[active]
    finished_at = float(t[1:][np.hypot(*np.diff(xy, axis=0).T) > 0.3][-1])
    stuck_seconds = 5.0 * float(np.sum(moved < 0.5))
    longest, run = 0, 0
    for flag in moved < 0.5:
        run = run + 1 if flag else 0
        longest = max(longest, run)

    log = open(os.path.join(folder, 'node.log'), errors='ignore').read()
    inspections = [(float(a), float(b)) for a, b in re.findall(r'Sending inspection goal \(([-0-9.]+), ([-0-9.]+)\)', log)]
    near_sign = sum(1 for g in inspections if min(math.hypot(g[0] - x, g[1] - y) for x, y in SIGNS) < 6.0)
    interventions = []
    path = os.path.join(folder, 'interventions.txt')
    if os.path.exists(path):
        interventions = [l.strip() for l in open(path) if l.strip()]
    return dict(
        t=t, xy=xy, coverage=100.0 * free / REFERENCE_FREE, distance=distance, stuck_seconds=stuck_seconds,
        longest_stuck=5.0 * longest, seconds=float(t[-1]), last_move=finished_at,
        frontier_goals=len(re.findall(r'Frontier goal', log)),
        stuck_events=len(re.findall(r'Stuck at', log)),
        retries=len(re.findall(r'will retry', log)),
        failed=len(re.findall(r'Frontier goal failed|finished with status (?:5|6)', log)),
        artefacts=len(re.findall(r'New artefact', log)),
        inspections=len(re.findall(r'Planning 2 complete', log)),
        inspection_goals=len(inspections), near_sign=near_sign,
        timeouts=len(re.findall(r'Inspection timeout or target lost', log)),
        interventions=interventions)


runs = {}
for spec in sys.argv[3:]:
    name, folder = spec.split('=', 1)
    runs[name] = analyse(os.path.expanduser(folder))

labels = [('Run length [s]', 'seconds', '{:.0f}'), ('Robot stops moving at [s]', 'last_move', '{:.0f}'), ('Distance driven [m]', 'distance', '{:.0f}'),
          ('Cave mapped at the end [%]', None, '{:.0f}'), ('Time not moving before that [s]', 'stuck_seconds', '{:.0f}'),
          ('Longest stop before that [s]', 'longest_stuck', '{:.0f}'), ('Frontier goals chosen', 'frontier_goals', '{}'),
          ('Goals that failed or were cancelled', 'failed', '{}'), ('Goals retried after an instant abort', 'retries', '{}'),
          ('Stuck events handled by the code', 'stuck_events', '{}'), ('Manual rescues needed', None, '{}'),
          ('Artefacts the code reported', 'artefacts', '{}'), ('Inspections completed', 'inspections', '{}'),
          ('Inspection goals within 6 m of a real stop sign', 'near_sign', '{}'), ('Inspections that timed out', 'timeouts', '{}')]
width = max(len(l[0]) for l in labels)
print(' ' * width + ' | ' + ' | '.join(f'{n:>10s}' for n in runs))
for label, key, fmt in labels:
    cells = []
    for r in runs.values():
        if label.startswith('Cave mapped'):
            value = r['coverage'][-1]
        elif label.startswith('Manual rescues'):
            value = len(r['interventions'])
        else:
            value = r[key]
        cells.append(fmt.format(value).rjust(10))
    print(label.ljust(width) + ' | ' + ' | '.join(cells))
print(f'(reference: a fully explored map has {REFERENCE_FREE:.0f} m^2 of free space; the cave has {len(SIGNS)} real stop signs)')

colours = ['tab:blue', 'tab:red', 'tab:green']
figure, axes = plt.subplots(1, 1 + len(runs), figsize=(6 + 5.2 * len(runs), 5.2))
for (name, r), colour in zip(runs.items(), colours):
    axes[0].plot(r['t'] / 60.0, r['coverage'], color=colour, linewidth=2.2, label=name)
axes[0].set(xlabel='Time [minutes]', ylabel='Cave mapped [% of the free space]', title='How fast the cave gets mapped')
axes[0].grid(alpha=.3)
axes[0].legend()
walls = (reference['map'] >= 50)
extent = [float(reference['ox']), float(reference['ox']) + walls.shape[1] * RES, float(reference['oy']), float(reference['oy']) + walls.shape[0] * RES]
for axis, (name, r), colour in zip(axes[1:], runs.items(), colours):
    axis.imshow(np.where(walls, 0.25, 1.0), origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
    axis.plot(r['xy'][:, 0], r['xy'][:, 1], '-', color=colour, linewidth=1.3)
    axis.plot(*r['xy'][0], 'o', color='green', markersize=8)
    for x, y in SIGNS:
        axis.plot(x, y, 's', color='red', markersize=8)
    axis.set_title(f'{name}: path of the robot ({r["distance"]:.0f} m)\n(green = start, red squares = real stop signs)', fontsize=10)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
figure.tight_layout()
figure.savefig(sys.argv[2], dpi=120)
