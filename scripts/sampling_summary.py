"""Summarise the Gazebo sampling runs (Advanced 6): table for the report and comparison plot.

Reads report/data/gazebo_runs/<strategy>[_twX]_sN.csv, one file per run, written by the sampling node.
Usage: python3 scripts/sampling_summary.py [runs_dir] [latex_table_out] [figure_out]
"""
import csv
import glob
import os
import re
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
runs_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, '..', 'report', 'data', 'gazebo_runs')
table_out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(HERE, '..', 'report', 'latex', 'generated', 'sampling_table.tex')
figure_out = sys.argv[3] if len(sys.argv) > 3 else os.path.join(HERE, '..', 'report', 'figures', 'advanced6_gazebo_multiseed.png')

LABELS = {'adaptive_tw0.3': r'Adaptive, travel weight 0.3', 'adaptive_tw0.1': r'Adaptive, travel weight 0.1', 'random': 'Random baseline'}
COLOURS = {'adaptive_tw0.3': 'tab:red', 'adaptive_tw0.1': 'tab:orange', 'random': 'tab:blue'}

runs = {}
for path in sorted(glob.glob(os.path.join(runs_dir, '*.csv'))):
    m = re.match(r'(?P<config>.+)_s(?P<seed>\d+)\.csv$', os.path.basename(path))
    if not m:
        continue
    rows = list(csv.DictReader(open(path)))
    if len(rows) < 30:
        continue
    runs.setdefault(m['config'], []).append({
        'seed': int(m['seed']),
        'rmse': np.array([float(r['rmse']) for r in rows]),
        'travel': np.array([float(r['travelled_m']) for r in rows]),
        'time': float(rows[-1]['time_s']) - float(rows[0]['time_s'])})

order = [c for c in ('adaptive_tw0.3', 'adaptive_tw0.1', 'random') if c in runs]


def mean_std(values):
    return f'{np.mean(values):.3f} $\\pm$ {np.std(values):.3f}'


lines = [r'\begin{tabular}{@{}lccccccc@{}}', r'\toprule',
         r'\textbf{Strategy} & \textbf{Runs} & \textbf{RMSE @10} & \textbf{RMSE @20} & \textbf{RMSE @30} & '
         r'\textbf{Distance} & \textbf{Time} & \textbf{Dist.\ for RMSE$\le$0.30} \\',
         r'\midrule']
print(f'{"config":16s} runs  RMSE@10        RMSE@20        RMSE@30        distance   time')
for config in order:
    r = runs[config]
    reach = []
    for run in r:
        hit = np.nonzero(run['rmse'] <= 0.30)[0]
        reach.append(run['travel'][hit[0]] if len(hit) else np.nan)
    reached = [x for x in reach if not np.isnan(x)]
    reach_text = (f'{np.mean(reached):.0f} m ({len(reached)}/{len(r)})' if reached else f'never (0/{len(r)})')
    distance = np.mean([x['travel'][-1] for x in r])
    minutes = np.mean([x['time'] for x in r]) / 60.0
    lines.append(f'{LABELS[config]} & {len(r)} & {mean_std([x["rmse"][9] for x in r])} & {mean_std([x["rmse"][19] for x in r])} & '
                 f'{mean_std([x["rmse"][29] for x in r])} & {distance:.0f} m & {minutes:.1f} min & {reach_text} \\\\')
    print(f'{config:16s} {len(r):3d}   ' + '   '.join(f'{np.mean([x["rmse"][i] for x in r]):.3f}+-{np.std([x["rmse"][i] for x in r]):.3f}' for i in (9, 19, 29))
          + f'   {distance:5.0f} m   {minutes:4.1f} min   RMSE<=0.30 after {reach_text}')
lines += [r'\bottomrule', r'\end{tabular}']
os.makedirs(os.path.dirname(table_out), exist_ok=True)
open(table_out, 'w').write('\n'.join(lines) + '\n')

figure, axes = plt.subplots(1, 2, figsize=(12, 4.4))
for config in order:
    r = runs[config]
    n = np.arange(1, 31)
    rmse = np.array([x['rmse'][:30] for x in r])
    axes[0].plot(n, rmse.mean(0), color=COLOURS[config], label=f'{LABELS[config]} ({len(r)} runs)')
    axes[0].fill_between(n, rmse.mean(0) - rmse.std(0), rmse.mean(0) + rmse.std(0), color=COLOURS[config], alpha=0.15)
    common = np.linspace(0, min(x['travel'][-1] for x in r), 100)
    curves = np.array([np.interp(common, x['travel'], x['rmse']) for x in r])
    axes[1].plot(common, curves.mean(0), color=COLOURS[config], label=LABELS[config])
    axes[1].fill_between(common, curves.mean(0) - curves.std(0), curves.mean(0) + curves.std(0), color=COLOURS[config], alpha=0.15)
axes[0].set(xlabel='Number of samples', ylabel='RMSE of reconstructed field [wt % H$_2$O]', title='Error against samples (mean and std over seeds)')
axes[1].set(xlabel='Distance travelled [m]', ylabel='RMSE of reconstructed field [wt % H$_2$O]', title='Error against distance travelled')
for a in axes:
    a.grid(alpha=0.3)
    a.legend(fontsize=8)
figure.tight_layout()
figure.savefig(figure_out, dpi=130)
print('saved', os.path.abspath(table_out), 'and', os.path.abspath(figure_out))
