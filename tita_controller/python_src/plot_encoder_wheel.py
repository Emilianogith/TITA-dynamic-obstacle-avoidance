#!/usr/bin/env python3
import argparse
import re
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'

PATTERN = re.compile(
    r'(\d+\.\d+)\s+(joint_\w+):\s+pos:\s+([-\d\.]+):\s+filtered pos:\s+([-\d\.]+)'
    r'\s+vel:\s+([-\d\.]+)\s+vel_diff:\s+([-\d\.]+)\s+filter_pos:\s+([-\d\.]+)\s+filter_vel:\s+([-\d\.]+)'
)


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot wheel encoder / filter data')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    wheel_log = log_dir / 'wheel_log.txt'
    if not wheel_log.exists():
        print(f'  [skip] not found: {wheel_log}')
        return

    rows = []
    with open(wheel_log) as f:
        for line in f:
            m = PATTERN.search(line)
            if m:
                rows.append(m.groups())

    if not rows:
        print('  [skip] wheel_log.txt has no parseable data')
        return

    df = pd.DataFrame(rows, columns=[
        'timestamp', 'joint', 'position', 'filtered_position', 'velocity',
        'vel_diff', 'filter_pos', 'filter_vel'
    ])
    numeric = ['timestamp', 'position', 'filtered_position', 'velocity', 'vel_diff', 'filter_pos', 'filter_vel']
    df[numeric] = df[numeric].astype(float)
    df['timestamp'] -= df['timestamp'].min()

    joints = df['joint'].unique()
    fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex=False, gridspec_kw={'width_ratios': [3, 2]})
    axes_left  = axes[:, 0]
    axes_right = axes[:, 1]

    for joint in joints:
        jdf = df[df['joint'] == joint]
        for col, ax in zip(['position', 'velocity', 'filtered_position', 'vel_diff'], axes_left):
            ax.plot(jdf['timestamp'], jdf[col], label=joint)
            ax.axhline(jdf[col].mean(), linestyle='--', alpha=0.5,
                       label=f'{joint} avg: {jdf[col].mean():.4f}')
        axes_right[0].plot(jdf['timestamp'], jdf['filter_pos'], label=joint)
        axes_right[0].axhline(jdf['filter_pos'].mean(), linestyle='--', alpha=0.5,
                              label=f'{joint} avg: {jdf["filter_pos"].mean():.4f}')
        axes_right[1].plot(jdf['timestamp'], jdf['filter_vel'], label=joint)
        axes_right[1].axhline(jdf['filter_vel'].mean(), linestyle='--', alpha=0.5,
                              label=f'{joint} avg: {jdf["filter_vel"].mean():.4f}')

    axes_left[0].set_ylabel('Position');         axes_left[1].set_ylabel('Velocity')
    axes_left[2].set_ylabel('Filtered Position'); axes_left[3].set_ylabel('Velocity Diff')
    axes_left[3].set_xlabel('Time (s)')
    axes_right[0].set_ylabel('Filter Position'); axes_right[0].set_xlabel('Time (s)')
    axes_right[1].set_ylabel('Filter Velocity'); axes_right[1].set_xlabel('Time (s)')
    axes_right[2].set_visible(False); axes_right[3].set_visible(False)

    for ax in axes_left:       ax.grid(True); ax.legend(fontsize=8)
    for ax in axes_right[:2]:  ax.grid(True); ax.legend(fontsize=8)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()