#!/usr/bin/env python3
import argparse
import re
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot joint position / velocity / effort from log')
    parser.add_argument('joint_name', type=str,
                        help='Joint name to plot (e.g. joint_right_leg_4)')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    parser.add_argument('--file', type=str, default=None,
                        help='Override log file path')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    log_file = Path(args.file) if args.file else log_dir / 'joint_state_log.txt'
    if not log_file.exists():
        print(f'  [skip] not found: {log_file}')
        return

    joint_name = args.joint_name
    pattern = re.compile(
        rf'(?P<time>\d+\.\d+)\s+\d+\.\d+\s+{re.escape(joint_name)}:\s+'
        rf'pos:\s+(?P<pos>[-+]?\d*\.\d+)\s+vel:\s+(?P<vel>[-+]?\d*\.\d+)\s+effort:\s+(?P<effort>[-+]?\d*\.\d+)'
    )

    times, positions, velocities, efforts = [], [], [], []
    with open(log_file) as f:
        for line in f:
            m = pattern.search(line)
            if m:
                times.append(float(m.group('time')))
                positions.append(float(m.group('pos')))
                velocities.append(float(m.group('vel')))
                efforts.append(float(m.group('effort')))

    if not times:
        print(f'  [skip] No data found for joint: {joint_name}')
        return

    t0    = times[0]
    times = [t - t0 for t in times]

    mean_pos    = np.mean(positions)
    mean_vel    = np.mean(velocities)
    mean_effort = np.mean(efforts)

    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 6))

    axs[0].plot(times, positions, label='Position')
    axs[0].axhline(mean_pos, color='red', linestyle='--', label=f'avg={mean_pos:.3f}')
    axs[0].set_ylabel('Position [rad]'); axs[0].set_title(f'{joint_name} position')
    axs[0].grid(True); axs[0].legend(fontsize=9)

    axs[1].plot(times, velocities, label='Velocity')
    axs[1].axhline(mean_vel, color='red', linestyle='--', label=f'avg={mean_vel:.3f}')
    axs[1].set_ylabel('Velocity [rad/s]'); axs[1].set_title(f'{joint_name} velocity')
    axs[1].grid(True); axs[1].legend(fontsize=9)

    axs[2].plot(times, efforts, label='Effort')
    axs[2].axhline(mean_effort, color='red', linestyle='--', label=f'avg={mean_effort:.3f}')
    axs[2].set_ylabel('Effort [Nm]'); axs[2].set_xlabel('Time [s]')
    axs[2].set_title(f'{joint_name} effort'); axs[2].grid(True); axs[2].legend(fontsize=9)

    plt.tight_layout()
    plt.show()

    print(f'{joint_name}: pos avg={mean_pos:.6f} rad   vel avg={mean_vel:.6f} rad/s   effort avg={mean_effort:.6f} Nm')


if __name__ == '__main__':
    main()