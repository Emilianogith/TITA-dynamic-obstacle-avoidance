#!/usr/bin/env python3
import argparse
import matplotlib.pyplot as plt
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot controller timing log')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    timing_log = log_dir / 'timing_log.txt'
    if not timing_log.exists():
        print(f'  [skip] not found: {timing_log}')
        return

    time_mpc, time_wbc, time_total = [], [], []
    with open(timing_log) as f:
        lines = f.readlines()

    for line in lines[1:]:
        parts = line.strip().split(',')
        if len(parts) < 3:
            continue
        time_mpc.append(float(parts[0]))
        time_wbc.append(float(parts[1]))
        time_total.append(float(parts[2]))

    if not time_mpc:
        print('  [skip] timing_log.txt is empty or has no data rows')
        return

    cycles    = list(range(len(time_mpc)))
    avg_mpc   = sum(time_mpc)   / len(time_mpc)
    avg_wbc   = sum(time_wbc)   / len(time_wbc)
    avg_total = sum(time_total) / len(time_total)

    plt.figure(figsize=(10, 5))
    plt.plot(cycles, time_mpc,   label='MPC (µs)')
    plt.plot(cycles, time_wbc,   label='WBC (µs)')
    plt.plot(cycles, time_total, label='Total (µs)')
    plt.axhline(avg_mpc,   linestyle='--', label=f'MPC avg = {avg_mpc:.1f} µs')
    plt.axhline(avg_wbc,   linestyle='--', label=f'WBC avg = {avg_wbc:.1f} µs')
    plt.axhline(avg_total, linestyle='--', label=f'Total avg = {avg_total:.1f} µs')
    plt.xlabel('Control cycle')
    plt.ylabel('Time (µs)')
    plt.title('Controller Timing')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()