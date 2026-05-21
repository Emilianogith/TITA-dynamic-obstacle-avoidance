#!/usr/bin/env python3
import argparse
import re
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'


def plot_joint_data(path, ylabel, title):
    data = np.loadtxt(path)
    n_steps, n_joints = data.shape
    print(f'  Loaded {n_steps} timesteps, {n_joints} joints from {path.name}')
    plt.figure(figsize=(10, 6))
    for j in range(n_joints):
        plt.plot(np.arange(n_steps), data[:, j], label=f'Joint {j+1}')
    plt.xlabel('Control cycle'); plt.ylabel(ylabel); plt.title(title)
    plt.legend(loc='best'); plt.grid(True); plt.tight_layout()


def parse_joint_velocities(path):
    pattern = re.compile(r'(\S+):\s+pos:\s+[-\d.eE]+\s+vel:\s+([-\d.eE]+)')
    joint_data = {}
    with open(path) as f:
        for line in f:
            if line.startswith('-'):
                continue
            m = pattern.search(line)
            if m:
                joint_data.setdefault(m.group(1), []).append(float(m.group(2)))
    return joint_data


def plot_joint_velocities(joint_data):
    plt.figure(figsize=(12, 7))
    for name, velocities in joint_data.items():
        plt.plot(np.arange(len(velocities)), velocities, label=name)
    plt.xlabel('Control cycle'); plt.ylabel('Velocity [rad/s]')
    plt.title('Joint velocities over time')
    plt.legend(loc='best'); plt.grid(True); plt.tight_layout()


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot joint efforts / velocities')
    parser.add_argument('-eff', action='store_true',
                        help='Plot efforts from joint_eff.txt')
    parser.add_argument('-vel', action='store_true',
                        help='Plot velocities from joint_state_log.txt')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    if args.eff:
        eff_path = log_dir / 'joint_eff.txt'
        if not eff_path.exists():
            print(f'  [skip] not found: {eff_path}')
        else:
            plot_joint_data(eff_path, 'Torque [Nm]', 'Joint torques over time')

    if args.vel:
        js_path = log_dir / 'joint_state_log.txt'
        if not js_path.exists():
            print(f'  [skip] not found: {js_path}')
        else:
            joint_data = parse_joint_velocities(js_path)
            if not joint_data:
                print('  [skip] No joint data found in joint_state_log.txt')
            else:
                for name, vals in joint_data.items():
                    print(f'    {name}: {len(vals)} samples')
                plot_joint_velocities(joint_data)

    plt.show()


if __name__ == '__main__':
    main()