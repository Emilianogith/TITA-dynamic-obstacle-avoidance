#!/usr/bin/env python3
import argparse
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'

JOINT_ORDER = [
    'joint_left_leg_1', 'joint_left_leg_2', 'joint_left_leg_3', 'joint_left_leg_4',
    'joint_right_leg_1', 'joint_right_leg_2', 'joint_right_leg_3', 'joint_right_leg_4',
]


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot tau sent vs recorded for a joint')
    parser.add_argument('joint_name', type=str,
                        help='Joint name (e.g. joint_right_leg_4)')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    if args.joint_name not in JOINT_ORDER:
        raise ValueError(f'Joint {args.joint_name} not found in known joint list')

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    tau_path = log_dir / 'tau_commanded_log.txt'
    js_path  = log_dir / 'joint_state_log.txt'
    joint    = args.joint_name

    if not js_path.exists():
        print(f'  [skip] not found: {js_path}')
        return

    js_df = pd.read_csv(js_path)
    t_rec  = js_df['time_now_s'].values
    effort = js_df[f'{joint}_effort'].values

    t_cmd, tau_cmd = None, None
    if tau_path.exists():
        tau_df = pd.read_csv(tau_path)
        t_cmd   = tau_df['time_s'].values
        tau_cmd = tau_df[f'{joint}_effort'].values
    else:
        print(f'  [warn] not found: {tau_path}')

    t0    = min(t_cmd[0] if t_cmd is not None else t_rec[0], t_rec[0])
    t_rec = t_rec - t0
    if t_cmd is not None:
        t_cmd = t_cmd - t0

    plt.figure(figsize=(8, 4))
    if t_cmd is not None:
        plt.plot(t_cmd, tau_cmd, label='tau commanded', linestyle='--')
    plt.plot(t_rec, effort, label=f'effort measured ({joint})')
    plt.xlabel('Time [s]')
    plt.ylabel('Torque [Nm]')
    plt.title(f'Tau Commanded vs Effort Measured — {joint}')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()