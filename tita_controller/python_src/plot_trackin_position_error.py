#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot WBC task position errors')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    wbc_path  = log_dir / 'wbc_log.txt'
    plan_file = log_dir / 'plan' / 'x.txt'

    if not wbc_path.exists():
        print(f'  [skip] not found: {wbc_path}')
        return

    log_data = np.loadtxt(wbc_path, delimiter=',', skiprows=1)

    t_ms        = log_data[:, 0]
    com_act     = log_data[:, 1:4]
    com_des     = log_data[:, 4:7]
    wheel_l_act = log_data[:, 7:10]
    wheel_l_des = log_data[:, 10:13]
    wheel_r_act = log_data[:, 13:16]
    wheel_r_des = log_data[:, 16:19]

    fig, axs = plt.subplots(3, 1, figsize=(10, 7))

    for i, label in enumerate(['CoM x error', 'CoM y error', 'CoM z error']):
        axs[0].plot(t_ms, com_des[:, i] - com_act[:, i], label=label, linewidth=2)
    axs[0].set_xlabel('Time [ms]'); axs[0].set_ylabel('Position [m]')
    axs[0].set_title('CoM error'); axs[0].legend(loc='upper right'); axs[0].grid(True)

    for i, label in enumerate(['Left wheel x error', 'Left wheel y error', 'Left wheel z error']):
        axs[1].plot(t_ms, wheel_l_des[:, i] - wheel_l_act[:, i], label=label, linewidth=2)
    axs[1].set_xlabel('Time [ms]'); axs[1].set_ylabel('Position [m]')
    axs[1].set_title('Left wheel error'); axs[1].legend(loc='upper right'); axs[1].grid(True)

    for i, label in enumerate(['Right wheel x error', 'Right wheel y error', 'Right wheel z error']):
        axs[2].plot(t_ms, wheel_r_des[:, i] - wheel_r_act[:, i], label=label, linewidth=2)
    axs[2].set_xlabel('Time [ms]'); axs[2].set_ylabel('Position [m]')
    axs[2].set_title('Right wheel error'); axs[2].legend(loc='upper right'); axs[2].grid(True)

    plt.subplots_adjust(hspace=0.6, top=0.93, bottom=0.08)

    if plan_file.exists():
        plan_data = np.loadtxt(plan_file)
        if plan_data.ndim == 1:
            plan_data = plan_data.reshape(1, -1)

        n_cur = len(com_act)
        if len(plan_data) < n_cur:
            com_plan_ext = np.vstack((plan_data[:, 0:3], np.tile(plan_data[-1, 0:3], (n_cur - len(plan_data), 1))))
        else:
            com_plan_ext = plan_data[:n_cur, 0:3]

        com_err_plan = com_plan_ext - com_act[:n_cur]
        t_common = t_ms[:n_cur]

        fig2, axs2 = plt.subplots(3, 1, figsize=(10, 7))
        for i, title in enumerate(['CoM x error (reference - current)',
                                   'CoM y error (reference - current)',
                                   'CoM z error (reference - current)']):
            axs2[i].plot(t_common, com_err_plan[:, i], linewidth=2)
            axs2[i].set_xlabel('Time [ms]'); axs2[i].set_ylabel('Position [m]')
            axs2[i].set_title(title); axs2[i].grid(True)
        plt.subplots_adjust(hspace=0.6, top=0.93, bottom=0.08)

    plt.show()


if __name__ == '__main__':
    main()