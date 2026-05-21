#!/usr/bin/env python3
import argparse
import re
import matplotlib.pyplot as plt
from collections import defaultdict
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot velocity control: tau commanded vs measured effort')
    parser.add_argument('joint_name', type=str,
                        help='Joint name to plot (e.g. joint_right_leg_4)')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    tau_path      = log_dir / 'tau_commanded.txt'
    js_path       = log_dir / 'joint_state_log.txt'
    joint_to_plot = args.joint_name

    if not js_path.exists():
        print(f'  [skip] not found: {js_path}')
        return

    t0_state = None
    with open(js_path) as f:
        for line in f:
            m = re.match(r'\d+\.\d+\s+(\d+\.\d+)', line)
            if m:
                t0_state = float(m.group(1))
                break

    if t0_state is None:
        print('  [skip] could not find timestamp in joint_state_log.txt')
        return

    tau_data = defaultdict(lambda: {'t': [], 'vel': [], 'target': [], 'tau_cmd': []})
    if tau_path.exists():
        with open(tau_path) as f:
            current_time = None
            for line in f:
                ts_m = re.match(r'^(\d+\.\d+)', line)
                if ts_m:
                    current_time = float(ts_m.group(1))
                m = re.search(
                    r'(joint_\w+).*effort commanded:\s+([-\d\.eE]+).*vel_target:\s+([-\d\.eE]+)\s+vel_feedback:\s+([-\d\.eE]+)',
                    line
                )
                if m and current_time is not None and m.group(1) == joint_to_plot:
                    tau_data[joint_to_plot]['t'].append(current_time - t0_state)
                    tau_data[joint_to_plot]['tau_cmd'].append(float(m.group(2)))
                    tau_data[joint_to_plot]['vel'].append(float(m.group(4)))
                    tau_data[joint_to_plot]['target'].append(float(m.group(3)))
    else:
        print(f'  [warn] not found: {tau_path}  (tau commanded subplot will be empty)')

    state_data = defaultdict(lambda: {'t': [], 'vel': [], 'tau': []})
    with open(js_path) as f:
        for line in f:
            m = re.search(
                r'\d+\.\d+\s+(\d+\.\d+)\s+(joint_\w+):.*vel:\s+([-\d\.eE]+)\s+effort:\s+([-\d\.eE]+)',
                line
            )
            if m and m.group(2) == joint_to_plot:
                state_data[joint_to_plot]['t'].append(float(m.group(1)) - t0_state)
                state_data[joint_to_plot]['vel'].append(float(m.group(3)))
                state_data[joint_to_plot]['tau'].append(float(m.group(4)))

    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

    if joint_to_plot in state_data:
        axs[0].plot(state_data[joint_to_plot]['t'], state_data[joint_to_plot]['vel'], label='measured vel')
    if joint_to_plot in tau_data:
        axs[0].plot(tau_data[joint_to_plot]['t'], tau_data[joint_to_plot]['vel'], label='vel_feedback')
        axs[0].plot(tau_data[joint_to_plot]['t'], tau_data[joint_to_plot]['target'], linestyle='--', label='vel_target')
    axs[0].set_ylabel('velocity'); axs[0].set_title(joint_to_plot)
    axs[0].legend(); axs[0].grid()

    if joint_to_plot in tau_data:
        axs[1].plot(tau_data[joint_to_plot]['t'], tau_data[joint_to_plot]['tau_cmd'], label='tau commanded')
    if joint_to_plot in state_data:
        axs[1].plot(state_data[joint_to_plot]['t'], state_data[joint_to_plot]['tau'], label='tau measured')
    axs[1].set_xlabel('time [s]'); axs[1].set_ylabel('torque')
    axs[1].legend(); axs[1].grid()

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()