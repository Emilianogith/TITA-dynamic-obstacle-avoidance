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
    parser = argparse.ArgumentParser(description='Plot IMU angular velocity and acceleration')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    imu_path = log_dir / 'imu_log.txt'
    if not imu_path.exists():
        print(f'  [skip] not found: {imu_path}')
        return

    timestamps = []
    wx, wy, wz = [], [], []
    ax, ay, az = [], [], []

    timestamp_re = re.compile(r'Timestamp:\s+([0-9]+\.[0-9]+)')
    ang_vel_re   = re.compile(r'angular_velocity:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)')
    lin_acc_re   = re.compile(r'linear_acceleration:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)')

    with open(imu_path) as f:
        for line in f:
            ts_match = timestamp_re.search(line)
            if not ts_match:
                continue
            ang_match = ang_vel_re.search(line)
            acc_match = lin_acc_re.search(line)
            if ang_match and acc_match:
                timestamps.append(float(ts_match.group(1)))
                wx.append(float(ang_match.group(1))); wy.append(float(ang_match.group(2))); wz.append(float(ang_match.group(3)))
                ax.append(float(acc_match.group(1))); ay.append(float(acc_match.group(2))); az.append(float(acc_match.group(3)))

    if not timestamps:
        print('  [skip] imu_log.txt has no parseable data')
        return

    t0       = timestamps[0]
    time_sec = [t - t0 for t in timestamps]

    mean_wx = np.mean(wx); mean_wy = np.mean(wy); mean_wz = np.mean(wz)
    mean_ax = np.mean(ax); mean_ay = np.mean(ay); mean_az = np.mean(az)

    fig, axs = plt.subplots(3, 2, sharex=True, figsize=(12, 8))

    for data, mean_val, ylabel, row in zip(
        [wx, wy, wz], [mean_wx, mean_wy, mean_wz],
        ['ωx [rad/s]', 'ωy [rad/s]', 'ωz [rad/s]'], [0, 1, 2]
    ):
        axs[row, 0].plot(time_sec, data, label=ylabel.split(' ')[0])
        axs[row, 0].axhline(mean_val, color='red', linestyle='--', label=f'avg={mean_val:.3f}')
        axs[row, 0].set_ylabel(ylabel)
        axs[row, 0].grid(True)
        axs[row, 0].legend(fontsize=9)
    axs[2, 0].set_xlabel('time [s]')

    for data, mean_val, ylabel, row in zip(
        [ax, ay, az], [mean_ax, mean_ay, mean_az],
        ['ax [m/s²]', 'ay [m/s²]', 'az [m/s²]'], [0, 1, 2]
    ):
        axs[row, 1].plot(time_sec, data, label=ylabel.split(' ')[0])
        axs[row, 1].axhline(mean_val, color='red', linestyle='--', label=f'avg={mean_val:.3f}')
        axs[row, 1].set_ylabel(ylabel)
        axs[row, 1].grid(True)
        axs[row, 1].legend(fontsize=9)
    axs[2, 1].set_xlabel('time [s]')

    fig.suptitle('IMU: Angular Velocity and Linear Acceleration vs Time')
    plt.tight_layout()
    plt.show()

    print(f'Mean ωx: {mean_wx:.6f} rad/s   ωy: {mean_wy:.6f} rad/s   ωz: {mean_wz:.6f} rad/s')
    print(f'Mean ax: {mean_ax:.6f} m/s²    ay: {mean_ay:.6f} m/s²    az: {mean_az:.6f} m/s²')


if __name__ == '__main__':
    main()