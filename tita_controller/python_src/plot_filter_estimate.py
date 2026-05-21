#!/usr/bin/env python3
"""
Plot:
- Base position estimate
- pCL / pCR estimated
- Velocity estimate
- Velocity from differentiated position
- Continuous Roll Pitch Yaw from quaternions
"""
import argparse
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'


def quat_to_rpy(qx, qy, qz, qw):
    roll  = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
    yaw   = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw


def make_quat_continuous(q):
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0:
            q[i] = -q[i]
    return q


def plot_xyz(ax, t, x, y, z, title, ylabel=''):
    ax.plot(t, x, label='x'); ax.plot(t, y, label='y'); ax.plot(t, z, label='z')
    ax.set_title(title)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3); ax.legend(fontsize=9)


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot Kalman filter state estimate')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    parser.add_argument('--file', type=str, default=None,
                        help='Override path to kf_test.csv')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    csv_path = Path(args.file) if args.file else log_dir / 'kf_test.csv'
    if not csv_path.exists():
        print(f'  [skip] not found: {csv_path}')
        return

    df = pd.read_csv(csv_path)
    t  = df['t'].values

    px = df['p_est_x'].values; py = df['p_est_y'].values; pz = df['p_est_z'].values
    vx_diff = np.gradient(px, t); vy_diff = np.gradient(py, t); vz_diff = np.gradient(pz, t)

    fig, axes = plt.subplots(3, 2, figsize=(16, 8), sharex=True)

    plot_xyz(axes[0,0], t, df['p_est_x'], df['p_est_y'], df['p_est_z'],
             'Base Position Estimate', 'Position [m]')
    plot_xyz(axes[1,0], t, df['p_cL_est_x'], df['p_cL_est_y'], df['p_cL_est_z'],
             'pCL Estimate', 'Position [m]')
    plot_xyz(axes[2,0], t, df['p_cR_est_x'], df['p_cR_est_y'], df['p_cR_est_z'],
             'pCR Estimate', 'Position [m]')
    axes[2,0].set_xlabel('Time [s]')

    plot_xyz(axes[0,1], t, df['v_est_x'], df['v_est_y'], df['v_est_z'],
             'Velocity Estimate', 'Velocity [m/s]')

    axes[1,1].plot(t, vx_diff, '--', label='v_diff_x')
    axes[1,1].plot(t, vy_diff, '--', label='v_diff_y')
    axes[1,1].plot(t, vz_diff, '--', label='v_diff_z')
    axes[1,1].set_title('Velocity: Differentiated Position')
    axes[1,1].set_ylabel('Velocity [m/s]')
    axes[1,1].grid(True, alpha=0.3); axes[1,1].legend(fontsize=9)

    q = make_quat_continuous(df[['qx','qy','qz','qw']].values)
    roll, pitch, yaw = quat_to_rpy(q[:,0], q[:,1], q[:,2], q[:,3])
    roll = np.unwrap(roll); pitch = np.unwrap(pitch); yaw = np.unwrap(yaw)

    axes[2,1].plot(t, roll, label='roll')
    axes[2,1].plot(t, pitch, label='pitch')
    axes[2,1].plot(t, yaw, label='yaw')
    axes[2,1].set_title('Orientation (Roll Pitch Yaw)')
    axes[2,1].set_ylabel('Angle [rad]'); axes[2,1].set_xlabel('Time [s]')
    axes[2,1].grid(True, alpha=0.3); axes[2,1].legend(fontsize=9)

    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()