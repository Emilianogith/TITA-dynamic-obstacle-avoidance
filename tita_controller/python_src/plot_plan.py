#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'

DT_MS  = 2
NX     = 14
PCOM_X_COL, PCOM_Y_COL, PCOM_Z_COL = 0, 1, 2
C_X_COL,    C_Y_COL,    C_Z_COL    = 6, 7, 8
THETA_COL  = 10
VEC_OFF_Y  = 0.567 / 2


def main() -> None:
    parser = argparse.ArgumentParser(description='Plot MPC plan trajectories')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)

    x_path    = log_dir / 'plan' / 'x.txt'
    jump_path = log_dir / 'jump_traj.txt'

    if not x_path.exists():
        print(f'  [skip] not found: {x_path}')
        return

    x = np.loadtxt(x_path, ndmin=2)
    if x.shape[1] < NX:
        print(f'  [skip] Expected at least {NX} columns, got {x.shape[1]}')
        return

    pcom_x = x[:, PCOM_X_COL]; pcom_y = x[:, PCOM_Y_COL]; pcom_z = x[:, PCOM_Z_COL]
    c_x    = x[:, C_X_COL];    c_y    = x[:, C_Y_COL];    c_z    = x[:, C_Z_COL]
    theta  = x[:, THETA_COL]

    off_x = -np.sin(theta) * VEC_OFF_Y; off_y = np.cos(theta) * VEC_OFF_Y
    cL_x, cL_y = c_x + off_x, c_y + off_y
    cR_x, cR_y = c_x - off_x, c_y - off_y
    t_ms = np.arange(x.shape[0]) * DT_MS

    fig, ax = plt.subplots(1, 2, figsize=(10, 6), layout='constrained')

    ax[0].plot(pcom_x, pcom_y, lw=2, label='CoM')
    ax[0].plot(c_x, c_y, '--', lw=2, label='c')
    ax[0].plot(cL_x, cL_y, '-.', lw=1.5, label='cL')
    ax[0].plot(cR_x, cR_y, ':', lw=1.5, label='cR')
    ax[0].plot(pcom_x[0], pcom_y[0], 'o', ls='', label='start')
    ax[0].plot(pcom_x[-1], pcom_y[-1], 'x', ls='', label='end')
    ax[0].set_xlabel('x [m]'); ax[0].set_ylabel('y [m]')
    ax[0].set_title('Plan x–y reference trajectories')
    ax[0].grid(True); ax[0].legend()

    all_x = np.concatenate([pcom_x, c_x, cL_x, cR_x])
    all_y = np.concatenate([pcom_y, c_y, cL_y, cR_y])
    pad_x = 0.1 * (all_x.max() - all_x.min() + 1e-9)
    pad_y = 0.1 * (all_y.max() - all_y.min() + 1e-9)
    ax[0].set_xlim(all_x.min() - pad_x, all_x.max() + pad_x)
    ax[0].set_ylim(all_y.min() - pad_y, all_y.max() + pad_y)

    ax[1].plot(t_ms, pcom_z, lw=2, label='CoM z')
    ax[1].plot(t_ms, c_z, '--', lw=2, label='c z')
    ax[1].set_xlabel('t [ms]'); ax[1].set_ylabel('z [m]')
    ax[1].set_title('Plan z reference trajectories')
    ax[1].grid(True); ax[1].legend()

    if jump_path.exists():
        lines = jump_path.read_text().strip().splitlines()
        t0_jump = float(lines[0].strip().split()[0])
        x_jump  = np.loadtxt(lines[1:], ndmin=2)
        t_jump  = np.arange(x_jump.shape[0]) * DT_MS + t0_jump

        fig2, ax2 = plt.subplots(figsize=(8, 6), layout='constrained')
        ax2.plot(t_jump, x_jump[:, PCOM_Z_COL], lw=2, label='CoM z')
        ax2.plot(t_jump, x_jump[:, C_Z_COL], '--', lw=2, label='c z')
        ax2.set_xlabel('t [ms]'); ax2.set_ylabel('z [m]')
        ax2.set_title('Plan z reference in jump trajectory')
        ax2.grid(True); ax2.legend()

    plt.show()


if __name__ == '__main__':
    main()