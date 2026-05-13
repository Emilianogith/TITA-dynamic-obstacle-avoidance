#!/usr/bin/env python3
"""
Generalized plotting script for TITA controller logs.

By default every group is saved to images/ (organised in subdirectories).
Pass --show-<group> to open that group in interactive windows instead of
saving — you can zoom, pan, and inspect values.  Multiple flags are allowed.

Plots produced (each skipped gracefully if the source file is missing):
  encoder_wheels/    wheel_log.txt
  estimation/        kf_test.csv (odom vs est, filter analysis, contact points, XY trajectory)
  imu/               imu_log.txt
  odometry/          odom.txt, robot_odom.csv
  plan/              robot_logs/plan/x.txt  [+ jump_traj.txt]
  timings/           robot_logs/timing_log.txt
  wbc/               robot_logs/wbc_log.txt  [+ robot_logs/plan/x.txt]
  mpc/               robot_logs/mpc_data/{t}/x.txt + u.txt  (requires --mpc-t)
  joints/state/      joint_state_log.txt  (per joint)
  joints/torques/    /tmp/joint_eff.txt
  joints/velocities/ /tmp/joint_vel.txt
  joints/numeric_derivative/  numeric_derivative.txt  (per joint)
  joints/tau_velocity/        tau_commanded.txt + joint_state_log.txt  (per joint)
"""

import argparse
import re
import sys
import numpy as np
import pandas as pd
import matplotlib

# ---------------------------------------------------------------------------
# Determine which groups to show interactively — must happen before pyplot
# import so that matplotlib.use('Agg') is called at the right time.
# ---------------------------------------------------------------------------
_ALL_GROUPS = {'encoder', 'estimation', 'imu', 'odom', 'plan', 'timings', 'wbc', 'mpc', 'joints'}
_SHOW = {g for g in _ALL_GROUPS if f'--show-{g}' in sys.argv}
if not _SHOW:
    matplotlib.use('Agg')

import matplotlib.pyplot as plt
from pathlib import Path
from collections import defaultdict

# ---------------------------------------------------------------------------
# Wong (2011) 8-colour colorblind-safe palette
# ---------------------------------------------------------------------------
CB = [
    '#000000',  # 0 black
    '#E69F00',  # 1 orange
    '#56B4E9',  # 2 sky blue
    '#009E73',  # 3 bluish green
    '#F0E442',  # 4 yellow
    '#0072B2',  # 5 blue
    '#D55E00',  # 6 vermilion
    '#CC79A7',  # 7 reddish pink
]

plt.rcParams.update({
    'axes.prop_cycle': plt.cycler(color=CB),
    'axes.grid': True,
    'grid.alpha': 0.3,
    'grid.linestyle': '--',
    'font.size': 10,
    'axes.titlesize': 11,
    'axes.labelsize': 10,
    'legend.fontsize': 9,
    'figure.dpi': 150,
})

SCRIPT_DIR = Path(__file__).resolve().parent
print(f'Output directory: {SCRIPT_DIR / "images"}')
# Default log dir is robot_logs/ under the working directory (e.g. ros2_ws/).
# Override with --log-dir if the script is run from a different location.
# use path to find robot logs relative to mine even if the script is run from a different directory
ROBOT_LOGS = Path('robot_logs').resolve()
print(f'Robot logs directory: {ROBOT_LOGS}')

# Output directory: images/ relative to this script
OUT_DIR = SCRIPT_DIR / 'images'
print(f'  ({OUT_DIR.relative_to(SCRIPT_DIR)})')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def save(fig, group: str, *parts: str) -> None:
    """Save fig to images/<parts> or, if group is in _SHOW, keep it open."""
    if group in _SHOW:
        print(f'  queued {"/".join(parts)}')
        return  # leave fig open; plt.show() at the end renders all
    path = OUT_DIR.joinpath(*parts)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, bbox_inches='tight')
    plt.close(fig)
    print(f'  saved  {path.relative_to(OUT_DIR.parent)}')


def warn_missing(path: Path) -> None:
    print(f'  [skip] not found: {path}')


def _xyz_lines(ax, t, x, y, z, labels=('x', 'y', 'z'), linestyles=('-', '-', '-')):
    for d, lbl, ls, c in zip([x, y, z], labels, linestyles, [CB[5], CB[3], CB[6]]):
        ax.plot(t, d, ls, color=c, label=lbl)
    ax.legend()


def _quat_to_rpy(qx, qy, qz, qw):
    roll  = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
    yaw   = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw


def _fix_quat_continuity(q: np.ndarray) -> np.ndarray:
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i - 1], q[i]) < 0:
            q[i] = -q[i]
    return q


# ---------------------------------------------------------------------------
# Encoder wheel log  (wheel_log.txt)
# ---------------------------------------------------------------------------

def plot_encoder_wheels(log_dir: Path) -> None:
    log_file = log_dir / 'wheel_log.txt'
    if not log_file.exists():
        warn_missing(log_file)
        return

    pattern = (
        r'(\d+\.\d+)\s+(joint_\w+):\s+pos:\s+([-\d\.]+):\s+filtered pos:\s+([-\d\.]+)'
        r'\s+vel:\s+([-\d\.]+)\s+vel_diff:\s+([-\d\.]+)\s+filter_pos:\s+([-\d\.]+)\s+filter_vel:\s+([-\d\.]+)'
    )
    rows = [m.groups() for line in log_file.read_text().splitlines()
            if (m := re.search(pattern, line))]
    if not rows:
        print('  [skip] no data in wheel_log.txt')
        return

    cols = ['timestamp', 'joint', 'position', 'filtered_position',
            'velocity', 'vel_diff', 'filter_pos', 'filter_vel']
    df = pd.DataFrame(rows, columns=cols)
    for c in cols[2:]:
        df[c] = df[c].astype(float)
    df['timestamp'] = df['timestamp'].astype(float) - df['timestamp'].astype(float).min()

    joints = df['joint'].unique()
    fig, axes = plt.subplots(3, 2, figsize=(16, 11))
    fig.suptitle('Wheel Encoder Data', fontsize=13)

    left_conf  = [('position',          'Position [rad]'),
                  ('velocity',          'Velocity [rad/s]'),
                  ('filtered_position', 'Filtered Position [rad]')]
    right_conf = [('filter_pos', 'Filter Position [rad]'),
                  ('filter_vel', 'Filter Velocity [rad/s]'),
                  ('vel_diff',   'Velocity Diff [rad/s]')]

    for color_idx, joint in enumerate(joints):
        jdf = df[df['joint'] == joint]
        t = jdf['timestamp']
        c = CB[color_idx % len(CB)]
        for row, (metric, ylabel) in enumerate(left_conf):
            ax = axes[row, 0]
            ax.plot(t, jdf[metric], color=c, label=joint)
            avg = jdf[metric].mean()
            ax.axhline(avg, linestyle='--', color=c, alpha=0.55, label=f'avg={avg:.3f}')
            ax.set_ylabel(ylabel)
        for row, (metric, ylabel) in enumerate(right_conf):
            ax = axes[row, 1]
            ax.plot(t, jdf[metric], color=c, label=joint)
            avg = jdf[metric].mean()
            ax.axhline(avg, linestyle='--', color=c, alpha=0.55, label=f'avg={avg:.3f}')
            ax.set_ylabel(ylabel)

    for col in range(2):
        axes[-1, col].set_xlabel('Time [s]')
    for row_axes in axes:
        for ax in row_axes:
            ax.legend(fontsize=7)

    fig.tight_layout()
    save(fig, 'encoder', 'encoder_wheels', 'wheel_data.png')


# ---------------------------------------------------------------------------
# State estimation  (kf_test.csv)
# ---------------------------------------------------------------------------

def plot_estimation(log_dir: Path) -> None:
    csv = log_dir / 'kf_test.csv'
    if not csv.exists():
        warn_missing(csv)
        return

    df = pd.read_csv(csv)
    t = df['t'].values - df['t'].iloc[0]

    # --- Odometry vs Estimated Position ---
    fig, ax = plt.subplots(figsize=(13, 5))
    for axis, c_odom, c_est, ls_odom in zip(
        ['x', 'y', 'z'],
        [CB[5], CB[3], CB[6]],
        [CB[2], CB[1], CB[7]],
        ['--', '-.', ':'],
    ):
        ax.plot(t, df[f'p_odom_{axis}'], ls_odom, color=c_odom, alpha=0.8, label=f'odom {axis}')
        ax.plot(t, df[f'p_est_{axis}'],  '-',      color=c_est,             label=f'est  {axis}')
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Position [m]')
    ax.set_title('Odometry vs Estimated Base Position'); ax.legend(ncol=2)
    fig.tight_layout()
    save(fig, 'none', 'estimation', 'odom_vs_est_position.png')

    # --- XY Position Error (norm) ---
    pos_err = np.sqrt((df['p_est_x'] - df['p_odom_x'])**2 +
                      (df['p_est_y'] - df['p_odom_y'])**2)
    fig, ax = plt.subplots(figsize=(13, 4))
    ax.plot(t, pos_err, color=CB[6])
    ax.set_xlabel('Time [s]'); ax.set_ylabel('‖error‖ [m]')
    ax.set_title('Base XY Position: ‖Estimate − Odometry‖')
    fig.tight_layout()
    save(fig, 'none', 'estimation', 'position_error.png')

    # --- Velocity estimate ---
    fig, ax = plt.subplots(figsize=(13, 5))
    _xyz_lines(ax, t,
               df['v_est_x'].values, df['v_est_y'].values, df['v_est_z'].values)
    ax.set_xlabel('Time [s]'); ax.set_ylabel('Velocity [m/s]')
    ax.set_title('Estimated Base Velocity')
    fig.tight_layout()
    save(fig, 'none', 'estimation', 'velocity.png')

    # --- Contact points ---
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    fig.suptitle('Estimated Contact Points')
    for ax, side in zip(axes, ['cL', 'cR']):
        _xyz_lines(ax, t,
                   df[f'p_{side}_est_x'].values,
                   df[f'p_{side}_est_y'].values,
                   df[f'p_{side}_est_z'].values)
        ax.set_ylabel('Position [m]'); ax.set_title(f'{side} Contact Point')
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'none', 'estimation', 'contact_points.png')

    # --- XY Trajectory (top view) ---
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(df['p_odom_x'], df['p_odom_y'], '--', color=CB[5], label='Base odom')
    ax.plot(df['p_est_x'],  df['p_est_y'],        color=CB[2], label='Base est')
    ax.plot(df['p_cL_est_x'], df['p_cL_est_y'],   color=CB[6], label='cL')
    ax.plot(df['p_cR_est_x'], df['p_cR_est_y'],   color=CB[7], label='cR')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')
    ax.set_title('XY Trajectories (Top View)')
    ax.set_aspect('equal'); ax.legend()
    fig.tight_layout()
    save(fig, 'none', 'estimation', 'xy_trajectory.png')

    # --- Filter analysis (6-panel) ---
    px = df['p_est_x'].values; py = df['p_est_y'].values; pz = df['p_est_z'].values
    vx_d = np.gradient(px, t); vy_d = np.gradient(py, t); vz_d = np.gradient(pz, t)

    has_quat = all(c in df.columns for c in ['qx', 'qy', 'qz', 'qw'])
    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    fig.suptitle('State Estimation — Filter Analysis', fontsize=13)

    def _panel(ax, x, y, z, title, ylabel):
        _xyz_lines(ax, t, x, y, z)
        ax.set_title(title); ax.set_ylabel(ylabel)

    _panel(axes[0, 0], px, py, pz,                           'Base Position Estimate',  'Position [m]')
    _panel(axes[1, 0], df['p_cL_est_x'].values,
                       df['p_cL_est_y'].values,
                       df['p_cL_est_z'].values,              'cL Position Estimate',    'Position [m]')
    _panel(axes[2, 0], df['p_cR_est_x'].values,
                       df['p_cR_est_y'].values,
                       df['p_cR_est_z'].values,              'cR Position Estimate',    'Position [m]')
    _panel(axes[0, 1], df['v_est_x'].values,
                       df['v_est_y'].values,
                       df['v_est_z'].values,                 'Velocity Estimate',       'Velocity [m/s]')

    for c, d, lbl in zip([CB[5], CB[3], CB[6]], [vx_d, vy_d, vz_d], ['vx_diff', 'vy_diff', 'vz_diff']):
        axes[1, 1].plot(t, d, '--', color=c, label=lbl)
    axes[1, 1].set_title('Velocity from Differentiated Position')
    axes[1, 1].set_ylabel('Velocity [m/s]'); axes[1, 1].legend()

    if has_quat:
        q = _fix_quat_continuity(df[['qx', 'qy', 'qz', 'qw']].values)
        roll, pitch, yaw = _quat_to_rpy(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
        for c, d, lbl in zip([CB[1], CB[2], CB[5]],
                              [np.unwrap(roll), np.unwrap(pitch), np.unwrap(yaw)],
                              ['roll', 'pitch', 'yaw']):
            axes[2, 1].plot(t, d, color=c, label=lbl)
        axes[2, 1].set_title('Orientation (Roll Pitch Yaw)')
        axes[2, 1].set_ylabel('Angle [rad]'); axes[2, 1].legend()
    else:
        axes[2, 1].set_visible(False)

    axes[2, 0].set_xlabel('Time [s]'); axes[2, 1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'estimation', 'estimation', 'filter_analysis.png')


# ---------------------------------------------------------------------------
# IMU  (imu_log.txt)
# ---------------------------------------------------------------------------

def plot_imu(log_dir: Path) -> None:
    log_path = log_dir / 'imu_log.txt'
    if not log_path.exists():
        warn_missing(log_path)
        return

    ts_re  = re.compile(r'Timestamp:\s+([0-9]+\.[0-9]+)')
    ang_re = re.compile(r'angular_velocity:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)')
    acc_re = re.compile(r'linear_acceleration:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)')

    timestamps, wx, wy, wz, aax, aay, aaz = [], [], [], [], [], [], []
    cur_ts = None
    for line in log_path.read_text().splitlines():
        ts = ts_re.search(line)
        if ts:
            cur_ts = float(ts.group(1))
        an = ang_re.search(line)
        ac = acc_re.search(line)
        if an and ac and cur_ts is not None:
            timestamps.append(cur_ts)
            wx.append(float(an.group(1))); wy.append(float(an.group(2))); wz.append(float(an.group(3)))
            aax.append(float(ac.group(1))); aay.append(float(ac.group(2))); aaz.append(float(ac.group(3)))

    if not timestamps:
        print('  [skip] no IMU data found')
        return

    t = np.array(timestamps) - timestamps[0]
    fig, axs = plt.subplots(3, 2, sharex=True, figsize=(15, 10))
    fig.suptitle('IMU: Angular Velocity and Linear Acceleration', fontsize=13)

    ang_conf = list(zip([wx, wy, wz], ['ωx [rad/s]', 'ωy [rad/s]', 'ωz [rad/s]'], [CB[5], CB[3], CB[6]]))
    acc_conf = list(zip([aax, aay, aaz], ['ax [m/s²]', 'ay [m/s²]', 'az [m/s²]'], [CB[2], CB[1], CB[7]]))

    for i, (data, ylabel, c) in enumerate(ang_conf):
        avg = np.mean(data)
        axs[i, 0].plot(t, data, color=c, label=ylabel.split('[')[0].strip())
        axs[i, 0].axhline(avg, linestyle='--', color=CB[0], alpha=0.6, label=f'avg={avg:.4f}')
        axs[i, 0].set_ylabel(ylabel); axs[i, 0].legend()

    for i, (data, ylabel, c) in enumerate(acc_conf):
        avg = np.mean(data)
        axs[i, 1].plot(t, data, color=c, label=ylabel.split('[')[0].strip())
        axs[i, 1].axhline(avg, linestyle='--', color=CB[0], alpha=0.6, label=f'avg={avg:.4f}')
        axs[i, 1].set_ylabel(ylabel); axs[i, 1].legend()

    axs[2, 0].set_xlabel('Time [s]'); axs[2, 1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'imu', 'imu', 'imu_data.png')


# ---------------------------------------------------------------------------
# Simple odometry  (odom.txt)
# ---------------------------------------------------------------------------

def plot_odom(log_dir: Path) -> None:
    log_path = log_dir / 'odom.txt'
    if not log_path.exists():
        warn_missing(log_path)
        return

    ts_re  = re.compile(r'Timestamp:\s+([0-9]+\.[0-9]+)')
    pos_re = re.compile(r'Position:\s+x=([-\d\.]+),\s+y=([-\d\.]+),\s+z=([-\d\.]+)')
    ang_re = re.compile(r'Angular Velocity:\s+x=([-\d\.]+),\s+y=([-\d\.]+),\s+z=([-\d\.]+)')

    timestamps, xs, ys, zs, wx, wy, wz = [], [], [], [], [], [], []
    cur_ts = cur_pos = None
    for line in log_path.read_text().splitlines():
        ts = ts_re.search(line)
        if ts:
            cur_ts = float(ts.group(1)); continue
        pm = pos_re.search(line)
        if pm:
            cur_pos = (float(pm.group(1)), float(pm.group(2)), float(pm.group(3))); continue
        am = ang_re.search(line)
        if am and cur_ts is not None and cur_pos is not None:
            timestamps.append(cur_ts)
            xs.append(cur_pos[0]); ys.append(cur_pos[1]); zs.append(cur_pos[2])
            wx.append(float(am.group(1))); wy.append(float(am.group(2))); wz.append(float(am.group(3)))
            cur_ts = cur_pos = None

    if not timestamps:
        print('  [skip] no odometry data found')
        return

    t = np.array(timestamps) - timestamps[0]
    fig, axs = plt.subplots(3, 2, sharex=True, figsize=(14, 9))
    fig.suptitle('Robot Odometry: Position and Angular Velocity', fontsize=13)

    for i, (data, ylabel, c) in enumerate(zip(
        [xs, ys, zs], ['x [m]', 'y [m]', 'z [m]'], [CB[5], CB[3], CB[6]]
    )):
        axs[i, 0].plot(t, data, color=c); axs[i, 0].set_ylabel(ylabel)

    for i, (data, ylabel, c) in enumerate(zip(
        [wx, wy, wz], ['ωx [rad/s]', 'ωy [rad/s]', 'ωz [rad/s]'], [CB[2], CB[1], CB[7]]
    )):
        axs[i, 1].plot(t, data, color=c); axs[i, 1].set_ylabel(ylabel)

    axs[2, 0].set_xlabel('Time [s]'); axs[2, 1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'odom', 'odometry', 'simple_odom.png')


# ---------------------------------------------------------------------------
# Robot odometry CSV  (robot_odom.csv)
# ---------------------------------------------------------------------------

def plot_robot_odometry(log_dir: Path) -> None:
    csv = log_dir / 'robot_odom.csv'
    if not csv.exists():
        warn_missing(csv)
        return

    df = pd.read_csv(csv); df.columns = df.columns.str.strip()
    t = df['t'].values

    px = df['p_odom_x'].values; py = df['p_odom_y'].values; pz = df['p_odom_z'].values
    vx_d = np.gradient(px, t); vy_d = np.gradient(py, t); vz_d = np.gradient(pz, t)

    def _panel(ax, x, y, z, title, ylabel):
        _xyz_lines(ax, t, x, y, z)
        ax.set_title(title); ax.set_ylabel(ylabel)

    fig, axes = plt.subplots(4, 2, figsize=(16, 14), sharex=True)
    fig.suptitle('Robot Odometry (CSV)', fontsize=13)

    _panel(axes[0, 0], px, py, pz,
           'Base Position (Odom)', 'Position [m]')
    _panel(axes[0, 1], df['v_odom_x'].values, df['v_odom_y'].values, df['v_odom_z'].values,
           'Base Velocity (Odom)', 'Velocity [m/s]')
    _panel(axes[1, 0], df['p_cL_odom_x'].values, df['p_cL_odom_y'].values, df['p_cL_odom_z'].values,
           'Left Contact Position (Odom)', 'Position [m]')
    _panel(axes[1, 1], df['p_cR_odom_x'].values, df['p_cR_odom_y'].values, df['p_cR_odom_z'].values,
           'Right Contact Position (Odom)', 'Position [m]')

    for c, d, lbl in zip([CB[5], CB[3], CB[6]], [vx_d, vy_d, vz_d], ['vx_diff', 'vy_diff', 'vz_diff']):
        axes[2, 0].plot(t, d, '--', color=c, label=lbl)
    axes[2, 0].set_title('Velocity from Differentiated Position')
    axes[2, 0].set_ylabel('Velocity [m/s]'); axes[2, 0].legend()

    has_wheels = 'w_l_odom' in df.columns and 'w_r_odom' in df.columns
    if has_wheels:
        axes[2, 1].plot(t, df['w_l_odom'].values, color=CB[5], label='left')
        axes[2, 1].plot(t, df['w_r_odom'].values, color=CB[6], label='right')
        wl_avg = df['w_l_odom'].mean(); wr_avg = df['w_r_odom'].mean()
        axes[2, 1].axhline(wl_avg, linestyle='--', color=CB[5], alpha=0.6,
                           label=f'avg L={wl_avg:.3f}')
        axes[2, 1].axhline(wr_avg, linestyle='--', color=CB[6], alpha=0.6,
                           label=f'avg R={wr_avg:.3f}')
        axes[2, 1].set_title('Wheel Angular Velocity')
        axes[2, 1].set_ylabel('Angular Vel [rad/s]'); axes[2, 1].legend()

    for ax in [axes[3, 0], axes[3, 1]]:
        ax.set_visible(False)
    axes[2, 0].set_xlabel('Time [s]'); axes[2, 1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'odom', 'odometry', 'robot_odom.png')


# ---------------------------------------------------------------------------
# MPC reference plan  (/tmp/plan/x.txt)
# ---------------------------------------------------------------------------

def plot_plan(tmp_dir: Path) -> None:
    x_path = tmp_dir / 'plan' / 'x.txt'
    if not x_path.exists():
        warn_missing(x_path)
        return

    x = np.loadtxt(x_path, ndmin=2)
    DT_MS = 2; VEC_OFF_Y = 0.567 / 2

    pcom_x, pcom_y, pcom_z = x[:, 0], x[:, 1], x[:, 2]
    c_x,    c_y,    c_z    = x[:, 6], x[:, 7], x[:, 8]
    theta = x[:, 10]
    off_x = -np.sin(theta) * VEC_OFF_Y
    off_y =  np.cos(theta) * VEC_OFF_Y
    cL_x, cL_y = c_x + off_x, c_y + off_y
    cR_x, cR_y = c_x - off_x, c_y - off_y
    t_ms = np.arange(x.shape[0]) * DT_MS

    fig, axes = plt.subplots(1, 2, figsize=(15, 6), layout='constrained')
    ax0, ax1 = axes

    ax0.plot(pcom_x, pcom_y, color=CB[5], lw=2, label='CoM')
    ax0.plot(c_x,    c_y,    '--', color=CB[1], lw=2, label='c')
    ax0.plot(cL_x,   cL_y,   '-.', color=CB[3], lw=1.5, label='cL')
    ax0.plot(cR_x,   cR_y,   ':',  color=CB[6], lw=1.5, label='cR')
    ax0.plot(pcom_x[0],  pcom_y[0],  'o',  color=CB[0], ms=8, label='start')
    ax0.plot(pcom_x[-1], pcom_y[-1], 'x',  color=CB[7], ms=10, mew=2, label='end')
    all_x = np.concatenate([pcom_x, c_x, cL_x, cR_x])
    all_y = np.concatenate([pcom_y, c_y, cL_y, cR_y])
    pad = lambda v: 0.1 * (v.max() - v.min() + 1e-9)
    ax0.set_xlim(all_x.min() - pad(all_x), all_x.max() + pad(all_x))
    ax0.set_ylim(all_y.min() - pad(all_y), all_y.max() + pad(all_y))
    ax0.set_xlabel('x [m]'); ax0.set_ylabel('y [m]')
    ax0.set_title('X–Y Reference Plan'); ax0.legend()

    ax1.plot(t_ms, pcom_z, color=CB[5], lw=2, label='CoM z')
    ax1.plot(t_ms, c_z,    '--', color=CB[1], lw=2, label='c z')
    ax1.set_xlabel('t [ms]'); ax1.set_ylabel('z [m]')
    ax1.set_title('Z Reference Plan'); ax1.legend()
    save(fig, 'plan', 'plan', 'reference_plan.png')

    jump_path = tmp_dir / 'plan' / 'jump_traj.txt'
    if jump_path.exists():
        lines_txt = jump_path.read_text().strip().splitlines()
        t0_j = float(lines_txt[0].strip().split()[0])
        x_j  = np.loadtxt(lines_txt[1:], ndmin=2)
        t_j  = np.arange(x_j.shape[0]) * DT_MS + t0_j
        fig_j, ax_j = plt.subplots(figsize=(12, 5), layout='constrained')
        ax_j.plot(t_j, x_j[:, 2], color=CB[5], lw=2, label='CoM z')
        ax_j.plot(t_j, x_j[:, 8], '--', color=CB[1], lw=2, label='c z')
        ax_j.set_xlabel('t [ms]'); ax_j.set_ylabel('z [m]')
        ax_j.set_title('Jump Trajectory Z Plan'); ax_j.legend()
        save(fig_j, 'plan', 'plan', 'jump_z.png')


# ---------------------------------------------------------------------------
# Controller timings  (/tmp/timing_log.txt)
# ---------------------------------------------------------------------------

def plot_timings(tmp_dir: Path) -> None:
    log = tmp_dir / 'timing_log.txt'
    if not log.exists():
        warn_missing(log)
        return

    mpc_t, wbc_t, total_t = [], [], []
    for line in log.read_text().splitlines()[1:]:
        parts = line.strip().split(',')
        if len(parts) >= 3:
            try:
                mpc_t.append(float(parts[0]))
                wbc_t.append(float(parts[1]))
                total_t.append(float(parts[2]))
            except ValueError:
                continue

    if not mpc_t:
        print('  [skip] no timing data')
        return

    cycles = np.arange(len(mpc_t))
    fig, ax = plt.subplots(figsize=(13, 5))
    ax.plot(cycles, mpc_t,   color=CB[5], label='MPC')
    ax.plot(cycles, wbc_t,   color=CB[3], label='WBC')
    ax.plot(cycles, total_t, color=CB[6], label='Total')
    for data, c, lbl in zip(
        [mpc_t, wbc_t, total_t], [CB[5], CB[3], CB[6]], ['MPC', 'WBC', 'Total']
    ):
        avg = np.mean(data)
        ax.axhline(avg, linestyle='--', color=c, alpha=0.65, label=f'{lbl} avg={avg:.1f} µs')
    ax.set_xlabel('Control cycle'); ax.set_ylabel('Time [µs]')
    ax.set_title('Controller Timing'); ax.legend(ncol=2)
    fig.tight_layout()
    save(fig, 'timings', 'timings', 'controller_timing.png')


# ---------------------------------------------------------------------------
# WBC tracking errors  (/tmp/wbc_log.txt  +  /tmp/plan/x.txt)
# ---------------------------------------------------------------------------

def plot_wbc_tracking(tmp_dir: Path) -> None:
    wbc_log = tmp_dir / 'wbc_log.txt'
    if not wbc_log.exists():
        warn_missing(wbc_log)
        return

    log_data = np.loadtxt(wbc_log, delimiter=',', skiprows=1)
    t_ms    = log_data[:, 0]
    com_act = log_data[:, 1:4];  com_des = log_data[:, 4:7]
    wl_act  = log_data[:, 7:10]; wl_des  = log_data[:, 10:13]
    wr_act  = log_data[:, 13:16]; wr_des = log_data[:, 16:19]

    fig, axs = plt.subplots(3, 1, figsize=(13, 11), sharex=True)
    fig.suptitle('WBC Tracking Errors (desired − actual)', fontsize=13)
    labels = ['x', 'y', 'z']
    for ax, err, title in zip(
        axs,
        [com_des - com_act, wl_des - wl_act, wr_des - wr_act],
        ['CoM Error', 'Left Wheel Error', 'Right Wheel Error'],
    ):
        for c, d, lbl in zip([CB[5], CB[3], CB[6]], err.T, labels):
            ax.plot(t_ms, d, color=c, label=f'{lbl}')
        ax.set_ylabel('Error [m]'); ax.set_title(title); ax.legend()
    axs[-1].set_xlabel('Time [ms]')
    plt.subplots_adjust(hspace=0.35)
    save(fig, 'wbc', 'wbc', 'tracking_errors.png')

    plan_file = tmp_dir / 'plan' / 'x.txt'
    if plan_file.exists():
        plan_data = np.loadtxt(plan_file, ndmin=2)
        com_plan  = plan_data[:, 0:3]
        n_cur = len(com_act)
        if len(com_plan) < n_cur:
            com_plan = np.vstack([com_plan,
                                   np.tile(com_plan[-1], (n_cur - len(com_plan), 1))])
        else:
            com_plan = com_plan[:n_cur]
        err_plan = com_plan - com_act[:n_cur]
        t_c = t_ms[:n_cur]

        fig2, axs2 = plt.subplots(3, 1, figsize=(13, 11), sharex=True)
        fig2.suptitle('CoM Error: Reference Plan − Current', fontsize=13)
        for ax, c, d, lbl in zip(axs2, [CB[5], CB[3], CB[6]], err_plan.T, labels):
            ax.plot(t_c, d, color=c)
            ax.set_ylabel('Error [m]')
            ax.set_title(f'CoM {lbl} error (reference − current)')
        axs2[-1].set_xlabel('Time [ms]')
        plt.subplots_adjust(hspace=0.35)
        save(fig2, 'wbc', 'wbc', 'reference_plan_errors.png')


# ---------------------------------------------------------------------------
# MPC prediction snapshot  (/tmp/mpc_data/{t:.6f}/x.txt + u.txt)
# ---------------------------------------------------------------------------

def plot_mpc_snapshot(tmp_dir: Path, t_msec: float) -> None:
    folder   = tmp_dir / 'mpc_data' / f'{t_msec:.6f}'
    x_path   = folder / 'x.txt'
    u_path   = folder / 'u.txt'
    if not x_path.exists():
        warn_missing(x_path)
        return

    data   = np.loadtxt(x_path)
    data_u = np.loadtxt(u_path)
    com_x, com_y, com_z = data[:, 0], data[:, 1], data[:, 2]
    pc_x,  pc_y         = data[:, 6], data[:, 7]
    fz                  = data_u[:, -1]

    steps = np.arange(len(com_x))
    fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)
    fig.suptitle(f'MPC Prediction Snapshot — t = {t_msec:.3f} ms', fontsize=13)

    axs[0].plot(np.arange(len(fz)), fz, color=CB[6])
    axs[0].set_ylabel('Force Z [N]'); axs[0].set_title('Vertical Force')

    axs[1].plot(steps, com_x, color=CB[5], label='CoM X')
    axs[1].plot(steps, pc_x,  color=CB[1], label='Pc X')
    axs[1].set_ylabel('X [m]'); axs[1].legend()

    axs[2].plot(steps, com_y, color=CB[5], label='CoM Y')
    axs[2].plot(steps, pc_y,  color=CB[1], label='Pc Y')
    axs[2].set_ylabel('Y [m]'); axs[2].legend()

    axs[3].plot(steps, com_z, color=CB[5], label='CoM Z')
    axs[3].set_ylabel('Z [m]'); axs[3].set_title('CoM Z'); axs[3].legend()
    axs[-1].set_xlabel('Prediction step')

    fig.tight_layout()
    save(fig, 'mpc', 'mpc', f'snapshot_{t_msec:.3f}.png')


# ---------------------------------------------------------------------------
# Joint torques / velocities  (/tmp/joint_eff.txt, /tmp/joint_vel.txt)
# ---------------------------------------------------------------------------

def plot_joint_tau(tmp_dir: Path) -> None:
    configs = [
        ('joint_eff.txt', 'Torque [Nm]',     'Joint Commanded Torques',    'torques'),
        ('joint_vel.txt', 'Velocity [rad/s]', 'Joint Commanded Velocities', 'velocities'),
    ]
    for fname, ylabel, title, subdir in configs:
        fpath = tmp_dir / fname
        if not fpath.exists():
            warn_missing(fpath)
            continue
        data = np.loadtxt(fpath)
        if data.ndim == 1:
            data = data[:, np.newaxis]
        n_steps, n_joints = data.shape
        t = np.arange(n_steps)
        fig, ax = plt.subplots(figsize=(13, 5))
        for j in range(n_joints):
            ax.plot(t, data[:, j], color=CB[j % len(CB)], label=f'Joint {j + 1}')
        ax.set_xlabel('Control cycle'); ax.set_ylabel(ylabel)
        ax.set_title(title); ax.legend(ncol=max(1, n_joints // 4))
        fig.tight_layout()
        save(fig, 'joints', 'joints', subdir, fname.replace('.txt', '.png'))


# ---------------------------------------------------------------------------
# Per-joint state  (joint_state_log.txt)
# ---------------------------------------------------------------------------

def _parse_joint_state(log_path: Path, joint: str):
    pattern = re.compile(
        rf'(?P<time>\d+\.\d+)\s+\d+\.\d+\s+{re.escape(joint)}:\s+'
        rf'pos:\s+(?P<pos>[-+]?\d*\.\d+)\s+vel:\s+(?P<vel>[-+]?\d*\.\d+)'
        rf'\s+effort:\s+(?P<effort>[-+]?\d*\.\d+)'
    )
    times, positions, velocities, efforts = [], [], [], []
    for line in log_path.read_text().splitlines():
        m = pattern.search(line)
        if m:
            times.append(float(m.group('time')))
            positions.append(float(m.group('pos')))
            velocities.append(float(m.group('vel')))
            efforts.append(float(m.group('effort')))
    return times, positions, velocities, efforts


def plot_joint_state(log_dir: Path, joint_names: list) -> None:
    log_path = log_dir / 'joint_state_log.txt'
    if not log_path.exists():
        warn_missing(log_path)
        return

    for joint in joint_names:
        times, pos, vel, eff = _parse_joint_state(log_path, joint)
        if not times:
            print(f'  [skip] no state data for {joint}')
            continue
        t = np.array(times) - times[0]

        fig, axs = plt.subplots(3, 1, sharex=True, figsize=(12, 9))
        fig.suptitle(f'Joint State: {joint}', fontsize=13)
        conf = [
            (pos, 'Position [rad]',  'Position', CB[5]),
            (vel, 'Velocity [rad/s]','Velocity',  CB[3]),
            (eff, 'Effort [Nm]',     'Effort',    CB[6]),
        ]
        for ax, (data, ylabel, title, c) in zip(axs, conf):
            avg = np.mean(data)
            ax.plot(t, data, color=c, label=title)
            ax.axhline(avg, linestyle='--', color=CB[0], alpha=0.6, label=f'avg={avg:.3f}')
            ax.set_ylabel(ylabel); ax.set_title(title); ax.legend()
        axs[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'joints', 'joints', 'state', f'{joint}.png')


# ---------------------------------------------------------------------------
# Numeric derivative  (numeric_derivative.txt)
# ---------------------------------------------------------------------------

def plot_numeric_derivative(log_dir: Path, joint_names: list) -> None:
    log_file = log_dir / 'numeric_derivative.txt'
    if not log_file.exists():
        warn_missing(log_file)
        return

    content = log_file.read_text().splitlines()
    for joint in joint_names:
        pattern = re.compile(
            rf'(?P<time>\d+\.\d+)\s+\d+\.\d+\s+{re.escape(joint)}:\s+'
            rf'pos:\s+(?P<pos>[-+]?\d*\.\d+)\s+numeric_vel:\s+(?P<numvel>[-+]?\d*\.\d+)'
        )
        times, positions, num_vels = [], [], []
        for line in content:
            m = pattern.search(line)
            if m:
                times.append(float(m.group('time')))
                positions.append(float(m.group('pos')))
                num_vels.append(float(m.group('numvel')))
        if not times:
            print(f'  [skip] no numeric derivative data for {joint}')
            continue
        t = np.array(times) - times[0]

        fig, axs = plt.subplots(2, 1, sharex=True, figsize=(12, 7))
        fig.suptitle(f'Numeric Derivative: {joint}', fontsize=13)
        for ax, data, ylabel, title, c in zip(
            axs,
            [positions, num_vels],
            ['Position [rad]', 'Numeric Velocity [rad/s]'],
            ['Position', 'Numeric Derivative'],
            [CB[5], CB[3]],
        ):
            avg = np.mean(data)
            ax.plot(t, data, color=c, label=title)
            ax.axhline(avg, linestyle='--', color=CB[0], alpha=0.6, label=f'avg={avg:.3f}')
            ax.set_ylabel(ylabel); ax.set_title(title); ax.legend()
        axs[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'joints', 'joints', 'numeric_derivative', f'{joint}.png')


# ---------------------------------------------------------------------------
# Tau commanded + joint state  (tau_commanded.txt + joint_state_log.txt)
# ---------------------------------------------------------------------------

def plot_tau_velocity(log_dir: Path, joint_names: list) -> None:
    tau_file = log_dir / 'tau_commanded.txt'
    state_file = log_dir / 'joint_state_log.txt'

    if not tau_file.exists() and not state_file.exists():
        print('  [skip] neither tau_commanded.txt nor joint_state_log.txt found')
        return

    # Reference t0 from joint state file
    t0_state = None
    if state_file.exists():
        for line in state_file.read_text().splitlines():
            m = re.match(r'\d+\.\d+\s+(\d+\.\d+)', line)
            if m:
                t0_state = float(m.group(1))
                break

    for joint in joint_names:
        tau_d   = defaultdict(list)
        state_d = defaultdict(list)

        if tau_file.exists() and t0_state is not None:
            cur_time = None
            for line in tau_file.read_text().splitlines():
                ts = re.match(r'^(\d+\.\d+)', line)
                if ts:
                    cur_time = float(ts.group(1))
                m = re.search(
                    rf'({re.escape(joint)}).*effort commanded:\s+([-\d\.eE]+)'
                    rf'.*vel_target:\s+([-\d\.eE]+)\s+vel_feedback:\s+([-\d\.eE]+)',
                    line,
                )
                if m and cur_time is not None:
                    tau_d['t'].append(cur_time - t0_state)
                    tau_d['tau_cmd'].append(float(m.group(2)))
                    tau_d['target'].append(float(m.group(3)))
                    tau_d['vel'].append(float(m.group(4)))

        if state_file.exists() and t0_state is not None:
            pattern = re.compile(
                rf'\d+\.\d+\s+(\d+\.\d+)\s+{re.escape(joint)}:'
                rf'.*vel:\s+([-\d\.eE]+)\s+effort:\s+([-\d\.eE]+)'
            )
            for line in state_file.read_text().splitlines():
                m = pattern.search(line)
                if m:
                    state_d['t'].append(float(m.group(1)) - t0_state)
                    state_d['vel'].append(float(m.group(2)))
                    state_d['tau'].append(float(m.group(3)))

        if not tau_d['t'] and not state_d['t']:
            print(f'  [skip] no tau/vel data for {joint}')
            continue

        fig, axs = plt.subplots(2, 1, sharex=True, figsize=(13, 9))
        fig.suptitle(f'Torque & Velocity Control: {joint}', fontsize=13)

        if state_d['t']:
            axs[0].plot(state_d['t'], state_d['vel'], color=CB[5], label='measured vel')
        if tau_d['t']:
            axs[0].plot(tau_d['t'], tau_d['vel'],    color=CB[2], label='vel feedback')
            axs[0].plot(tau_d['t'], tau_d['target'], '--', color=CB[1], label='vel target')
        axs[0].set_ylabel('Velocity [rad/s]'); axs[0].legend()

        if tau_d['t']:
            axs[1].plot(tau_d['t'], tau_d['tau_cmd'], color=CB[6], label='tau commanded')
        if state_d['t']:
            axs[1].plot(state_d['t'], state_d['tau'], color=CB[3], label='tau measured')
        axs[1].set_xlabel('Time [s]'); axs[1].set_ylabel('Torque [Nm]'); axs[1].legend()
        fig.tight_layout()
        save(fig, 'joints', 'joints', 'tau_velocity', f'{joint}.png')


# ---------------------------------------------------------------------------
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Generate TITA controller plots.\n'
            'Default: save every group to images/.\n'
            'Pass --show-<group> to open that group interactively instead '
            '(enables zooming/panning); other groups still save normally.\n'
            'Multiple --show-* flags are allowed.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--log-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Directory with robot log files  [default: {ROBOT_LOGS}]')
    parser.add_argument('--tmp-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Temporary data directory  [default: {ROBOT_LOGS}]')
    parser.add_argument('--joints', nargs='+',
                        default=['joint_left_leg_4', 'joint_right_leg_4'],
                        help='Joint names for per-joint plots')
    parser.add_argument('--mpc-t', type=float, default=None,
                        metavar='TIMESTEP_MS',
                        help='MPC snapshot timestep (ms) — enables mpc group')
    parser.add_argument('--only', nargs='+', default=None,
                        metavar='GROUP',
                        help='Run only the listed groups: '
                             'encoder estimation imu odom plan timings wbc mpc joints')

    show_group = parser.add_argument_group(
        'interactive display (show instead of save)'
    )
    for g in sorted(_ALL_GROUPS):
        show_group.add_argument(f'--show-{g}', action='store_true',
                                help=f'Show {g} plots interactively')

    args = parser.parse_args()

    # If --show-X flags are given without --only, restrict execution to those groups only.
    only = set(args.only) if args.only else (_SHOW if _SHOW else None)

    def run(group: str, fn, *a, **kw) -> None:
        if only and group not in only:
            return
        print(f'\n[{group}]')
        try:
            fn(*a, **kw)
        except Exception as exc:
            print(f'  ERROR in {fn.__name__}: {exc}')

    run('encoder',    plot_encoder_wheels,      args.log_dir)
    run('estimation', plot_estimation,          args.log_dir)
    run('imu',        plot_imu,                 args.log_dir)
    run('odom',       plot_odom,                args.log_dir)
    run('odom',       plot_robot_odometry,      args.log_dir)
    run('plan',       plot_plan,                args.tmp_dir)
    run('timings',    plot_timings,             args.tmp_dir)
    run('wbc',        plot_wbc_tracking,        args.tmp_dir)
    run('joints',     plot_joint_tau,           args.tmp_dir)
    run('joints',     plot_joint_state,         args.log_dir, args.joints)
    run('joints',     plot_numeric_derivative,  args.log_dir, args.joints)
    run('joints',     plot_tau_velocity,        args.log_dir, args.joints)
    if args.mpc_t is not None:
        run('mpc',    plot_mpc_snapshot,        args.tmp_dir, args.mpc_t)

    if _SHOW:
        shown = ', '.join(sorted(_SHOW))
        print(f'\nOpening interactive windows for: {shown}')
        print('Close all windows to exit.')
        plt.show()

    saved = _ALL_GROUPS - _SHOW
    if saved and not only:
        print(f'\nImages saved under  {OUT_DIR.relative_to(OUT_DIR.parent.parent)}/')


if __name__ == '__main__':
    main()
