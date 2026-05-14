#!/usr/bin/env python3
"""
TITA controller plotting script.

Images are saved to images/ (relative to this script), organised as:
  filter/           kf_test.csv  — single summary plot of KF state
  wbc/
    com_task/       wbc_log.txt  — CoM position & velocity (actual vs desired)
    wheel_task/     wbc_log.txt  — Wheel position & velocity (actual vs desired)
    torso_task/     wbc_log.txt  — Torso orientation (skipped if missing)
    wbc_solution/   wbc_log.txt + joint_state_log.txt + tau_commanded_log.txt
  timings/          timing_log.txt
  feedback/
    imu/            imu_log.txt  — raw IMU sensor data
    joints/         joint_state_log.txt  — raw joint feedback

Legacy groups (still accessible via --only encoder|odom|plan):
  encoder   wheel_log.txt
  odom      odom.txt, robot_odom.csv
  plan      plan/x.txt  [+ jump_traj.txt]

Pass --show-<group> to view a group interactively instead of saving.
Pass --only <group> [<group>...] to run only those groups.
"""

import argparse
import re
import sys
import numpy as np
import pandas as pd
import matplotlib

_ALL_GROUPS = {'filter', 'wbc', 'timings', 'feedback', 'encoder', 'odom', 'plan'}
_DEFAULT_GROUPS = {'filter', 'wbc', 'timings', 'feedback'}
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
ROBOT_LOGS = Path.cwd() / 'robot_logs'
OUT_DIR = SCRIPT_DIR / 'images'

JOINT_NAMES = [
    'joint_left_leg_1', 'joint_left_leg_2', 'joint_left_leg_3', 'joint_left_leg_4',
    'joint_right_leg_1', 'joint_right_leg_2', 'joint_right_leg_3', 'joint_right_leg_4',
]

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def save(fig, group: str, *parts: str) -> None:
    if group in _SHOW:
        print(f'  queued {"/".join(parts)}')
        return
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


def _xyz_comparison(ax, t_act, act_x, act_y, act_z, t_des, des_x, des_y, des_z):
    """Plot actual (solid) vs desired (dashed) for x, y, z on a single axis."""
    for d_act, d_des, c, axis in zip(
        [act_x, act_y, act_z], [des_x, des_y, des_z],
        [CB[5], CB[3], CB[6]], ['x', 'y', 'z']
    ):
        ax.plot(t_act, d_act, '-',  color=c, label=f'{axis} actual')
        ax.plot(t_des, d_des, '--', color=c, alpha=0.75, label=f'{axis} des')
    ax.legend(ncol=2, fontsize=8)


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
# Data loaders  (all files now live in robot_logs/)
# ---------------------------------------------------------------------------

def _load_csv(path: Path) -> pd.DataFrame | None:
    if not path.exists():
        warn_missing(path)
        return None
    try:
        return pd.read_csv(path)
    except Exception as e:
        print(f'  [error] reading {path}: {e}')
        return None


def _load_wbc(log_dir: Path) -> pd.DataFrame | None:
    return _load_csv(log_dir / 'wbc_log.txt')


def _load_joint_state(log_dir: Path) -> pd.DataFrame | None:
    return _load_csv(log_dir / 'joint_state_log.txt')


def _load_imu(log_dir: Path) -> pd.DataFrame | None:
    return _load_csv(log_dir / 'imu_log.txt')


def _load_tau_commanded(log_dir: Path) -> pd.DataFrame | None:
    df = _load_csv(log_dir / 'tau_commanded_log.txt')
    if df is None:
        df = _load_csv(log_dir / 'tau_commanded.txt')   # legacy name
    return df


# ---------------------------------------------------------------------------
# filter  (kf_test.csv) — single summary figure
# ---------------------------------------------------------------------------

def plot_filter(log_dir: Path) -> None:
    csv = log_dir / 'kf_test.csv'
    if not csv.exists():
        warn_missing(csv)
        return

    df = pd.read_csv(csv)
    if df.empty:
        print('  [skip] kf_test.csv has no data rows')
        return
    t = df['t'].values - df['t'].iloc[0]

    has_quat = all(c in df.columns for c in ['qx', 'qy', 'qz', 'qw'])

    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    fig.suptitle('KF State Estimation — Filter Output', fontsize=13)

    # [0,0] estimated base position
    ax = axes[0, 0]
    _xyz_lines(ax, t, df['p_est_x'].values, df['p_est_y'].values, df['p_est_z'].values)
    ax.set_ylabel('Position [m]'); ax.set_title('Estimated Base Position')

    # [0,1] estimated base velocity
    ax = axes[0, 1]
    _xyz_lines(ax, t, df['v_est_x'].values, df['v_est_y'].values, df['v_est_z'].values)
    ax.set_ylabel('Velocity [m/s]'); ax.set_title('Estimated Base Velocity')

    # [1,0] left contact point
    ax = axes[1, 0]
    _xyz_lines(ax, t,
               df['p_cL_est_x'].values, df['p_cL_est_y'].values, df['p_cL_est_z'].values)
    ax.set_ylabel('Position [m]'); ax.set_title('Left Contact Point (est)')

    # [1,1] right contact point
    ax = axes[1, 1]
    _xyz_lines(ax, t,
               df['p_cR_est_x'].values, df['p_cR_est_y'].values, df['p_cR_est_z'].values)
    ax.set_ylabel('Position [m]'); ax.set_title('Right Contact Point (est)')

    # [2,0] orientation RPY
    if has_quat:
        q = _fix_quat_continuity(df[['qx', 'qy', 'qz', 'qw']].values)
        roll, pitch, yaw = _quat_to_rpy(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
        ax = axes[2, 0]
        for d, lbl, c in zip(
            [np.unwrap(roll), np.unwrap(pitch), np.unwrap(yaw)],
            ['roll', 'pitch', 'yaw'], [CB[1], CB[2], CB[5]]
        ):
            ax.plot(t, d, color=c, label=lbl)
        ax.set_ylabel('Angle [rad]'); ax.set_title('Orientation (RPY)'); ax.legend()
    else:
        axes[2, 0].set_visible(False)

    # [2,1] XY trajectory
    ax = axes[2, 1]
    ax.plot(df['p_est_x'].values, df['p_est_y'].values, color=CB[5])
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('XY Trajectory'); ax.set_aspect('equal', adjustable='datalim')

    for col in range(2):
        axes[2, col].set_xlabel('Time [s]')
    axes[2, 1].set_xlabel('x [m]')
    fig.tight_layout()
    save(fig, 'filter', 'filter', 'filter_summary.png')


# ---------------------------------------------------------------------------
# wbc — task tracking and WBC solution
# ---------------------------------------------------------------------------

def _wbc_time(df: pd.DataFrame) -> np.ndarray:
    return (df['time_ms'].values - df['time_ms'].iloc[0]) / 1000.0


def plot_wbc_com(df: pd.DataFrame) -> None:
    t = _wbc_time(df)

    # --- Position ---
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('WBC CoM Task — Position Comparison', fontsize=13)
    for ax, axis in zip(axes, ['x', 'y', 'z']):
        ax.plot(t, df[f'com_{axis}'].values,     '-',  color=CB[5], label='actual')
        ax.plot(t, df[f'com_{axis}_des'].values, '--', color=CB[6], label='desired')
        ax.set_ylabel(f'{axis} [m]'); ax.set_title(f'CoM {axis}'); ax.legend()
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'wbc', 'wbc', 'com_task', 'comparison', 'position.png')

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('WBC CoM Task — Position Error', fontsize=13)
    for ax, axis in zip(axes, ['x', 'y', 'z']):
        err = df[f'com_{axis}_des'].values - df[f'com_{axis}'].values
        ax.plot(t, err, color=CB[6])
        ax.axhline(0, linestyle='--', color=CB[0], alpha=0.4)
        ax.set_ylabel(f'err {axis} [m]'); ax.set_title(f'CoM {axis} error')
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'wbc', 'wbc', 'com_task', 'error', 'position_error.png')

    if 'com_vx' not in df.columns:
        return

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('WBC CoM Task — Velocity Comparison', fontsize=13)
    for ax, axis in zip(axes, ['x', 'y', 'z']):
        ax.plot(t, df[f'com_v{axis}'].values,     '-',  color=CB[5], label='actual')
        ax.plot(t, df[f'com_v{axis}_des'].values, '--', color=CB[6], label='desired')
        ax.set_ylabel(f'v{axis} [m/s]'); ax.set_title(f'CoM v{axis}'); ax.legend()
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'wbc', 'wbc', 'com_task', 'comparison', 'velocity.png')

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('WBC CoM Task — Velocity Error', fontsize=13)
    for ax, axis in zip(axes, ['x', 'y', 'z']):
        err = df[f'com_v{axis}_des'].values - df[f'com_v{axis}'].values
        ax.plot(t, err, color=CB[6])
        ax.axhline(0, linestyle='--', color=CB[0], alpha=0.4)
        ax.set_ylabel(f'err v{axis} [m/s]'); ax.set_title(f'CoM v{axis} error')
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'wbc', 'wbc', 'com_task', 'error', 'velocity_error.png')

    if 'com_ax_des' not in df.columns:
        return

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('WBC CoM Task — Desired Acceleration', fontsize=13)
    for ax, axis, c in zip(axes, ['x', 'y', 'z'], [CB[5], CB[3], CB[6]]):
        ax.plot(t, df[f'com_a{axis}_des'].values, color=c)
        ax.set_ylabel(f'a{axis} [m/s²]'); ax.set_title(f'CoM a{axis} des')
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'wbc', 'wbc', 'com_task', 'comparison', 'acceleration_des.png')


def plot_wbc_wheels(df: pd.DataFrame) -> None:
    t = _wbc_time(df)

    for side, prefix in [('Left', 'wheel_l'), ('Right', 'wheel_r')]:
        fname = prefix.replace('wheel_', '')

        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'WBC {side} Wheel — Position Comparison', fontsize=13)
        for ax, axis in zip(axes, ['x', 'y', 'z']):
            ax.plot(t, df[f'{prefix}_{axis}'].values,     '-',  color=CB[5], label='actual')
            ax.plot(t, df[f'{prefix}_{axis}_des'].values, '--', color=CB[6], label='desired')
            ax.set_ylabel(f'{axis} [m]'); ax.set_title(f'{side} Wheel {axis}'); ax.legend()
        axes[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'wbc', 'wbc', 'wheel_task', 'comparison', f'{fname}_position.png')

        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'WBC {side} Wheel — Position Error', fontsize=13)
        for ax, axis in zip(axes, ['x', 'y', 'z']):
            err = df[f'{prefix}_{axis}_des'].values - df[f'{prefix}_{axis}'].values
            ax.plot(t, err, color=CB[6])
            ax.axhline(0, linestyle='--', color=CB[0], alpha=0.4)
            ax.set_ylabel(f'err {axis} [m]'); ax.set_title(f'{side} Wheel {axis} error')
        axes[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'wbc', 'wbc', 'wheel_task', 'error', f'{fname}_position_error.png')

        if f'{prefix}_vx' not in df.columns:
            continue

        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'WBC {side} Wheel — Velocity Comparison', fontsize=13)
        for ax, axis in zip(axes, ['x', 'y', 'z']):
            ax.plot(t, df[f'{prefix}_v{axis}'].values,     '-',  color=CB[5], label='actual')
            ax.plot(t, df[f'{prefix}_v{axis}_des'].values, '--', color=CB[6], label='desired')
            ax.set_ylabel(f'v{axis} [m/s]'); ax.set_title(f'{side} Wheel v{axis}'); ax.legend()
        axes[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'wbc', 'wbc', 'wheel_task', 'comparison', f'{fname}_velocity.png')

        fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
        fig.suptitle(f'WBC {side} Wheel — Velocity Error', fontsize=13)
        for ax, axis in zip(axes, ['x', 'y', 'z']):
            err = df[f'{prefix}_v{axis}_des'].values - df[f'{prefix}_v{axis}'].values
            ax.plot(t, err, color=CB[6])
            ax.axhline(0, linestyle='--', color=CB[0], alpha=0.4)
            ax.set_ylabel(f'err v{axis} [m/s]'); ax.set_title(f'{side} Wheel v{axis} error')
        axes[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'wbc', 'wbc', 'wheel_task', 'error', f'{fname}_velocity_error.png')


def plot_wbc_torso(df: pd.DataFrame) -> None:
    # torso columns: torso_x=roll, torso_y=pitch, torso_z=yaw  (actual + desired)
    if 'torso_x' not in df.columns:
        print('  [skip] no torso task columns in wbc_log.txt')
        return

    t = _wbc_time(df)
    labels   = ['Roll', 'Pitch', 'Yaw']
    act_cols = ['torso_x',     'torso_y',     'torso_z']
    des_cols = ['torso_x_des', 'torso_y_des', 'torso_z_des']

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('WBC Torso Task — Orientation Comparison', fontsize=13)
    for ax, lbl, ac, dc in zip(axes, labels, act_cols, des_cols):
        if ac in df.columns:
            ax.plot(t, df[ac].values, '-',  color=CB[5], label='actual')
        if dc in df.columns:
            ax.plot(t, df[dc].values, '--', color=CB[6], label='desired')
        ax.set_ylabel(f'{lbl} [rad]'); ax.set_title(lbl); ax.legend()
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'wbc', 'wbc', 'torso_task', 'comparison', 'orientation.png')

    # angular velocity desired
    if 'torso_vx_des' not in df.columns:
        return
    fig, axes = plt.subplots(3, 1, figsize=(13, 10), sharex=True)
    fig.suptitle('WBC Torso Task — Desired Angular Velocity', fontsize=13)
    for ax, axis, c in zip(axes, ['x', 'y', 'z'], [CB[5], CB[3], CB[6]]):
        ax.plot(t, df[f'torso_v{axis}_des'].values, color=c)
        ax.set_ylabel(f'ω{axis} [rad/s]')
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'wbc', 'wbc', 'torso_task', 'comparison', 'angular_velocity_des.png')


def plot_wbc_solution(
    wbc_df: pd.DataFrame,
    js_df: pd.DataFrame | None,
    tau_df: pd.DataFrame | None,
    joint_names: list,
) -> None:
    t_wbc = _wbc_time(wbc_df)

    if js_df is not None and js_df.empty:
        js_df = None
    if tau_df is not None and tau_df.empty:
        tau_df = None

    t_js = t_tau = None
    if js_df is not None:
        t_js = js_df['time_stamp_s'].values - js_df['time_stamp_s'].iloc[0]
    if tau_df is not None and 'time_s' in tau_df.columns:
        t_tau = tau_df['time_s'].values - tau_df['time_s'].iloc[0]
    elif tau_df is not None:
        t_tau = np.arange(len(tau_df)) * 0.002

    for joint in joint_names:
        sol_pos_col = f'sol_{joint}_pos'
        sol_vel_col = f'sol_{joint}_vel'
        sol_tau_col = f'sol_{joint}_tau'

        has_solution = sol_pos_col in wbc_df.columns
        has_js       = js_df is not None and f'{joint}_pos' in js_df.columns
        has_tau      = tau_df is not None and f'{joint}_tau' in tau_df.columns

        if not has_solution and not has_js and not has_tau:
            print(f'  [skip] no solution/feedback data for {joint}')
            continue

        # --- comparison/{joint}.png ---
        fig, axes = plt.subplots(3, 1, figsize=(13, 11), sharex=False)
        fig.suptitle(f'WBC Solution vs Feedback — {joint}', fontsize=13)

        ax = axes[0]
        if has_solution:
            ax.plot(t_wbc, wbc_df[sol_pos_col].values, '--', color=CB[6],
                    label='WBC desired', linewidth=1.5)
        if has_js:
            ax.plot(t_js, js_df[f'{joint}_pos'].values, '-', color=CB[5],
                    label='actual', linewidth=1.2)
        ax.set_ylabel('Position [rad]'); ax.set_title('Joint Position'); ax.legend()

        ax = axes[1]
        if has_solution:
            ax.plot(t_wbc, wbc_df[sol_vel_col].values, '--', color=CB[6],
                    label='WBC desired', linewidth=1.5)
        if has_js:
            ax.plot(t_js, js_df[f'{joint}_vel'].values, '-', color=CB[5],
                    label='actual', linewidth=1.2)
        ax.set_ylabel('Velocity [rad/s]'); ax.set_title('Joint Velocity'); ax.legend()

        ax = axes[2]
        if has_solution:
            ax.plot(t_wbc, wbc_df[sol_tau_col].values, '-', color=CB[3],
                    label='WBC tau', linewidth=1.5)
        if has_tau:
            ax.plot(t_tau, tau_df[f'{joint}_tau'].values, '--', color=CB[1],
                    label='commanded (PD+FF)', linewidth=1.2)
        if has_js and f'{joint}_effort' in js_df.columns:
            ax.plot(t_js, js_df[f'{joint}_effort'].values, ':', color=CB[7],
                    label='measured effort', linewidth=1.0, alpha=0.8)
        ax.set_xlabel('Time [s]'); ax.set_ylabel('Torque [Nm]')
        ax.set_title('Joint Torque'); ax.legend()

        fig.tight_layout()
        save(fig, 'wbc', 'wbc', 'wbc_solution', 'comparison', f'{joint}.png')

        # --- error/{joint}_error.png ---
        if has_solution and has_js:
            pos_err = wbc_df[sol_pos_col].values - np.interp(
                t_wbc, t_js, js_df[f'{joint}_pos'].values)
            vel_err = wbc_df[sol_vel_col].values - np.interp(
                t_wbc, t_js, js_df[f'{joint}_vel'].values)

            fig, (ax_p, ax_v) = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
            fig.suptitle(f'WBC Solution Error — {joint}', fontsize=13)

            ax_p.plot(t_wbc, pos_err, color=CB[6])
            ax_p.axhline(0, linestyle='--', color=CB[0], alpha=0.4)
            ax_p.set_ylabel('err [rad]'); ax_p.set_title('Position error (WBC desired − actual)')

            ax_v.plot(t_wbc, vel_err, color=CB[3])
            ax_v.axhline(0, linestyle='--', color=CB[0], alpha=0.4)
            ax_v.set_ylabel('err [rad/s]'); ax_v.set_title('Velocity error (WBC desired − actual)')
            ax_v.set_xlabel('Time [s]')

            fig.tight_layout()
            save(fig, 'wbc', 'wbc', 'wbc_solution', 'error', f'{joint}_error.png')

    # --- all_torques overview ---
    sol_tau_cols = [c for c in wbc_df.columns if c.startswith('sol_') and c.endswith('_tau')]
    if sol_tau_cols:
        fig, ax = plt.subplots(figsize=(13, 6))
        ax.set_title('WBC Output Torques — All Joints')
        for i, col in enumerate(sol_tau_cols):
            name = col.replace('sol_', '').replace('_tau', '')
            ax.plot(t_wbc, wbc_df[col].values, color=CB[i % len(CB)], label=name)
        ax.set_xlabel('Time [s]'); ax.set_ylabel('Torque [Nm]')
        ax.legend(ncol=2, fontsize=8)
        fig.tight_layout()
        save(fig, 'wbc', 'wbc', 'wbc_solution', 'all_torques.png')


def plot_wbc(log_dir: Path, joint_names: list) -> None:
    wbc_df = _load_wbc(log_dir)
    if wbc_df is None or wbc_df.empty:
        print('  [skip] wbc_log.txt has no data rows yet')
        return
    js_df  = _load_joint_state(log_dir)
    tau_df = _load_tau_commanded(log_dir)

    plot_wbc_com(wbc_df)
    plot_wbc_wheels(wbc_df)
    plot_wbc_torso(wbc_df)
    plot_wbc_solution(wbc_df, js_df, tau_df, joint_names)


# ---------------------------------------------------------------------------
# timings  (timing_log.txt)
# ---------------------------------------------------------------------------

def plot_timings(log_dir: Path) -> None:
    log = log_dir / 'timing_log.txt'
    if not log.exists():
        warn_missing(log)
        return

    df = pd.read_csv(log)
    if df.empty:
        print('  [skip] no timing data')
        return

    budget_us = 2000  # 500 Hz → 2 ms budget
    cycles = np.arange(len(df))

    # Columns to plot individually (skip columns that are all zero)
    candidates = {
        'time_mpc_us':    ('MPC',    CB[5]),
        'time_wbc_us':    ('WBC',    CB[3]),
        'time_filter_us': ('Filter', CB[2]),
    }
    individual = {col: meta for col, meta in candidates.items()
                  if col in df.columns and df[col].any()}

    # --- One figure per individual timing ---
    for col, (lbl, c) in individual.items():
        data = df[col].values
        avg  = data.mean()
        fig, (ax_t, ax_h) = plt.subplots(2, 1, figsize=(13, 8),
                                          gridspec_kw={'height_ratios': [2, 1]})
        fig.suptitle(f'{lbl} Timing', fontsize=13)

        ax_t.plot(cycles, data, color=c, lw=0.8)
        ax_t.axhline(avg, linestyle='--', color=CB[0], alpha=0.7,
                     label=f'avg = {avg:.1f} µs')
        ax_t.set_ylabel('Time [µs]'); ax_t.set_xlabel('Control cycle')
        ax_t.set_title(f'{lbl} execution time per cycle'); ax_t.legend()

        ax_h.hist(data, bins=60, color=c, alpha=0.8)
        ax_h.axvline(avg, linestyle='--', color=CB[0], alpha=0.7,
                     label=f'avg = {avg:.1f} µs')
        ax_h.set_xlabel('Time [µs]'); ax_h.set_ylabel('Count')
        ax_h.set_title('Distribution'); ax_h.legend()

        fig.tight_layout()
        save(fig, 'timings', 'timings', f'{lbl.lower()}_timing.png')

    # --- Total timing figure ---
    if 'total_time_us' not in df.columns:
        return

    total = df['total_time_us'].values
    avg_total = total.mean()
    pct_over  = 100.0 * (total > budget_us).mean()

    fig, (ax_t, ax_h) = plt.subplots(2, 1, figsize=(13, 8),
                                      gridspec_kw={'height_ratios': [2, 1]})
    fig.suptitle('Total Cycle Time', fontsize=13)

    # stacked area: show contribution of each component
    bottom = np.zeros(len(df))
    for col, (lbl, c) in individual.items():
        ax_t.fill_between(cycles, bottom, bottom + df[col].values,
                          color=c, alpha=0.6, label=lbl)
        bottom += df[col].values
    ax_t.plot(cycles, total, color=CB[0], lw=0.8, label='total')
    ax_t.axhline(budget_us, linestyle='--', color=CB[6],
                 label=f'budget {budget_us} µs')
    ax_t.axhline(avg_total, linestyle=':', color=CB[0], alpha=0.7,
                 label=f'avg {avg_total:.1f} µs')
    ax_t.set_ylabel('Time [µs]'); ax_t.set_xlabel('Control cycle')
    ax_t.set_title('Per-cycle breakdown'); ax_t.legend(ncol=3, fontsize=8)

    ax_h.hist(total, bins=60, color=CB[6], alpha=0.8)
    ax_h.axvline(budget_us, linestyle='--', color=CB[0],
                 label=f'budget {budget_us} µs  ({pct_over:.1f}% over)')
    ax_h.axvline(avg_total, linestyle=':', color=CB[0], alpha=0.7,
                 label=f'avg {avg_total:.1f} µs')
    ax_h.set_xlabel('Total time [µs]'); ax_h.set_ylabel('Count')
    ax_h.set_title('Distribution of total cycle time'); ax_h.legend()

    fig.tight_layout()
    save(fig, 'timings', 'timings', 'total_timing.png')


# ---------------------------------------------------------------------------
# feedback — raw sensor data only, no comparison
# ---------------------------------------------------------------------------

def plot_feedback_imu(log_dir: Path) -> None:
    df = _load_imu(log_dir)
    if df is None:
        return

    t = df['time_s'].values - df['time_s'].iloc[0]

    # --- Angular velocity ---
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle('IMU — Angular Velocity', fontsize=13)
    for ax, axis, c in zip(axes, ['x', 'y', 'z'], [CB[5], CB[3], CB[6]]):
        data = df[f'w{axis}'].values
        avg  = float(np.mean(data))
        ax.plot(t, data, color=c)
        ax.axhline(avg, linestyle='--', color=CB[0], alpha=0.6, label=f'avg={avg:.4f}')
        ax.set_ylabel(f'ω{axis} [rad/s]'); ax.legend()
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'feedback', 'feedback', 'imu', 'angular_velocity.png')

    # --- Linear acceleration ---
    fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    fig.suptitle('IMU — Linear Acceleration', fontsize=13)
    for ax, axis, c in zip(axes, ['x', 'y', 'z'], [CB[2], CB[1], CB[7]]):
        data = df[f'a{axis}'].values
        avg  = float(np.mean(data))
        ax.plot(t, data, color=c)
        ax.axhline(avg, linestyle='--', color=CB[0], alpha=0.6, label=f'avg={avg:.4f}')
        ax.set_ylabel(f'a{axis} [m/s²]'); ax.legend()
    axes[-1].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'feedback', 'feedback', 'imu', 'linear_acceleration.png')

    # --- Orientation (RPY from quaternion) ---
    if all(c in df.columns for c in ['qx', 'qy', 'qz', 'qw']):
        q = _fix_quat_continuity(df[['qx', 'qy', 'qz', 'qw']].values)
        roll, pitch, yaw = _quat_to_rpy(q[:, 0], q[:, 1], q[:, 2], q[:, 3])
        fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
        fig.suptitle('IMU — Orientation (RPY)', fontsize=13)
        for ax, d, lbl, c in zip(
            axes,
            [np.unwrap(roll), np.unwrap(pitch), np.unwrap(yaw)],
            ['Roll', 'Pitch', 'Yaw'],
            [CB[1], CB[2], CB[5]],
        ):
            ax.plot(t, d, color=c)
            ax.set_ylabel(f'{lbl} [rad]')
        axes[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'feedback', 'feedback', 'imu', 'orientation.png')


def plot_feedback_joints(log_dir: Path, joint_names: list) -> None:
    df = _load_joint_state(log_dir)
    if df is None:
        return

    t = df['time_stamp_s'].values - df['time_stamp_s'].iloc[0]

    for joint in joint_names:
        pos_col = f'{joint}_pos'
        vel_col = f'{joint}_vel'
        eff_col = f'{joint}_effort'

        if pos_col not in df.columns:
            print(f'  [skip] no feedback data for {joint}')
            continue

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(f'Feedback — {joint}', fontsize=13)

        conf = [
            (pos_col, 'Position [rad]',   'Position',  CB[5]),
            (vel_col, 'Velocity [rad/s]', 'Velocity',  CB[3]),
            (eff_col, 'Effort [Nm]',      'Effort',    CB[6]),
        ]
        for ax, (col, ylabel, title, c) in zip(axes, conf):
            if col in df.columns:
                data = df[col].values
                avg  = float(np.mean(data))
                ax.plot(t, data, color=c)
                ax.axhline(avg, linestyle='--', color=CB[0], alpha=0.55,
                           label=f'avg={avg:.3f}')
            ax.set_ylabel(ylabel); ax.set_title(title); ax.legend()

        axes[-1].set_xlabel('Time [s]')
        fig.tight_layout()
        save(fig, 'feedback', 'feedback', 'joints', f'{joint}.png')


# ---------------------------------------------------------------------------
# Legacy groups
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
    save(fig, 'encoder', 'encoder', 'wheel_data.png')


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
    save(fig, 'odom', 'odom', 'simple_odom.png')


def plot_robot_odometry(log_dir: Path) -> None:
    csv = log_dir / 'robot_odom.csv'
    if not csv.exists():
        warn_missing(csv)
        return

    df = pd.read_csv(csv); df.columns = df.columns.str.strip()
    t = df['t'].values

    fig, axes = plt.subplots(3, 2, figsize=(16, 12), sharex=True)
    fig.suptitle('Robot Odometry (CSV)', fontsize=13)

    def _panel(ax, x, y, z, title, ylabel):
        _xyz_lines(ax, t, x, y, z)
        ax.set_title(title); ax.set_ylabel(ylabel)

    _panel(axes[0, 0], df['p_odom_x'].values, df['p_odom_y'].values, df['p_odom_z'].values,
           'Base Position', 'Position [m]')
    _panel(axes[0, 1], df['v_odom_x'].values, df['v_odom_y'].values, df['v_odom_z'].values,
           'Base Velocity', 'Velocity [m/s]')
    _panel(axes[1, 0], df['p_cL_odom_x'].values, df['p_cL_odom_y'].values, df['p_cL_odom_z'].values,
           'Left Contact (Odom)', 'Position [m]')
    _panel(axes[1, 1], df['p_cR_odom_x'].values, df['p_cR_odom_y'].values, df['p_cR_odom_z'].values,
           'Right Contact (Odom)', 'Position [m]')

    px = df['p_odom_x'].values; py = df['p_odom_y'].values; pz = df['p_odom_z'].values
    vx_d = np.gradient(px, t); vy_d = np.gradient(py, t); vz_d = np.gradient(pz, t)
    for c, d, lbl in zip([CB[5], CB[3], CB[6]], [vx_d, vy_d, vz_d], ['vx', 'vy', 'vz']):
        axes[2, 0].plot(t, d, '--', color=c, label=lbl)
    axes[2, 0].set_title('Velocity (Differentiated)'); axes[2, 0].set_ylabel('Vel [m/s]'); axes[2, 0].legend()

    if 'w_l_odom' in df.columns:
        axes[2, 1].plot(t, df['w_l_odom'].values, color=CB[5], label='left')
        axes[2, 1].plot(t, df['w_r_odom'].values, color=CB[6], label='right')
        axes[2, 1].set_title('Wheel Angular Velocity'); axes[2, 1].set_ylabel('[rad/s]'); axes[2, 1].legend()

    for col in range(2):
        axes[2, col].set_xlabel('Time [s]')
    fig.tight_layout()
    save(fig, 'odom', 'odom', 'robot_odom.png')


def plot_plan(log_dir: Path) -> None:
    x_path = log_dir / 'plan' / 'x.txt'
    if not x_path.exists():
        warn_missing(x_path)
        return

    x = np.loadtxt(x_path, ndmin=2)
    DT_MS = 2; VEC_OFF_Y = 0.567 / 2
    pcom_x, pcom_y, pcom_z = x[:, 0], x[:, 1], x[:, 2]
    c_x,    c_y,    c_z    = x[:, 6], x[:, 7], x[:, 8]
    theta = x[:, 10]
    off_x = -np.sin(theta) * VEC_OFF_Y; off_y = np.cos(theta) * VEC_OFF_Y
    cL_x, cL_y = c_x + off_x, c_y + off_y
    cR_x, cR_y = c_x - off_x, c_y - off_y
    t_ms = np.arange(x.shape[0]) * DT_MS

    fig, axes = plt.subplots(1, 2, figsize=(15, 6), layout='constrained')
    ax0, ax1 = axes
    ax0.plot(pcom_x, pcom_y, color=CB[5], lw=2, label='CoM')
    ax0.plot(c_x, c_y, '--', color=CB[1], lw=2, label='c')
    ax0.plot(cL_x, cL_y, '-.', color=CB[3], lw=1.5, label='cL')
    ax0.plot(cR_x, cR_y, ':',  color=CB[6], lw=1.5, label='cR')
    ax0.plot(pcom_x[0], pcom_y[0], 'o', color=CB[0], ms=8)
    ax0.plot(pcom_x[-1], pcom_y[-1], 'x', color=CB[7], ms=10, mew=2)
    ax0.set_xlabel('x [m]'); ax0.set_ylabel('y [m]')
    ax0.set_title('X–Y Reference Plan'); ax0.legend()
    ax1.plot(t_ms, pcom_z, color=CB[5], lw=2, label='CoM z')
    ax1.plot(t_ms, c_z, '--', color=CB[1], lw=2, label='c z')
    ax1.set_xlabel('t [ms]'); ax1.set_ylabel('z [m]')
    ax1.set_title('Z Reference Plan'); ax1.legend()
    save(fig, 'plan', 'plan', 'reference_plan.png')

    jump_path = log_dir / 'plan' / 'jump_traj.txt'
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
# CLI entry point
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            'Generate TITA controller plots.\n'
            'Default: filter, wbc, timings, feedback groups.\n'
            'Pass --show-<group> to open interactively; --only to restrict groups.\n'
            'Groups: filter wbc timings feedback encoder odom plan'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument('--log-dir', type=Path, default=ROBOT_LOGS,
                        help=f'robot_logs/ directory  [default: {ROBOT_LOGS}]')
    parser.add_argument('--joints', nargs='+', default=JOINT_NAMES,
                        help='Joint names for per-joint plots')
    parser.add_argument('--only', nargs='+', default=None, metavar='GROUP',
                        help='Run only the listed groups')

    show_group = parser.add_argument_group('interactive display')
    for g in sorted(_ALL_GROUPS):
        show_group.add_argument(f'--show-{g}', action='store_true',
                                help=f'Show {g} plots interactively')

    args = parser.parse_args()

    log_dir = args.log_dir
    print(f'Log directory : {log_dir}')
    print(f'Output        : {OUT_DIR}')

    only = set(args.only) if args.only else (_SHOW if _SHOW else _DEFAULT_GROUPS)

    def run(group: str, fn, *a, **kw) -> None:
        if group not in only:
            return
        print(f'\n[{group}]')
        try:
            fn(*a, **kw)
        except Exception as exc:
            print(f'  ERROR in {fn.__name__}: {exc}')

    run('filter',   plot_filter,          log_dir)
    run('wbc',      plot_wbc,             log_dir, args.joints)
    run('timings',  plot_timings,         log_dir)
    run('feedback', plot_feedback_imu,    log_dir)
    run('feedback', plot_feedback_joints, log_dir, args.joints)
    run('encoder',  plot_encoder_wheels,  log_dir)
    run('odom',     plot_odom,            log_dir)
    run('odom',     plot_robot_odometry,  log_dir)
    run('plan',     plot_plan,            log_dir)

    if _SHOW:
        shown = ', '.join(sorted(_SHOW))
        print(f'\nOpening interactive windows for: {shown}')
        plt.show()
    else:
        print(f'\nImages saved under  {OUT_DIR.relative_to(OUT_DIR.parent)}/')


if __name__ == '__main__':
    main()