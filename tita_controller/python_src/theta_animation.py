#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'

X_FILE    = 'x.txt'
U_FILE    = 'u.txt'
THETA_COL = 10
V_COL     = 11
OMEGA_COL = 12
ALPHA_COL = 2
DT_STEP   = 10.0


def parse_timestep(name: str):
    try:
        return float(name)
    except ValueError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description='Animate MPC theta/v/omega/alpha predictions')
    parser.add_argument('--base-dir', type=Path, default=ROBOT_LOGS,
                        help=f'Parent of robot_logs_N folders  [default: {ROBOT_LOGS}]')
    parser.add_argument('--exp', type=int, default=None, metavar='N',
                        help='Experiment number (skip interactive prompt)')
    args = parser.parse_args()

    log_dir, _out_dir = select_experiment(args.base_dir, args.exp)
    base_dir = log_dir / 'mpc_data'

    folders = sorted(
        [(parse_timestep(p.name), p) for p in base_dir.iterdir()
         if p.is_dir() and parse_timestep(p.name) is not None],
        key=lambda tp: tp[0]
    ) if base_dir.exists() else []

    if not folders:
        print(f'  [skip] No numeric timestep folders found in {base_dir}')
        return

    times = []
    theta_hist, v_hist, omega_hist, u0_alpha_list = [], [], [], []
    pred_x_t_list = []
    pred_theta_list, pred_v_list, pred_omega_list = [], [], []
    pred_u_t_list = []
    pred_alpha_list = []
    t_max_pred = -np.inf
    empty = np.array([])

    for t, p in folders:
        x_path = p / X_FILE; u_path = p / U_FILE
        if not x_path.exists() or not u_path.exists():
            continue
        x_data = np.loadtxt(x_path, ndmin=2)
        u_data = np.loadtxt(u_path, ndmin=2)
        if x_data.shape[0] == 0 or u_data.shape[0] == 0:
            continue

        x0 = x_data[0]
        times.append(t)
        theta_hist.append(x0[THETA_COL])
        v_hist.append(x0[V_COL])
        omega_hist.append(x0[OMEGA_COL])
        u0_alpha_list.append(u_data[0, ALPHA_COL])

        if x_data.shape[0] >= 2:
            Lx = x_data.shape[0] - 1
            pred_xt = t + np.arange(1, Lx + 1, dtype=float) * DT_STEP
            pred_x_t_list.append(pred_xt)
            pred_theta_list.append(x_data[1:, THETA_COL])
            pred_v_list.append(x_data[1:, V_COL])
            pred_omega_list.append(x_data[1:, OMEGA_COL])
            if pred_xt.size: t_max_pred = max(t_max_pred, pred_xt[-1])
        else:
            pred_x_t_list.append(empty)
            for lst in (pred_theta_list, pred_v_list, pred_omega_list):
                lst.append(empty)

        Lu = u_data.shape[0]
        pred_ut = t + np.arange(0, Lu, dtype=float) * DT_STEP
        pred_u_t_list.append(pred_ut)
        pred_alpha_list.append(u_data[:, ALPHA_COL])
        if pred_ut.size: t_max_pred = max(t_max_pred, pred_ut[-1])

    times       = np.asarray(times, dtype=float)
    theta_hist  = np.asarray(theta_hist, dtype=float)
    v_hist      = np.asarray(v_hist, dtype=float)
    omega_hist  = np.asarray(omega_hist, dtype=float)
    u0_alpha_list = np.asarray(u0_alpha_list, dtype=float)

    if times.size == 0:
        print('  [skip] Found folders but no readable data')
        return

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(2, 2)
    ax_theta = fig.add_subplot(gs[0, 0]); ax_v     = fig.add_subplot(gs[0, 1])
    ax_omega = fig.add_subplot(gs[1, 0]); ax_alpha = fig.add_subplot(gs[1, 1])

    def make_lines(ax, ylabel, xlabel=''):
        (hist,) = ax.plot([], [], lw=2, label='hist')
        (pred,) = ax.plot([], [], lw=2, label='pred')
        (pt,)   = ax.plot([], [], 'o', ls='')
        ax.set_ylabel(ylabel)
        if xlabel: ax.set_xlabel(xlabel)
        ax.grid(True); ax.legend()
        return hist, pred, pt

    ltheta, ptheta, pttheta = make_lines(ax_theta, 'theta [rad]')
    lv,     pv,     ptv     = make_lines(ax_v,     'v [m/s]')
    lomega, pomega, ptomega = make_lines(ax_omega, 'omega [rad/s]', 't [ms]')
    lalpha, palpha, ptalpha = make_lines(ax_alpha, 'alpha [rad/s²]', 't [ms]')

    def set_limits(ax, t_hist, *value_lists):
        t_min = np.min(t_hist)
        t_max = max(np.max(t_hist), t_max_pred if t_max_pred > -np.inf else np.max(t_hist))
        ax.set_xlim(t_min, t_max)
        V = []
        for v in value_lists:
            for arr in (v if isinstance(v, (list, tuple)) else [v]):
                arr = np.asarray(arr)
                if arr.size > 0 and np.any(np.isfinite(arr)):
                    V.append(arr[np.isfinite(arr)])
        if not V: return
        V = np.concatenate(V)
        vmin, vmax = np.min(V), np.max(V)
        dv = 1e-3 if abs(vmax - vmin) < 1e-12 else 0.2 * (vmax - vmin)
        ax.set_ylim(vmin - dv, vmax + dv)

    set_limits(ax_theta, times, theta_hist, pred_theta_list)
    set_limits(ax_v,     times, v_hist,     pred_v_list)
    set_limits(ax_omega, times, omega_hist, pred_omega_list)
    set_limits(ax_alpha, times, u0_alpha_list, pred_alpha_list)

    all_artists = (ltheta, ptheta, pttheta,
                   lv,     pv,     ptv,
                   lomega, pomega, ptomega,
                   lalpha, palpha, ptalpha)

    def init():
        for a in all_artists: a.set_data([], [])
        return all_artists

    def update(i):
        ts = times[:i+1]
        ltheta.set_data(ts, theta_hist[:i+1]); pttheta.set_data([ts[-1]], [theta_hist[i]])
        lv.set_data(ts, v_hist[:i+1]);          ptv.set_data([ts[-1]], [v_hist[i]])
        lomega.set_data(ts, omega_hist[:i+1]);   ptomega.set_data([ts[-1]], [omega_hist[i]])

        if i > 0:
            lalpha.set_data(times[:i], u0_alpha_list[:i])
        else:
            lalpha.set_data([], [])

        pxt = pred_x_t_list[i]
        if pxt.size > 0:
            ptheta.set_data(pxt, pred_theta_list[i])
            pv.set_data(pxt, pred_v_list[i])
            pomega.set_data(pxt, pred_omega_list[i])
        else:
            for a in (ptheta, pv, pomega): a.set_data([], [])

        put = pred_u_t_list[i]
        if put.size > 0:
            palpha.set_data(put, pred_alpha_list[i])
        else:
            palpha.set_data([], [])

        return all_artists

    ani = FuncAnimation(fig, update, frames=len(times), init_func=init,
                        interval=1, blit=True, repeat=False)

    anim_running = [True]
    def toggle(event):
        if event.key == ' ':
            if anim_running[0]: ani.event_source.stop()
            else:               ani.event_source.start()
            anim_running[0] = not anim_running[0]
    fig.canvas.mpl_connect('key_press_event', toggle)
    plt.tight_layout()
    plt.show()


if __name__ == '__main__':
    main()