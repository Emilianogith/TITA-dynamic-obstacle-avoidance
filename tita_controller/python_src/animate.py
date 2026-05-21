#!/usr/bin/env python3
import argparse
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

from _exp import select_experiment

SCRIPT_DIR = Path(__file__).resolve().parent
ROBOT_LOGS = Path.cwd() / 'robot_logs'

X_FILE = 'x.txt'
U_FILE = 'u.txt'

COM_X_COL, COM_Y_COL, COM_Z_COL = 0, 1, 2
PC_X_COL,  PC_Y_COL,  PC_Z_COL  = 6, 7, 8
ACC_X_COL = 0
FLZ_COL   = 5
FRZ_COL   = 8
DT_MS     = 10.0


def parse_timestep(name: str):
    try:
        return float(name)
    except ValueError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description='Animate MPC state + control predictions')
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
    com_x, com_y, com_z = [], [], []
    pc_x,  pc_y,  pc_z  = [], [], []
    u0_accx_list, u0_flz_list, u0_frz_list = [], [], []
    pred_x_t_list = []
    pred_com_x_list, pred_com_y_list, pred_com_z_list = [], [], []
    pred_pc_x_list,  pred_pc_y_list,  pred_pc_z_list  = [], [], []
    pred_u_t_list = []
    pred_accx_list, pred_flz_list, pred_frz_list = [], [], []
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
        com_x.append(x0[COM_X_COL]); com_y.append(x0[COM_Y_COL]); com_z.append(x0[COM_Z_COL])
        pc_x.append(x0[PC_X_COL]);   pc_y.append(x0[PC_Y_COL]);   pc_z.append(x0[PC_Z_COL])
        u0_accx_list.append(u_data[0, ACC_X_COL])
        u0_flz_list.append(u_data[0, FLZ_COL])
        u0_frz_list.append(u_data[0, FRZ_COL])

        if x_data.shape[0] >= 2:
            Lx = x_data.shape[0] - 1
            pred_xt = t + np.arange(1, Lx + 1, dtype=float) * DT_MS
            pred_x_t_list.append(pred_xt)
            pred_com_x_list.append(x_data[1:, COM_X_COL]); pred_com_y_list.append(x_data[1:, COM_Y_COL])
            pred_com_z_list.append(x_data[1:, COM_Z_COL])
            pred_pc_x_list.append(x_data[1:, PC_X_COL]);   pred_pc_y_list.append(x_data[1:, PC_Y_COL])
            pred_pc_z_list.append(x_data[1:, PC_Z_COL])
            if pred_xt.size: t_max_pred = max(t_max_pred, pred_xt[-1])
        else:
            pred_x_t_list.append(empty)
            for lst in (pred_com_x_list, pred_com_y_list, pred_com_z_list,
                        pred_pc_x_list, pred_pc_y_list, pred_pc_z_list):
                lst.append(empty)

        Lu = u_data.shape[0]
        pred_ut = t + np.arange(0, Lu, dtype=float) * DT_MS
        pred_u_t_list.append(pred_ut)
        pred_accx_list.append(u_data[:, ACC_X_COL])
        pred_flz_list.append(u_data[:, FLZ_COL])
        pred_frz_list.append(u_data[:, FRZ_COL])
        if pred_ut.size: t_max_pred = max(t_max_pred, pred_ut[-1])

    times = np.asarray(times, dtype=float)
    com_x = np.asarray(com_x); com_y = np.asarray(com_y); com_z = np.asarray(com_z)
    pc_x  = np.asarray(pc_x);  pc_y  = np.asarray(pc_y);  pc_z  = np.asarray(pc_z)
    u0_accx_list = np.asarray(u0_accx_list)
    u0_flz_list  = np.asarray(u0_flz_list)
    u0_frz_list  = np.asarray(u0_frz_list)

    if times.size == 0:
        print('  [skip] Found folders but no readable data')
        return

    fig = plt.figure(figsize=(13, 7))
    gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1], width_ratios=[1, 1])
    ax_x  = fig.add_subplot(gs[0, 0]); ax_y  = fig.add_subplot(gs[1, 0])
    ax_z  = fig.add_subplot(gs[2, 0]); ax_ax = fig.add_subplot(gs[0, 1])
    ax_flz= fig.add_subplot(gs[1, 1]); ax_frz= fig.add_subplot(gs[2, 1])

    def make_lines(ax, ylabel, xlabel=''):
        (hist,) = ax.plot([], [], lw=2, label='CoM (hist)', color='tab:blue')
        (pc_h,) = ax.plot([], [], lw=1.5, label='Pc (hist)', color='tab:orange')
        (pred,) = ax.plot([], [], lw=2, label='CoM (pred)', color='red')
        (ppc,)  = ax.plot([], [], lw=1.5, ls='--', label='Pc (pred)', color='red')
        (pt,)   = ax.plot([], [], 'o', ls='', color='tab:green')
        ax.set_ylabel(ylabel)
        if xlabel: ax.set_xlabel(xlabel)
        ax.grid(True); ax.legend(fontsize=8)
        return hist, pc_h, pred, ppc, pt

    lx, pc_lx, px, ppc_x, ptx = make_lines(ax_x, 'x [m]')
    ly, pc_ly, py, ppc_y, pty = make_lines(ax_y, 'y [m]')
    lz, pc_lz, pz, ppc_z, ptz = make_lines(ax_z, 'z [m]', 't [ms]')

    def make_u_lines(ax, ylabel, xlabel=''):
        (hist,) = ax.plot([], [], lw=2, label='applied hist')
        (pred,) = ax.plot([], [], lw=2, label='pred', color='red')
        (pt,)   = ax.plot([], [], 'o', ls='')
        ax.set_ylabel(ylabel)
        if xlabel: ax.set_xlabel(xlabel)
        ax.grid(True); ax.legend(fontsize=8)
        return hist, pred, pt

    lax, pax, ptax = make_u_lines(ax_ax,  'a [m/s²]')
    lflz, pflz, ptflz = make_u_lines(ax_flz, 'flz [N]')
    lfrz, pfrz, ptfrz = make_u_lines(ax_frz, 'frz [N]', 't [ms]')

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
        dv = 1e-3 if abs(vmax - vmin) < 1e-12 else 0.20 * (vmax - vmin)
        ax.set_ylim(vmin - dv, vmax + dv)

    set_limits(ax_x,  times, com_x, pc_x, pred_com_x_list, pred_pc_x_list)
    set_limits(ax_y,  times, com_y, pc_y, pred_com_y_list, pred_pc_y_list)
    set_limits(ax_z,  times, com_z, pc_z, pred_com_z_list, pred_pc_z_list)
    set_limits(ax_ax,  times, u0_accx_list, pred_accx_list)
    set_limits(ax_flz, times, u0_flz_list,  pred_flz_list)
    set_limits(ax_frz, times, u0_frz_list,  pred_frz_list)

    all_artists = (lx, pc_lx, px, ppc_x, ptx,
                   ly, pc_ly, py, ppc_y, pty,
                   lz, pc_lz, pz, ppc_z, ptz,
                   lax, pax, ptax, lflz, pflz, ptflz, lfrz, pfrz, ptfrz)

    def init():
        for a in all_artists: a.set_data([], [])
        return all_artists

    def update(i):
        ts = times[:i+1]
        lx.set_data(ts, com_x[:i+1]); pc_lx.set_data(ts, pc_x[:i+1]); ptx.set_data([ts[-1]], [com_x[i]])
        ly.set_data(ts, com_y[:i+1]); pc_ly.set_data(ts, pc_y[:i+1]); pty.set_data([ts[-1]], [com_y[i]])
        lz.set_data(ts, com_z[:i+1]); pc_lz.set_data(ts, pc_z[:i+1]); ptz.set_data([ts[-1]], [com_z[i]])

        if i > 0:
            ts_u = times[:i]
            lax.set_data(ts_u, u0_accx_list[:i])
            lflz.set_data(ts_u, u0_flz_list[:i])
            lfrz.set_data(ts_u, u0_frz_list[:i])
        else:
            lax.set_data([], []); lflz.set_data([], []); lfrz.set_data([], [])

        pxt = pred_x_t_list[i]
        if pxt.size > 0:
            px.set_data(pxt, pred_com_x_list[i]); ppc_x.set_data(pxt, pred_pc_x_list[i])
            py.set_data(pxt, pred_com_y_list[i]); ppc_y.set_data(pxt, pred_pc_y_list[i])
            pz.set_data(pxt, pred_com_z_list[i]); ppc_z.set_data(pxt, pred_pc_z_list[i])
        else:
            for a in (px, ppc_x, py, ppc_y, pz, ppc_z): a.set_data([], [])

        put = pred_u_t_list[i]
        if put.size > 0:
            pax.set_data(put, pred_accx_list[i])
            pflz.set_data(put, pred_flz_list[i])
            pfrz.set_data(put, pred_frz_list[i])
        else:
            for a in (pax, pflz, pfrz): a.set_data([], [])

        return all_artists

    ani = FuncAnimation(fig, update, frames=len(times), init_func=init,
                        interval=100, blit=True, repeat=False)

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