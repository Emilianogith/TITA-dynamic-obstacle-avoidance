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
PCOM_X_COL, PCOM_Y_COL = 0, 1
C_X_COL,    C_Y_COL    = 6, 7
THETA_COL   = 10
VEC_OFF_Y   = 0.567 / 2


def parse_timestep(name: str):
    try:
        return float(name)
    except ValueError:
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description='Animate MPC plan in x-y plane')
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
    pcom_x_hist, pcom_y_hist = [], []
    c_x_hist,    c_y_hist    = [], []
    cL_x_hist,   cL_y_hist   = [], []
    cR_x_hist,   cR_y_hist   = [], []
    pred_pcom_x_list, pred_pcom_y_list = [], []
    pred_c_x_list,    pred_c_y_list    = [], []
    pred_cL_x_list,   pred_cL_y_list   = [], []
    pred_cR_x_list,   pred_cR_y_list   = [], []

    empty = np.array([])

    for t, p in folders:
        x_path = p / X_FILE
        if not x_path.exists():
            for lst in (pred_pcom_x_list, pred_pcom_y_list, pred_c_x_list, pred_c_y_list,
                        pred_cL_x_list, pred_cL_y_list, pred_cR_x_list, pred_cR_y_list):
                lst.append(empty)
            continue

        x_data = np.loadtxt(x_path, ndmin=2)
        if x_data.shape[0] == 0:
            for lst in (pred_pcom_x_list, pred_pcom_y_list, pred_c_x_list, pred_c_y_list,
                        pred_cL_x_list, pred_cL_y_list, pred_cR_x_list, pred_cR_y_list):
                lst.append(empty)
            continue

        x0 = x_data[0]
        times.append(t)
        pcom_x_hist.append(x0[PCOM_X_COL]); pcom_y_hist.append(x0[PCOM_Y_COL])
        cx = x0[C_X_COL]; cy = x0[C_Y_COL]; theta0 = x0[THETA_COL]
        c_x_hist.append(cx); c_y_hist.append(cy)
        ox = -np.sin(theta0) * VEC_OFF_Y; oy = np.cos(theta0) * VEC_OFF_Y
        cL_x_hist.append(cx + ox); cL_y_hist.append(cy + oy)
        cR_x_hist.append(cx - ox); cR_y_hist.append(cy - oy)

        if x_data.shape[0] >= 2:
            pred_pcom_x = x_data[1:, PCOM_X_COL]; pred_pcom_y = x_data[1:, PCOM_Y_COL]
            pred_c_x    = x_data[1:, C_X_COL];    pred_c_y    = x_data[1:, C_Y_COL]
            pred_theta  = x_data[1:, THETA_COL]
            off_x = -np.sin(pred_theta) * VEC_OFF_Y; off_y = np.cos(pred_theta) * VEC_OFF_Y
            pred_pcom_x_list.append(pred_pcom_x); pred_pcom_y_list.append(pred_pcom_y)
            pred_c_x_list.append(pred_c_x);       pred_c_y_list.append(pred_c_y)
            pred_cL_x_list.append(pred_c_x + off_x); pred_cL_y_list.append(pred_c_y + off_y)
            pred_cR_x_list.append(pred_c_x - off_x); pred_cR_y_list.append(pred_c_y - off_y)
        else:
            for lst in (pred_pcom_x_list, pred_pcom_y_list, pred_c_x_list, pred_c_y_list,
                        pred_cL_x_list, pred_cL_y_list, pred_cR_x_list, pred_cR_y_list):
                lst.append(empty)

    times       = np.asarray(times)
    pcom_x_hist = np.asarray(pcom_x_hist); pcom_y_hist = np.asarray(pcom_y_hist)
    c_x_hist    = np.asarray(c_x_hist);    c_y_hist    = np.asarray(c_y_hist)
    cL_x_hist   = np.asarray(cL_x_hist);   cL_y_hist   = np.asarray(cL_y_hist)
    cR_x_hist   = np.asarray(cR_x_hist);   cR_y_hist   = np.asarray(cR_y_hist)

    if times.size == 0:
        print('  [skip] Found folders but no readable data')
        return

    fig, ax = plt.subplots(figsize=(8, 7), layout='constrained')
    (line_com_hist,)  = ax.plot([], [], '-',  lw=2,   label='CoM history')
    (line_c_hist,)    = ax.plot([], [], '--', lw=2,   label='c history')
    (line_cL_hist,)   = ax.plot([], [], '-.', lw=1.5, label='cL history')
    (line_cR_hist,)   = ax.plot([], [], ':',  lw=1.5, label='cR history')
    (line_com_pred,)  = ax.plot([], [], '-',  lw=1,   label='CoM prediction')
    (line_c_pred,)    = ax.plot([], [], '--', lw=1,   label='c prediction')
    (line_cL_pred,)   = ax.plot([], [], '-.', lw=1,   label='cL prediction')
    (line_cR_pred,)   = ax.plot([], [], ':',  lw=1,   label='cR prediction')
    (marker_c_curr,)  = ax.plot([], [], '^', ms=10, ls='', label='c current')
    (marker_com_curr,)= ax.plot([], [], 'x', ms=10, ls='', label='CoM current')
    (marker_cL_curr,) = ax.plot([], [], 'o', ms=7,  ls='', label='cL current')
    (marker_cR_curr,) = ax.plot([], [], 's', ms=7,  ls='', label='cR current')
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('MPC: trajectories of c, cL, cR and CoM in x–y plane')
    ax.grid(True); ax.legend()

    all_x = [pcom_x_hist, c_x_hist, cL_x_hist, cR_x_hist]
    all_y = [pcom_y_hist, c_y_hist, cL_y_hist, cR_y_hist]
    for lst in (pred_pcom_x_list, pred_c_x_list, pred_cL_x_list, pred_cR_x_list):
        for arr in lst:
            if arr.size > 0: all_x.append(arr)
    for lst in (pred_pcom_y_list, pred_c_y_list, pred_cL_y_list, pred_cR_y_list):
        for arr in lst:
            if arr.size > 0: all_y.append(arr)

    all_x = np.concatenate(all_x); all_y = np.concatenate(all_y)
    dx = (all_x.max() - all_x.min()) * 0.1 + 1e-6
    dy = (all_y.max() - all_y.min()) * 0.1 + 1e-6
    ax.set_xlim(all_x.min() - dx, all_x.max() + dx)
    ax.set_ylim(all_y.min() - dy, all_y.max() + dy)

    all_lines = (line_c_hist, line_com_hist, line_cL_hist, line_cR_hist,
                 line_c_pred, line_com_pred, line_cL_pred, line_cR_pred,
                 marker_c_curr, marker_com_curr, marker_cL_curr, marker_cR_curr)

    def init():
        for ln in all_lines:
            ln.set_data([], [])
        return all_lines

    def update(i):
        line_com_hist.set_data(pcom_x_hist[:i+1], pcom_y_hist[:i+1])
        line_c_hist.set_data(c_x_hist[:i+1], c_y_hist[:i+1])
        line_cL_hist.set_data(cL_x_hist[:i+1], cL_y_hist[:i+1])
        line_cR_hist.set_data(cR_x_hist[:i+1], cR_y_hist[:i+1])
        marker_com_curr.set_data([pcom_x_hist[i]], [pcom_y_hist[i]])
        marker_c_curr.set_data([c_x_hist[i]], [c_y_hist[i]])
        marker_cL_curr.set_data([cL_x_hist[i]], [cL_y_hist[i]])
        marker_cR_curr.set_data([cR_x_hist[i]], [cR_y_hist[i]])
        line_com_pred.set_data(pred_pcom_x_list[i], pred_pcom_y_list[i]) if pred_pcom_x_list[i].size > 0 else line_com_pred.set_data([], [])
        line_c_pred.set_data(pred_c_x_list[i], pred_c_y_list[i]) if pred_c_x_list[i].size > 0 else line_c_pred.set_data([], [])
        line_cL_pred.set_data(pred_cL_x_list[i], pred_cL_y_list[i]) if pred_cL_x_list[i].size > 0 else line_cL_pred.set_data([], [])
        line_cR_pred.set_data(pred_cR_x_list[i], pred_cR_y_list[i]) if pred_cR_x_list[i].size > 0 else line_cR_pred.set_data([], [])
        return all_lines

    ani = FuncAnimation(fig, update, frames=len(times), init_func=init,
                        interval=100, blit=True, repeat=False)

    anim_running = [True]
    def toggle(event):
        if event.key == ' ':
            if anim_running[0]:
                ani.event_source.stop()
            else:
                ani.event_source.start()
            anim_running[0] = not anim_running[0]
    fig.canvas.mpl_connect('key_press_event', toggle)
    plt.show()


if __name__ == '__main__':
    main()