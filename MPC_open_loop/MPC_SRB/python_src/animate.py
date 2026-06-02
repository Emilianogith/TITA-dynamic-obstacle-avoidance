import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

# ==== SETTINGS ====
BASE_DIR = Path("/tmp/mpc_data")
X_FILE = "x.txt"
U_FILE = "u.txt"

# Columns in x.txt
COM_X_COL, COM_Y_COL, COM_Z_COL = 0, 1, 2
PR_X_COL,  PR_Y_COL,  PR_Z_COL  = 12, 13, 14  # right foot position
PL_X_COL,  PL_Y_COL,  PL_Z_COL  = 17, 18, 19  # left foot position

# Columns in u.txt
ACC_R_COL = 0   # ar  (right foot tangential acc)
ACC_L_COL = 2   # al  (left  foot tangential acc)
FLZ_COL   = 6
FRZ_COL   = 9

DT_MS = 10.0

def parse_timestep(name: str):
    try:
        return float(name)
    except ValueError:
        return None

# ---- discover timestep folders ----
folders = []
for p in BASE_DIR.iterdir():
    if p.is_dir():
        t = parse_timestep(p.name)
        if t is not None:
            folders.append((t, p))
if not folders:
    raise RuntimeError(f"No numeric timestep folders found in {BASE_DIR}")
folders.sort(key=lambda tp: tp[0])

# ---- load executed state + prediction horizons ----
times = []
com_x, com_y, com_z = [], [], []
pr_x,  pr_y,  pr_z  = [], [], []
pl_x,  pl_y,  pl_z  = [], [], []

u0_accr_list = []
u0_accl_list = []
u0_flz_list  = []
u0_frz_list  = []

pred_x_t_list = []
pred_com_x_list, pred_com_y_list, pred_com_z_list = [], [], []
pred_pr_x_list,  pred_pr_y_list,  pred_pr_z_list  = [], [], []
pred_pl_x_list,  pred_pl_y_list,  pred_pl_z_list  = [], [], []

pred_u_t_list = []
pred_accr_list, pred_accl_list, pred_flz_list, pred_frz_list = [], [], [], []

t_max_pred = -np.inf

for t, p in folders:
    x_path = p / X_FILE
    u_path = p / U_FILE
    if not x_path.exists() or not u_path.exists():
        continue

    x_data = np.loadtxt(x_path, ndmin=2)
    u_data = np.loadtxt(u_path, ndmin=2)

    if x_data.shape[0] == 0 or u_data.shape[0] == 0:
        continue

    x0 = x_data[0]
    times.append(t)
    com_x.append(x0[COM_X_COL]); com_y.append(x0[COM_Y_COL]); com_z.append(x0[COM_Z_COL])
    pr_x.append(x0[PR_X_COL]);   pr_y.append(x0[PR_Y_COL]);   pr_z.append(x0[PR_Z_COL])
    pl_x.append(x0[PL_X_COL]);   pl_y.append(x0[PL_Y_COL]);   pl_z.append(x0[PL_Z_COL])

    u0_accr_list.append(u_data[0, ACC_R_COL])
    u0_accl_list.append(u_data[0, ACC_L_COL])
    u0_flz_list.append(u_data[0, FLZ_COL])
    u0_frz_list.append(u_data[0, FRZ_COL])

    # ---- state predictions: x1..xN ----
    if x_data.shape[0] >= 2:
        pred_com_x = x_data[1:, COM_X_COL]
        pred_com_y = x_data[1:, COM_Y_COL]
        pred_com_z = x_data[1:, COM_Z_COL]
        pred_pr_x  = x_data[1:, PR_X_COL]
        pred_pr_y  = x_data[1:, PR_Y_COL]
        pred_pr_z  = x_data[1:, PR_Z_COL]
        pred_pl_x  = x_data[1:, PL_X_COL]
        pred_pl_y  = x_data[1:, PL_Y_COL]
        pred_pl_z  = x_data[1:, PL_Z_COL]

        Lx = min(pred_com_x.size, pred_com_y.size, pred_com_z.size,
                 pred_pr_x.size,  pred_pr_y.size,  pred_pr_z.size,
                 pred_pl_x.size,  pred_pl_y.size,  pred_pl_z.size)

        pred_com_x = pred_com_x[:Lx]; pred_com_y = pred_com_y[:Lx]; pred_com_z = pred_com_z[:Lx]
        pred_pr_x  = pred_pr_x[:Lx];  pred_pr_y  = pred_pr_y[:Lx];  pred_pr_z  = pred_pr_z[:Lx]
        pred_pl_x  = pred_pl_x[:Lx];  pred_pl_y  = pred_pl_y[:Lx];  pred_pl_z  = pred_pl_z[:Lx]

        pred_xt = t + np.arange(1, Lx + 1, dtype=float) * DT_MS

        pred_x_t_list.append(pred_xt)
        pred_com_x_list.append(pred_com_x); pred_com_y_list.append(pred_com_y); pred_com_z_list.append(pred_com_z)
        pred_pr_x_list.append(pred_pr_x);   pred_pr_y_list.append(pred_pr_y);   pred_pr_z_list.append(pred_pr_z)
        pred_pl_x_list.append(pred_pl_x);   pred_pl_y_list.append(pred_pl_y);   pred_pl_z_list.append(pred_pl_z)

        if pred_xt.size:
            t_max_pred = max(t_max_pred, pred_xt[-1])
    else:
        for lst in (pred_x_t_list,
                    pred_com_x_list, pred_com_y_list, pred_com_z_list,
                    pred_pr_x_list,  pred_pr_y_list,  pred_pr_z_list,
                    pred_pl_x_list,  pred_pl_y_list,  pred_pl_z_list):
            lst.append(np.array([]))

    # ---- input predictions: u0..u_{N-1} ----
    Lu = u_data.shape[0]
    pred_ut = t + np.arange(0, Lu, dtype=float) * DT_MS

    pred_u_t_list.append(pred_ut)
    pred_accr_list.append(u_data[:Lu, ACC_R_COL])
    pred_accl_list.append(u_data[:Lu, ACC_L_COL])
    pred_flz_list.append(u_data[:Lu, FLZ_COL])
    pred_frz_list.append(u_data[:Lu, FRZ_COL])

    if pred_ut.size:
        t_max_pred = max(t_max_pred, pred_ut[-1])

# Convert to arrays
times = np.asarray(times, dtype=float)
com_x = np.asarray(com_x, dtype=float); com_y = np.asarray(com_y, dtype=float); com_z = np.asarray(com_z, dtype=float)
pr_x  = np.asarray(pr_x,  dtype=float); pr_y  = np.asarray(pr_y,  dtype=float); pr_z  = np.asarray(pr_z,  dtype=float)
pl_x  = np.asarray(pl_x,  dtype=float); pl_y  = np.asarray(pl_y,  dtype=float); pl_z  = np.asarray(pl_z,  dtype=float)

u0_accr_list = np.asarray(u0_accr_list, dtype=float)
u0_accl_list = np.asarray(u0_accl_list, dtype=float)
u0_flz_list  = np.asarray(u0_flz_list,  dtype=float)
u0_frz_list  = np.asarray(u0_frz_list,  dtype=float)

if times.size == 0:
    raise RuntimeError("Found folders, but no readable data.")

# ---- axis limits helper ----
def set_limits(ax, t_hist, *value_lists):
    t_min = np.min(t_hist)
    t_max_hist = np.max(t_hist)
    t_max = max(t_max_hist, t_max_pred if t_max_pred > -np.inf else t_max_hist)
    ax.set_xlim(t_min, t_max)

    V = []
    for v in value_lists:
        if isinstance(v, (list, tuple)):
            for arr in v:
                arr = np.asarray(arr)
                if arr.size > 0:
                    V.append(arr)
        else:
            arr = np.asarray(v)
            if arr.size > 0:
                V.append(arr)
    if not V:
        return
    V = np.concatenate(V)
    V = V[np.isfinite(V)]
    if V.size == 0:
        return
    vmin, vmax = np.min(V), np.max(V)
    dv = 0.20 * (vmax - vmin) if abs(vmax - vmin) > 1e-12 else 1e-3
    ax.set_ylim(vmin - dv, vmax + dv)

# ---- figure & axes: 3x2 grid ----
fig = plt.figure(figsize=(13, 7))
gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1], width_ratios=[1, 1])

ax_x   = fig.add_subplot(gs[0, 0])
ax_y   = fig.add_subplot(gs[1, 0])
ax_z   = fig.add_subplot(gs[2, 0])
ax_ax  = fig.add_subplot(gs[0, 1])
ax_flz = fig.add_subplot(gs[1, 1])
ax_frz = fig.add_subplot(gs[2, 1])

# ---- X axis: CoM + PR + PL ----
(line_x,)        = ax_x.plot([], [], lw=2,   label="CoM x (hist)",  color="tab:blue")
(line_pr_x,)     = ax_x.plot([], [], lw=1.5, label="PR x (hist)",   color="tab:orange")
(line_pl_x,)     = ax_x.plot([], [], lw=1.5, label="PL x (hist)",   color="tab:green")
(pred_x,)        = ax_x.plot([], [], lw=2,   label="CoM x (pred)",  color="tab:blue",   linestyle="--", alpha=0.6)
(pred_pr_x_ln,)  = ax_x.plot([], [], lw=1.5, label="PR x (pred)",   color="tab:orange", linestyle="--", alpha=0.6)
(pred_pl_x_ln,)  = ax_x.plot([], [], lw=1.5, label="PL x (pred)",   color="tab:green",  linestyle="--", alpha=0.6)
(pt_x,)          = ax_x.plot([], [], marker='o', linestyle='', color="red", zorder=5)
ax_x.set_ylabel("x [m]"); ax_x.grid(True); ax_x.legend(fontsize=7)

# ---- Y axis: CoM + PR + PL ----
(line_y,)        = ax_y.plot([], [], lw=2,   label="CoM y (hist)",  color="tab:blue")
(line_pr_y,)     = ax_y.plot([], [], lw=1.5, label="PR y (hist)",   color="tab:orange")
(line_pl_y,)     = ax_y.plot([], [], lw=1.5, label="PL y (hist)",   color="tab:green")
(pred_y,)        = ax_y.plot([], [], lw=2,   label="CoM y (pred)",  color="tab:blue",   linestyle="--", alpha=0.6)
(pred_pr_y_ln,)  = ax_y.plot([], [], lw=1.5, label="PR y (pred)",   color="tab:orange", linestyle="--", alpha=0.6)
(pred_pl_y_ln,)  = ax_y.plot([], [], lw=1.5, label="PL y (pred)",   color="tab:green",  linestyle="--", alpha=0.6)
(pt_y,)          = ax_y.plot([], [], marker='o', linestyle='', color="red", zorder=5)
ax_y.set_ylabel("y [m]"); ax_y.grid(True); ax_y.legend(fontsize=7)

# ---- Z axis: CoM + PR + PL ----
(line_z,)        = ax_z.plot([], [], lw=2,   label="CoM z (hist)",  color="tab:blue")
(line_pr_z,)     = ax_z.plot([], [], lw=1.5, label="PR z (hist)",   color="tab:orange")
(line_pl_z,)     = ax_z.plot([], [], lw=1.5, label="PL z (hist)",   color="tab:green")
(pred_z,)        = ax_z.plot([], [], lw=2,   label="CoM z (pred)",  color="tab:blue",   linestyle="--", alpha=0.6)
(pred_pr_z_ln,)  = ax_z.plot([], [], lw=1.5, label="PR z (pred)",   color="tab:orange", linestyle="--", alpha=0.6)
(pred_pl_z_ln,)  = ax_z.plot([], [], lw=1.5, label="PL z (pred)",   color="tab:green",  linestyle="--", alpha=0.6)
(pt_z,)          = ax_z.plot([], [], marker='o', linestyle='', color="red", zorder=5)
ax_z.set_ylabel("z [m]"); ax_z.set_xlabel("t [ms]"); ax_z.grid(True); ax_z.legend(fontsize=7)

# ---- Inputs ----
(line_ar,)     = ax_ax.plot([], [], lw=2,   label="aR (hist)",        color="tab:orange")
(line_al,)     = ax_ax.plot([], [], lw=2,   label="aL (hist)",        color="tab:green")
(pred_ar,)     = ax_ax.plot([], [], lw=1.5, label="aR (pred)",        color="tab:orange", linestyle="--", alpha=0.6)
(pred_al,)     = ax_ax.plot([], [], lw=1.5, label="aL (pred)",        color="tab:green",  linestyle="--", alpha=0.6)
(pt_ax,)       = ax_ax.plot([], [], marker='o', linestyle='', color="red", zorder=5)
ax_ax.set_ylabel("acc [m/s²]"); ax_ax.grid(True); ax_ax.legend(fontsize=7)

(line_flz,)    = ax_flz.plot([], [], lw=2, label="flz (applied hist)")
(pred_flz_ln,) = ax_flz.plot([], [], lw=2, label="flz (pred u0..uH-1)", color="red")
(pt_flz,)      = ax_flz.plot([], [], marker='o', linestyle='')
ax_flz.set_ylabel("flz [N]"); ax_flz.grid(True); ax_flz.legend(fontsize=7)

(line_frz,)    = ax_frz.plot([], [], lw=2, label="frz (applied hist)")
(pred_frz_ln,) = ax_frz.plot([], [], lw=2, label="frz (pred u0..uH-1)", color="red")
(pt_frz,)      = ax_frz.plot([], [], marker='o', linestyle='')
ax_frz.set_ylabel("frz [N]"); ax_frz.set_xlabel("t [ms]"); ax_frz.grid(True); ax_frz.legend(fontsize=7)

# ---- set limits ----
set_limits(ax_x,   times, com_x, pr_x,  pl_x,  pred_com_x_list, pred_pr_x_list, pred_pl_x_list)
set_limits(ax_y,   times, com_y, pr_y,  pl_y,  pred_com_y_list, pred_pr_y_list, pred_pl_y_list)
set_limits(ax_z,   times, com_z, pr_z,  pl_z,  pred_com_z_list, pred_pr_z_list, pred_pl_z_list)
set_limits(ax_ax,  times, u0_accr_list, u0_accl_list, pred_accr_list, pred_accl_list)
set_limits(ax_flz, times, u0_flz_list,  pred_flz_list)
set_limits(ax_frz, times, u0_frz_list,  pred_frz_list)

# ---- animation ----
ALL_LINES = (
    line_x,  line_pr_x,  line_pl_x,  pred_x,  pred_pr_x_ln,  pred_pl_x_ln,  pt_x,
    line_y,  line_pr_y,  line_pl_y,  pred_y,  pred_pr_y_ln,  pred_pl_y_ln,  pt_y,
    line_z,  line_pr_z,  line_pl_z,  pred_z,  pred_pr_z_ln,  pred_pl_z_ln,  pt_z,
    line_ar, line_al, pred_ar, pred_al, pt_ax,
    line_flz, pred_flz_ln, pt_flz,
    line_frz, pred_frz_ln, pt_frz,
)

def init():
    for ln in ALL_LINES:
        ln.set_data([], [])
    return ALL_LINES

def update(i):
    ts_state = times[:i+1]

    # history
    line_x.set_data(ts_state,  com_x[:i+1])
    line_pr_x.set_data(ts_state, pr_x[:i+1])
    line_pl_x.set_data(ts_state, pl_x[:i+1])
    pt_x.set_data([ts_state[-1]], [com_x[i]])

    line_y.set_data(ts_state,  com_y[:i+1])
    line_pr_y.set_data(ts_state, pr_y[:i+1])
    line_pl_y.set_data(ts_state, pl_y[:i+1])
    pt_y.set_data([ts_state[-1]], [com_y[i]])

    line_z.set_data(ts_state,  com_z[:i+1])
    line_pr_z.set_data(ts_state, pr_z[:i+1])
    line_pl_z.set_data(ts_state, pl_z[:i+1])
    pt_z.set_data([ts_state[-1]], [com_z[i]])

    # input history
    if i == 0:
        line_ar.set_data([], [])
        line_al.set_data([], [])
        line_flz.set_data([], [])
        line_frz.set_data([], [])
    else:
        ts_u = times[:i]
        line_ar.set_data(ts_u, u0_accr_list[:i])
        line_al.set_data(ts_u, u0_accl_list[:i])
        line_flz.set_data(ts_u, u0_flz_list[:i])
        line_frz.set_data(ts_u, u0_frz_list[:i])

    # state predictions
    pred_xt = pred_x_t_list[i]
    if pred_xt.size > 0:
        pred_x.set_data(pred_xt,       pred_com_x_list[i])
        pred_pr_x_ln.set_data(pred_xt, pred_pr_x_list[i])
        pred_pl_x_ln.set_data(pred_xt, pred_pl_x_list[i])

        pred_y.set_data(pred_xt,       pred_com_y_list[i])
        pred_pr_y_ln.set_data(pred_xt, pred_pr_y_list[i])
        pred_pl_y_ln.set_data(pred_xt, pred_pl_y_list[i])

        pred_z.set_data(pred_xt,       pred_com_z_list[i])
        pred_pr_z_ln.set_data(pred_xt, pred_pr_z_list[i])
        pred_pl_z_ln.set_data(pred_xt, pred_pl_z_list[i])
    else:
        for ln in (pred_x, pred_pr_x_ln, pred_pl_x_ln,
                   pred_y, pred_pr_y_ln, pred_pl_y_ln,
                   pred_z, pred_pr_z_ln, pred_pl_z_ln):
            ln.set_data([], [])

    # input predictions
    pred_ut = pred_u_t_list[i]
    if pred_ut.size > 0:
        pred_ar.set_data(pred_ut, pred_accr_list[i])
        pred_al.set_data(pred_ut, pred_accl_list[i])
        pred_flz_ln.set_data(pred_ut, pred_flz_list[i])
        pred_frz_ln.set_data(pred_ut, pred_frz_list[i])
    else:
        pred_ar.set_data([], [])
        pred_al.set_data([], [])
        pred_flz_ln.set_data([], [])
        pred_frz_ln.set_data([], [])

    return ALL_LINES

ani = FuncAnimation(
    fig, update, frames=len(times),
    init_func=init, interval=50, blit=True, repeat=False
)

# Spacebar toggles pause/play
anim_running = True
def toggle_animation(event):
    global anim_running
    if event.key == ' ':
        if anim_running:
            ani.event_source.stop()
        else:
            ani.event_source.start()
        anim_running = not anim_running

fig.canvas.mpl_connect('key_press_event', toggle_animation)

# Clean shutdown on window close (prevents segfault on macOS)
def on_close(event):
    ani.event_source.stop()

fig.canvas.mpl_connect('close_event', on_close)

plt.tight_layout()
plt.show()