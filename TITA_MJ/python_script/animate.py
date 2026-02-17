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
PC_X_COL,  PC_Y_COL,  PC_Z_COL = 6, 7, 8   # <-- add pc_z column (change if your file differs)

# Columns in u.txt
ACC_X_COL  = 0
FLZ_COL    = 5
FRZ_COL    = 8

# DT_MS = 2.0  # must match your folder-to-folder step
DT_MS = 10.0

def parse_timestep(name: str):
    try:
        return float(name)
    except ValueError:
        return None

# ---- discover timestep folders (names must be numeric) ----
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
pc_x,  pc_y,  pc_z  = [], [], []

# "planned u0 at time k" (applied during [t_k, t_{k+1}])
u0_accx_list = []
u0_flz_list  = []
u0_frz_list  = []

# state predictions (x1..xN)
pred_x_t_list = []
pred_com_x_list, pred_com_y_list, pred_com_z_list = [], [], []
pred_pc_x_list,  pred_pc_y_list,  pred_pc_z_list  = [], [], []

# input predictions (u0..u_{N-1})
pred_u_t_list = []
pred_accx_list, pred_flz_list, pred_frz_list = [], [], []

t_max_pred = -np.inf

for t, p in folders:
    x_path = p / X_FILE
    u_path = p / U_FILE
    if not x_path.exists() or not u_path.exists():
        continue

    x_data = np.loadtxt(x_path, ndmin=2)  # (NH+1, state_dim)
    u_data = np.loadtxt(u_path, ndmin=2)  # (NH, control_dim)

    if x_data.shape[0] == 0 or u_data.shape[0] == 0:
        continue

    # executed-now state (row 0)
    x0 = x_data[0]
    times.append(t)
    com_x.append(x0[COM_X_COL]); com_y.append(x0[COM_Y_COL]); com_z.append(x0[COM_Z_COL])
    pc_x.append(x0[PC_X_COL]);   pc_y.append(x0[PC_Y_COL]);   pc_z.append(x0[PC_Z_COL])

    # planned u0 at this MPC step (this is what you apply over [t, t+DT])
    u0_accx_list.append(u_data[0, ACC_X_COL])
    u0_flz_list.append(u_data[0, FLZ_COL])
    u0_frz_list.append(u_data[0, FRZ_COL])

    # ---- state predictions: x1..xN plotted at t+DT..t+N*DT ----
    if x_data.shape[0] >= 2:
        pred_com_x = x_data[1:, COM_X_COL]
        pred_com_y = x_data[1:, COM_Y_COL]
        pred_com_z = x_data[1:, COM_Z_COL]
        pred_pc_x  = x_data[1:, PC_X_COL]
        pred_pc_y  = x_data[1:, PC_Y_COL]
        pred_pc_z  = x_data[1:, PC_Z_COL]

        Lx = min(pred_com_x.size, pred_com_y.size, pred_com_z.size,
                 pred_pc_x.size, pred_pc_y.size, pred_pc_z.size)
        pred_com_x = pred_com_x[:Lx]
        pred_com_y = pred_com_y[:Lx]
        pred_com_z = pred_com_z[:Lx]
        pred_pc_x  = pred_pc_x[:Lx]
        pred_pc_y  = pred_pc_y[:Lx]
        pred_pc_z  = pred_pc_z[:Lx]

        pred_xt = t + np.arange(1, Lx + 1, dtype=float) * DT_MS

        pred_x_t_list.append(pred_xt)
        pred_com_x_list.append(pred_com_x)
        pred_com_y_list.append(pred_com_y)
        pred_com_z_list.append(pred_com_z)
        pred_pc_x_list.append(pred_pc_x)
        pred_pc_y_list.append(pred_pc_y)
        pred_pc_z_list.append(pred_pc_z)

        if pred_xt.size:
            t_max_pred = max(t_max_pred, pred_xt[-1])
    else:
        pred_x_t_list.append(np.array([]))
        pred_com_x_list.append(np.array([]))
        pred_com_y_list.append(np.array([]))
        pred_com_z_list.append(np.array([]))
        pred_pc_x_list.append(np.array([]))
        pred_pc_y_list.append(np.array([]))
        pred_pc_z_list.append(np.array([]))

    # ---- input predictions: u0..u_{N-1} plotted at t..t+(N-1)*DT ----
    Lu = u_data.shape[0]
    pred_accx = u_data[:Lu, ACC_X_COL]
    pred_flz  = u_data[:Lu, FLZ_COL]
    pred_frz  = u_data[:Lu, FRZ_COL]

    pred_ut = t + np.arange(0, Lu, dtype=float) * DT_MS  # starts at t for u0

    pred_u_t_list.append(pred_ut)
    pred_accx_list.append(pred_accx)
    pred_flz_list.append(pred_flz)
    pred_frz_list.append(pred_frz)

    if pred_ut.size:
        t_max_pred = max(t_max_pred, pred_ut[-1])

# Convert to arrays
times = np.asarray(times, dtype=float)
com_x = np.asarray(com_x, dtype=float); com_y = np.asarray(com_y, dtype=float); com_z = np.asarray(com_z, dtype=float)
pc_x  = np.asarray(pc_x, dtype=float);  pc_y  = np.asarray(pc_y, dtype=float);  pc_z  = np.asarray(pc_z, dtype=float)

u0_accx_list = np.asarray(u0_accx_list, dtype=float)
u0_flz_list  = np.asarray(u0_flz_list, dtype=float)
u0_frz_list  = np.asarray(u0_frz_list, dtype=float)

if times.size == 0:
    raise RuntimeError("Found folders, but no readable data.")

# ---- figure & axes: 3x2 grid ----
fig = plt.figure(figsize=(13, 7))
gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1], width_ratios=[1, 1])

ax_x   = fig.add_subplot(gs[0, 0])
ax_y   = fig.add_subplot(gs[1, 0])
ax_z   = fig.add_subplot(gs[2, 0])
ax_ax  = fig.add_subplot(gs[0, 1])
ax_flz = fig.add_subplot(gs[1, 1])
ax_frz = fig.add_subplot(gs[2, 1])

# ---- CoM / Pc plots ----
(line_pc_x,)  = ax_x.plot([], [], lw=1.5, label="Pc x (hist)", color="tab:orange")
(line_x,)     = ax_x.plot([], [], lw=2,   label="CoM x (hist)", color="tab:blue")
(pred_x,)     = ax_x.plot([], [], lw=2,   label="CoM x (pred)",  color="red")
(pred_pc_x,)  = ax_x.plot([], [], lw=1.5, linestyle="--", label="Pc x (pred)", color="red")
(pt_x,)       = ax_x.plot([], [], marker='o', linestyle='', color="tab:green")
ax_x.set_ylabel("x"); ax_x.grid(True); ax_x.legend()

(line_pc_y,)  = ax_y.plot([], [], lw=1.5, label="Pc y (hist)", color="tab:orange")
(line_y,)     = ax_y.plot([], [], lw=2,   label="CoM y (hist)", color="tab:blue")
(pred_y,)     = ax_y.plot([], [], lw=2,   label="CoM y (pred)",  color="red")
(pred_pc_y,)  = ax_y.plot([], [], lw=1.5, linestyle="--", label="Pc y (pred)", color="red")
(pt_y,)       = ax_y.plot([], [], marker='o', linestyle='', color="tab:green")
ax_y.set_ylabel("y"); ax_y.grid(True); ax_y.legend()

# --- z: add Pc z hist + pred (your request) ---
(line_pc_z,)  = ax_z.plot([], [], lw=1.5, label="Pc z (hist)", color="tab:orange")
(line_z,)     = ax_z.plot([], [], lw=2,   label="CoM z (hist)", color="tab:blue")
(pred_z,)     = ax_z.plot([], [], lw=2,   label="CoM z (pred)",  color="red")
(pred_pc_z,)  = ax_z.plot([], [], lw=1.5, linestyle="--", label="Pc z (pred)", color="red")
(pt_z,)       = ax_z.plot([], [], marker='o', linestyle='', color="tab:green")
ax_z.set_ylabel("z"); ax_z.set_xlabel("t [ms]"); ax_z.grid(True); ax_z.legend()

# ---- Inputs ----
(line_ax,)     = ax_ax.plot([], [], lw=2, label="a (applied hist)")
(pred_ax,)     = ax_ax.plot([], [], lw=2, label="a (pred u0..uH)", color="red")
(pt_ax,)       = ax_ax.plot([], [], marker='o', linestyle='')
ax_ax.set_ylabel("a"); ax_ax.grid(True); ax_ax.legend()

(line_flz,)    = ax_flz.plot([], [], lw=2, label="flz (applied hist)")
(pred_flz_ln,) = ax_flz.plot([], [], lw=2, label="flz (pred u0..uH)", color="red")
(pt_flz,)      = ax_flz.plot([], [], marker='o', linestyle='')
ax_flz.set_ylabel("flz"); ax_flz.grid(True); ax_flz.legend()

(line_frz,)    = ax_frz.plot([], [], lw=2, label="frz (applied hist)")
(pred_frz_ln,) = ax_frz.plot([], [], lw=2, label="frz (pred u0..uH)", color="red")
(pt_frz,)      = ax_frz.plot([], [], marker='o', linestyle='')
ax_frz.set_ylabel("frz"); ax_frz.set_xlabel("t [ms]"); ax_frz.grid(True); ax_frz.legend()

def set_limits(ax, t_hist, *value_lists):
    # time axis
    t_min = np.min(t_hist)
    t_max_hist = np.max(t_hist)
    t_max = max(t_max_hist, t_max_pred if t_max_pred > -np.inf else t_max_hist)
    ax.set_xlim(t_min, t_max)

    # values
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
    if abs(vmax - vmin) < 1e-12:
        dv = 1e-3
    else:
        dv = 0.20 * (vmax - vmin)
    ax.set_ylim(vmin - dv, vmax + dv)

# limits: include histories + all predictions
set_limits(ax_x,  times, com_x, pc_x, pred_com_x_list, pred_pc_x_list)
set_limits(ax_y,  times, com_y, pc_y, pred_com_y_list, pred_pc_y_list)
set_limits(ax_z,  times, com_z, pc_z, pred_com_z_list, pred_pc_z_list)

set_limits(ax_ax,  times, u0_accx_list, pred_accx_list)
set_limits(ax_flz, times, u0_flz_list,  pred_flz_list)
set_limits(ax_frz, times, u0_frz_list,  pred_frz_list)

# ---- animation ----
def init():
    for ln in (
        line_x, line_pc_x, pred_x, pred_pc_x, pt_x,
        line_y, line_pc_y, pred_y, pred_pc_y, pt_y,
        line_z, line_pc_z, pred_z, pred_pc_z, pt_z,
        line_ax, pred_ax, pt_ax,
        line_flz, pred_flz_ln, pt_flz,
        line_frz, pred_frz_ln, pt_frz
    ):
        ln.set_data([], [])
    return (
        line_x, line_pc_x, pred_x, pred_pc_x, pt_x,
        line_y, line_pc_y, pred_y, pred_pc_y, pt_y,
        line_z, line_pc_z, pred_z, pred_pc_z, pt_z,
        line_ax, pred_ax, pt_ax,
        line_flz, pred_flz_ln, pt_flz,
        line_frz, pred_frz_ln, pt_frz
    )

def update(i):
    # ----- STATES: show history up to and including current time -----
    ts_state = times[:i+1]

    line_x.set_data(ts_state, com_x[:i+1])
    line_pc_x.set_data(ts_state, pc_x[:i+1])
    pt_x.set_data([ts_state[-1]], [com_x[i]])

    line_y.set_data(ts_state, com_y[:i+1])
    line_pc_y.set_data(ts_state, pc_y[:i+1])
    pt_y.set_data([ts_state[-1]], [com_y[i]])

    line_z.set_data(ts_state, com_z[:i+1])
    line_pc_z.set_data(ts_state, pc_z[:i+1])
    pt_z.set_data([ts_state[-1]], [com_z[i]])

    # ----- INPUTS: show applied history only up to t_{i-1}, NOT at t_i -----
    # At time t_j you compute/apply u0; it acts over [t_j, t_{j+1}].
    if i == 0:
        line_ax.set_data([], [])
        line_flz.set_data([], [])
        line_frz.set_data([], [])
    else:
        ts_u = times[:i]  # up to t_{i-1}
        line_ax.set_data(ts_u, u0_accx_list[:i])
        line_flz.set_data(ts_u, u0_flz_list[:i])
        line_frz.set_data(ts_u, u0_frz_list[:i])

    # ----- predictions for current MPC step -----
    pred_xt = pred_x_t_list[i]
    if pred_xt.size > 0:
        pred_x.set_data(pred_xt, pred_com_x_list[i])
        pred_pc_x.set_data(pred_xt, pred_pc_x_list[i])

        pred_y.set_data(pred_xt, pred_com_y_list[i])
        pred_pc_y.set_data(pred_xt, pred_pc_y_list[i])

        pred_z.set_data(pred_xt, pred_com_z_list[i])
        pred_pc_z.set_data(pred_xt, pred_pc_z_list[i])
    else:
        pred_x.set_data([], []); pred_pc_x.set_data([], [])
        pred_y.set_data([], []); pred_pc_y.set_data([], [])
        pred_z.set_data([], []); pred_pc_z.set_data([], [])

    pred_ut = pred_u_t_list[i]
    if pred_ut.size > 0:
        pred_ax.set_data(pred_ut, pred_accx_list[i])
        pred_flz_ln.set_data(pred_ut, pred_flz_list[i])
        pred_frz_ln.set_data(pred_ut, pred_frz_list[i])
    else:
        pred_ax.set_data([], [])
        pred_flz_ln.set_data([], [])
        pred_frz_ln.set_data([], [])

    return (
        line_x, line_pc_x, pred_x, pred_pc_x, pt_x,
        line_y, line_pc_y, pred_y, pred_pc_y, pt_y,
        line_z, line_pc_z, pred_z, pred_pc_z, pt_z,
        line_ax, pred_ax, pt_ax,
        line_flz, pred_flz_ln, pt_flz,
        line_frz, pred_frz_ln, pt_frz
    )

ani = FuncAnimation(
    fig, update, frames=len(times),
    init_func=init, interval=1, blit=False, repeat=False
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

plt.tight_layout()
plt.show()
