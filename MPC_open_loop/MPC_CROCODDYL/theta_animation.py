import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

# ==== SETTINGS ====
BASE_DIR = Path("/tmp/mpc_data")
X_FILE = "x.txt"
U_FILE = "u.txt"

# State columns
THETA_COL = 10
V_COL     = 11
OMEGA_COL = 12

# Control column
ALPHA_COL = 2

# Time step between folders / MPC step (must match your logging)
DT_STEP = 2.0

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
theta_hist, v_hist, omega_hist = [], [], []

# u0(alpha) at each MPC solve time t_k (this is what you APPLY during [t_k, t_{k+1}])
u0_alpha_list = []

# state predictions
pred_x_t_list = []
pred_theta_list, pred_v_list, pred_omega_list = [], [], []

# input predictions
pred_u_t_list = []
pred_alpha_list = []

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
    theta_hist.append(x0[THETA_COL])
    v_hist.append(x0[V_COL])
    omega_hist.append(x0[OMEGA_COL])

    # current planned first input u0 (alpha) at time k (applied over next interval)
    u0_alpha_list.append(u_data[0, ALPHA_COL])

    # ---- state predictions: x1..xN plotted at t+DT..t+N*DT ----
    if x_data.shape[0] >= 2:
        theta_pred = x_data[1:, THETA_COL]
        v_pred     = x_data[1:, V_COL]
        omega_pred = x_data[1:, OMEGA_COL]

        Lx = min(theta_pred.size, v_pred.size, omega_pred.size)
        theta_pred = theta_pred[:Lx]
        v_pred     = v_pred[:Lx]
        omega_pred = omega_pred[:Lx]

        pred_xt = t + np.arange(1, Lx + 1, dtype=float) * DT_STEP

        pred_x_t_list.append(pred_xt)
        pred_theta_list.append(theta_pred)
        pred_v_list.append(v_pred)
        pred_omega_list.append(omega_pred)

        if pred_xt.size:
            t_max_pred = max(t_max_pred, pred_xt[-1])
    else:
        pred_x_t_list.append(np.array([]))
        pred_theta_list.append(np.array([]))
        pred_v_list.append(np.array([]))
        pred_omega_list.append(np.array([]))

    # ---- input predictions: u0..u_{N-1} plotted at t..t+(N-1)*DT ----
    Lu = u_data.shape[0]
    alpha_pred = u_data[:Lu, ALPHA_COL]
    pred_ut = t + np.arange(0, Lu, dtype=float) * DT_STEP  # u0 at time t

    pred_u_t_list.append(pred_ut)
    pred_alpha_list.append(alpha_pred)

    if pred_ut.size:
        t_max_pred = max(t_max_pred, pred_ut[-1])

# Convert to arrays
times      = np.asarray(times, dtype=float)
theta_hist = np.asarray(theta_hist, dtype=float)
v_hist     = np.asarray(v_hist, dtype=float)
omega_hist = np.asarray(omega_hist, dtype=float)
u0_alpha_list = np.asarray(u0_alpha_list, dtype=float)

if times.size == 0:
    raise RuntimeError("Found folders, but no readable data.")

# ---- figure & axes: 2x2 grid ----
fig = plt.figure(figsize=(13, 7))
gs = fig.add_gridspec(2, 2, height_ratios=[1, 1], width_ratios=[1, 1])

ax_theta = fig.add_subplot(gs[0, 0])
ax_v     = fig.add_subplot(gs[0, 1])
ax_omega = fig.add_subplot(gs[1, 0])
ax_alpha = fig.add_subplot(gs[1, 1])

# θ
(line_theta,)      = ax_theta.plot([], [], lw=2, label="θ (hist)")
(pred_theta_line,) = ax_theta.plot([], [], lw=2, label="θ (pred)")
(pt_theta,)        = ax_theta.plot([], [], marker='o', linestyle='')
ax_theta.set_ylabel("theta")
ax_theta.grid(True)
ax_theta.legend()

# v
(line_v,)      = ax_v.plot([], [], lw=2, label="v (hist)")
(pred_v_line,) = ax_v.plot([], [], lw=2, label="v (pred)")
(pt_v,)        = ax_v.plot([], [], marker='o', linestyle='')
ax_v.set_ylabel("v")
ax_v.grid(True)
ax_v.legend()

# ω
(line_omega,)      = ax_omega.plot([], [], lw=2, label="ω (hist)")
(pred_omega_line,) = ax_omega.plot([], [], lw=2, label="ω (pred)")
(pt_omega,)        = ax_omega.plot([], [], marker='o', linestyle='')
ax_omega.set_ylabel("omega")
ax_omega.set_xlabel("time")
ax_omega.grid(True)
ax_omega.legend()

# α
(line_alpha,)      = ax_alpha.plot([], [], lw=2, label="alpha (applied hist)")
(pred_alpha_line,) = ax_alpha.plot([], [], lw=2, label="alpha (pred u0..uH)")
(pt_alpha,)        = ax_alpha.plot([], [], marker='o', linestyle='')
ax_alpha.set_ylabel("alpha")
ax_alpha.set_xlabel("time")
ax_alpha.grid(True)
ax_alpha.legend()

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
    dv = 1e-3 if abs(vmax - vmin) < 1e-12 else 0.2 * (vmax - vmin)
    ax.set_ylim(vmin - dv, vmax + dv)

set_limits(ax_theta, times, theta_hist, pred_theta_list)
set_limits(ax_v,     times, v_hist,     pred_v_list)
set_limits(ax_omega, times, omega_hist, pred_omega_list)
set_limits(ax_alpha, times, u0_alpha_list, pred_alpha_list)

# ---- animation ----
def init():
    for ln in (line_theta, pred_theta_line, pt_theta,
               line_v,     pred_v_line,     pt_v,
               line_omega, pred_omega_line, pt_omega,
               line_alpha, pred_alpha_line, pt_alpha):
        ln.set_data([], [])
    return (line_theta, pred_theta_line, pt_theta,
            line_v,     pred_v_line,     pt_v,
            line_omega, pred_omega_line, pt_omega,
            line_alpha, pred_alpha_line, pt_alpha)

def update(i):
    # --------- STATES: history includes current x0 at time t_i ----------
    ts_state = times[:i+1]

    line_theta.set_data(ts_state, theta_hist[:i+1])
    pt_theta.set_data([ts_state[-1]], [theta_hist[i]])

    line_v.set_data(ts_state, v_hist[:i+1])
    pt_v.set_data([ts_state[-1]], [v_hist[i]])

    line_omega.set_data(ts_state, omega_hist[:i+1])
    pt_omega.set_data([ts_state[-1]], [omega_hist[i]])

    # --------- INPUTS: history should stop at t_{i-1}, NOT at t_i ----------
    # Applied alpha at time t_j is u0 from solve at time t_j, and affects [t_j, t_{j+1}]
    # So at frame i, show applied history only up to j=i-1:
    if i == 0:
        line_alpha.set_data([], [])
    else:
        ts_u = times[:i]                 # up to t_{i-1}
        alpha_applied = u0_alpha_list[:i]
        line_alpha.set_data(ts_u, alpha_applied)

    # --------- Predictions for current MPC step ----------
    # states
    pred_xt = pred_x_t_list[i]
    if pred_xt.size > 0:
        pred_theta_line.set_data(pred_xt, pred_theta_list[i])
        pred_v_line.set_data(pred_xt, pred_v_list[i])
        pred_omega_line.set_data(pred_xt, pred_omega_list[i])
    else:
        pred_theta_line.set_data([], [])
        pred_v_line.set_data([], [])
        pred_omega_line.set_data([], [])

    # inputs: predicted u0..uH-1 starting at current time t_i
    pred_ut = pred_u_t_list[i]
    if pred_ut.size > 0:
        pred_alpha_line.set_data(pred_ut, pred_alpha_list[i])
    else:
        pred_alpha_line.set_data([], [])

    return (line_theta, pred_theta_line, pt_theta,
            line_v,     pred_v_line,     pt_v,
            line_omega, pred_omega_line, pt_omega,
            line_alpha, pred_alpha_line, pt_alpha)

ani = FuncAnimation(
    fig,
    update,
    frames=len(times),
    init_func=init,
    interval=1,
    blit=False,
    repeat=False,
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
