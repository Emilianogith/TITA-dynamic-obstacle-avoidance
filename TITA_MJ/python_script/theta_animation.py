import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

# ==== SETTINGS ====
BASE_DIR = Path("/tmp/mpc_data")
X_FILE = "x.txt"
U_FILE = "u.txt"


THETA_COL = 10
V_COL     = 11
OMEGA_COL = 12

ALPHA_COL = 2 

# DT_STEP = 2.0 
DT_STEP = 10.0   # if dt in MPC is 0.01 s

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

# ---- load executed point + prediction horizon ----
times = []
theta_hist, v_hist, omega_hist = [], [], []
alpha_hist = []

pred_t_list = []
pred_theta_list, pred_v_list, pred_omega_list = [], [], []
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

    # executed-now (row 0)
    x0 = x_data[0]
    u0 = u_data[0]

    times.append(t)
    theta_hist.append(x0[THETA_COL])
    v_hist.append(x0[V_COL])
    omega_hist.append(x0[OMEGA_COL])
    alpha_hist.append(u0[ALPHA_COL])

    # predictions: rows 1..end
    if x_data.shape[0] >= 2:
        theta_pred = x_data[1:, THETA_COL]
        v_pred     = x_data[1:, V_COL]
        omega_pred = x_data[1:, OMEGA_COL]

        # u has NH rows; we match length with states (minus the first executed one)
        L = min(theta_pred.shape[0], v_pred.shape[0], omega_pred.shape[0],
                u_data.shape[0] - 1)

        theta_pred = theta_pred[:L]
        v_pred     = v_pred[:L]
        omega_pred = omega_pred[:L]

        alpha_pred = u_data[1:L+1, ALPHA_COL] if u_data.shape[0] > 1 else np.array([])

        pred_t = t + np.arange(1, L + 1, dtype=float) * DT_STEP

        pred_t_list.append(pred_t)
        pred_theta_list.append(theta_pred)
        pred_v_list.append(v_pred)
        pred_omega_list.append(omega_pred)
        pred_alpha_list.append(alpha_pred)

        if pred_t.size:
            t_max_pred = max(t_max_pred, pred_t[-1])
    else:
        pred_t_list.append(np.array([]))
        pred_theta_list.append(np.array([]))
        pred_v_list.append(np.array([]))
        pred_omega_list.append(np.array([]))
        pred_alpha_list.append(np.array([]))

times       = np.asarray(times)
theta_hist  = np.asarray(theta_hist)
v_hist      = np.asarray(v_hist)
omega_hist  = np.asarray(omega_hist)
alpha_hist  = np.asarray(alpha_hist)

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
ax_theta.set_ylabel("theta"); ax_theta.grid(True); ax_theta.legend()

# v
(line_v,)      = ax_v.plot([], [], lw=2, label="v (hist)")
(pred_v_line,) = ax_v.plot([], [], lw=2, label="v (pred)")
(pt_v,)        = ax_v.plot([], [], marker='o', linestyle='')
ax_v.set_ylabel("v"); ax_v.grid(True); ax_v.legend()

# ω
(line_omega,)      = ax_omega.plot([], [], lw=2, label="ω (hist)")
(pred_omega_line,) = ax_omega.plot([], [], lw=2, label="ω (pred)")
(pt_omega,)        = ax_omega.plot([], [], marker='o', linestyle='')
ax_omega.set_ylabel("omega"); ax_omega.set_xlabel("time"); ax_omega.grid(True); ax_omega.legend()

# α
(line_alpha,)      = ax_alpha.plot([], [], lw=2, label="alpha (hist)")
(pred_alpha_line,) = ax_alpha.plot([], [], lw=2, label="alpha (pred)")
(pt_alpha,)        = ax_alpha.plot([], [], marker='o', linestyle='')
ax_alpha.set_ylabel("alpha"); ax_alpha.set_xlabel("time"); ax_alpha.grid(True); ax_alpha.legend()


def set_limits(ax, t_hist, *value_lists):
    """Set x/y limits using history and prediction arrays."""
    # time
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
    vmin, vmax = np.min(V), np.max(V)
    if abs(vmax - vmin) < 1e-9:
        dv = 1e-3
    else:
        dv = 0.2 * (vmax - vmin)
    ax.set_ylim(vmin - dv, vmax + dv)


set_limits(ax_theta, times, theta_hist, pred_theta_list)
set_limits(ax_v,     times, v_hist,     pred_v_list)
set_limits(ax_omega, times, omega_hist, pred_omega_list)
set_limits(ax_alpha, times, alpha_hist, pred_alpha_list)


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
    ts = times[:i+1]

    # history
    line_theta.set_data(ts, theta_hist[:i+1])
    pt_theta.set_data([ts[-1]], [theta_hist[i]])

    line_v.set_data(ts, v_hist[:i+1])
    pt_v.set_data([ts[-1]], [v_hist[i]])

    line_omega.set_data(ts, omega_hist[:i+1])
    pt_omega.set_data([ts[-1]], [omega_hist[i]])

    line_alpha.set_data(ts, alpha_hist[:i+1])
    pt_alpha.set_data([ts[-1]], [alpha_hist[i]])

    # predictions for current MPC step
    pred_t = pred_t_list[i]
    if pred_t.size > 0:
        pred_theta_line.set_data(pred_t, pred_theta_list[i])
        pred_v_line.set_data(pred_t, pred_v_list[i])
        pred_omega_line.set_data(pred_t, pred_omega_list[i])
        pred_alpha_line.set_data(pred_t, pred_alpha_list[i])
    else:
        pred_theta_line.set_data([], [])
        pred_v_line.set_data([], [])
        pred_omega_line.set_data([], [])
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
    blit=True,
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
