import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

# ==== SETTINGS ====
BASE_DIR = Path("/tmp/mpc_data")
X_FILE = "x.txt"
U_FILE = "u.txt"

# Columns in x.txt
COM_X_COL, COM_Y_COL, COM_Z_COL = 9, 10, 2
PC_X_COL,  PC_Y_COL             = 6, 7

# Columns in u.txt
ACC_X_COL = 0
ACC_Y_COL = 1
FZ_COL    = 5

DT_MS = 1.0  # 0.002 s


def parse_timestep(name: str):
    try: return float(name)
    except ValueError: return None


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
com_x, com_y, com_z = [], [], []
pc_x,  pc_y         = [], []
fz_hist             = []
accx_hist, accy_hist = [], []

pred_t_list = []
pred_com_x_list, pred_com_y_list, pred_com_z_list = [], [], []
pred_pc_x_list,  pred_pc_y_list                   = [], []
pred_fz_list                                      = []
pred_accx_list, pred_accy_list                    = [], []

t_max_pred = -np.inf

for t, p in folders:
    x_path = p / X_FILE
    u_path = p / U_FILE
    if not x_path.exists() or not u_path.exists():
        continue

    x_data = np.loadtxt(x_path, ndmin=2)  # (NH+1, state_dim)
    u_data = np.loadtxt(u_path, ndmin=2)  # (NH or NH+1, control_dim)

    if x_data.shape[0] == 0 or u_data.shape[0] == 0:
        continue

    # executed-now (row 0)
    x0 = x_data[0]
    u0 = u_data[0]

    times.append(t)
    com_x.append(x0[COM_X_COL])
    com_y.append(x0[COM_Y_COL])
    com_z.append(x0[COM_Z_COL])

    pc_x.append(x0[PC_X_COL])
    pc_y.append(x0[PC_Y_COL])

    # store accelerations from u0
    accx_hist.append(u0[ACC_X_COL])
    accy_hist.append(u0[ACC_Y_COL])

    # store vertical force
    fz_hist.append(u0[FZ_COL])

    # predictions: rows 1..end
    if x_data.shape[0] >= 2:
        pred_com_x = x_data[1:, COM_X_COL]
        pred_com_y = x_data[1:, COM_Y_COL]
        pred_com_z = x_data[1:, COM_Z_COL]
        pred_pc_x  = x_data[1:, PC_X_COL]
        pred_pc_y  = x_data[1:, PC_Y_COL]

        L = min(pred_com_x.shape[0], pred_com_y.shape[0], pred_com_z.shape[0],
                pred_pc_x.shape[0], pred_pc_y.shape[0], u_data.shape[0])

        pred_com_x = pred_com_x[:L]
        pred_com_y = pred_com_y[:L]
        pred_com_z = pred_com_z[:L]
        pred_pc_x  = pred_pc_x[:L]
        pred_pc_y  = pred_pc_y[:L]

        pred_accx = u_data[1:L, ACC_X_COL]
        pred_accy = u_data[1:L, ACC_Y_COL]
        pred_fz   = u_data[1:L, FZ_COL]

        pred_t = t + np.arange(1, L + 1, dtype=float) * DT_MS

        pred_t_list.append(pred_t)
        pred_com_x_list.append(pred_com_x)
        pred_com_y_list.append(pred_com_y)
        pred_com_z_list.append(pred_com_z)
        pred_pc_x_list.append(pred_pc_x)
        pred_pc_y_list.append(pred_pc_y)

        pred_accx_list.append(pred_accx)
        pred_accy_list.append(pred_accy)
        pred_fz_list.append(pred_fz)

        if pred_t.size:
            t_max_pred = max(t_max_pred, pred_t[-1])
    else:
        pred_t_list.append(np.array([]))
        pred_com_x_list.append(np.array([]))
        pred_com_y_list.append(np.array([]))
        pred_com_z_list.append(np.array([]))
        pred_pc_x_list.append(np.array([]))
        pred_pc_y_list.append(np.array([]))

        pred_accx_list.append(np.array([]))
        pred_accy_list.append(np.array([]))
        pred_fz_list.append(np.array([]))


times  = np.asarray(times)
com_x  = np.asarray(com_x)
com_y  = np.asarray(com_y)
com_z  = np.asarray(com_z)
pc_x   = np.asarray(pc_x)
pc_y   = np.asarray(pc_y)
fz_hist = np.asarray(fz_hist)
accx_hist = np.asarray(accx_hist)
accy_hist = np.asarray(accy_hist)

if times.size == 0:
    raise RuntimeError("Found folders, but no readable data.")


# ---- figure & axes: 5x2 grid ----
fig = plt.figure(figsize=(13, 7))
gs = fig.add_gridspec(3, 2, height_ratios=[1,1,1], width_ratios=[1,1])

ax_x  = fig.add_subplot(gs[0, 0])
ax_y  = fig.add_subplot(gs[1, 0])
ax_z  = fig.add_subplot(gs[2, 0])
ax_ax = fig.add_subplot(gs[0, 1])
ax_ay = fig.add_subplot(gs[1, 1])
ax_fz = fig.add_subplot(gs[2, 1])


# CoM plots
(line_x,)     = ax_x.plot([], [], lw=2, label="CoM x")
(line_pc_x,)  = ax_x.plot([], [], lw=1.5, label="Pc x")
(pred_x,)     = ax_x.plot([], [], lw=2, color='red', label="pred CoM x")
(pred_pc_x,)  = ax_x.plot([], [], lw=1.5, color='red', linestyle='--', label="pred Pc x")
(pt_x,)       = ax_x.plot([], [], marker='o', linestyle='')
ax_x.set_ylabel("x"); ax_x.grid(True); ax_x.legend()

(line_y,)     = ax_y.plot([], [], lw=2, label="CoM y")
(line_pc_y,)  = ax_y.plot([], [], lw=1.5, label="Pc y")
(pred_y,)     = ax_y.plot([], [], lw=2, color='red', label="pred CoM y")
(pred_pc_y,)  = ax_y.plot([], [], lw=1.5, color='red', linestyle='--', label="pred Pc y")
(pt_y,)       = ax_y.plot([], [], marker='o', linestyle='')
ax_y.set_ylabel("y"); ax_y.grid(True); ax_y.legend()

(line_z,)     = ax_z.plot([], [], lw=2, label="CoM z")
(pred_z,)     = ax_z.plot([], [], lw=2, color='red', label="pred CoM z")
(pt_z,)       = ax_z.plot([], [], marker='o', linestyle='')
ax_z.set_ylabel("z"); ax_z.grid(True); ax_z.legend()


# NEW: accelerations
(line_ax,)    = ax_ax.plot([], [], lw=2, label="acc_x")
(pred_ax,)    = ax_ax.plot([], [], lw=2, color='red', label="pred acc_x")
(pt_ax,)      = ax_ax.plot([], [], marker='o', linestyle='')
ax_ax.set_ylabel("acc_x"); ax_ax.grid(True); ax_ax.legend()

(line_ay,)    = ax_ay.plot([], [], lw=2, label="acc_y")
(pred_ay,)    = ax_ay.plot([], [], lw=2, color='red', label="pred acc_y")
(pt_ay,)      = ax_ay.plot([], [], marker='o', linestyle='')
ax_ay.set_ylabel("acc_y"); ax_ay.grid(True); ax_ay.legend()


# Force z
(line_fz,)    = ax_fz.plot([], [], lw=2, label="f_z")
(pred_fz,)    = ax_fz.plot([], [], lw=2, color='red', label="pred f_z")
(pt_fz,)      = ax_fz.plot([], [], marker='o', linestyle='')
ax_fz.set_xlabel("t [ms]"); ax_fz.set_ylabel("f_z")
ax_fz.grid(True); ax_fz.legend()


# Axis limits helper
# def set_limits(ax, t_hist, values):
#     vmin, vmax = values.min(), values.max()
#     dv = (vmax - vmin) * 0.1 + 1e-9
#     ax.set_xlim(t_hist.min(), max(t_hist.max(), t_max_pred))
#     ax.set_ylim(vmin - dv, vmax + dv)

def set_limits(ax, t_hist, *value_lists):
    """
    Set axis limits using:
    - time history
    - multiple value arrays: history and predictions
    Each element in value_lists must be a list of arrays or a single array.
    """
    # ---- time axis ----
    t_min = np.min(t_hist)
    t_max_hist = np.max(t_hist)
    t_max = max(t_max_hist, t_max_pred)
    ax.set_xlim(t_min, t_max)

    # ---- collect all values ----
    V = []
    for v in value_lists:
        if isinstance(v, (list, tuple)):
            for arr in v:
                if arr is not None and len(arr) > 0:
                    V.append(np.asarray(arr))
        else:
            arr = np.asarray(v)
            if arr.size > 0:
                V.append(arr)

    if len(V) == 0:
        return

    V = np.concatenate(V)
    vmin, vmax = np.min(V), np.max(V)

    # ---- handle nearly-flat signals ----
    if abs(vmax - vmin) < 1e-9:
        dv = 1e-3
    else:
        dv = 0.20 * (vmax - vmin)

    ax.set_ylim(vmin - dv, vmax + dv)




set_limits(ax_x, times,
    com_x, pc_x,
    pred_com_x_list, pred_pc_x_list
)
set_limits(ax_y, times,
    com_y, pc_y,
    pred_com_y_list, pred_pc_y_list
)
set_limits(ax_z, times, com_z, pred_com_z_list)
set_limits(ax_ax, times, accx_hist, pred_accx_list)
set_limits(ax_ay, times, accy_hist, pred_accy_list)
set_limits(ax_fz, times, fz_hist, pred_fz_list)


# ---- animation ----
def init():
    for ln in (line_x, line_pc_x, pred_x, pred_pc_x, pt_x,
               line_y, line_pc_y, pred_y, pred_pc_y, pt_y,
               line_z, pred_z, pt_z,
               line_ax, pred_ax, pt_ax,
               line_ay, pred_ay, pt_ay,
               line_fz, pred_fz, pt_fz):
        ln.set_data([], [])
    return (line_x, line_pc_x, pred_x, pred_pc_x, pt_x,
            line_y, line_pc_y, pred_y, pred_pc_y, pt_y,
            line_z, pred_z, pt_z,
            line_ax, pred_ax, pt_ax,
            line_ay, pred_ay, pt_ay,
            line_fz, pred_fz, pt_fz)


def update(i):
    ts = times[:i+1]

    # history
    line_x.set_data(ts, com_x[:i+1]);     line_pc_x.set_data(ts, pc_x[:i+1]);  pt_x.set_data([ts[-1]], [com_x[i]])
    line_y.set_data(ts, com_y[:i+1]);     line_pc_y.set_data(ts, pc_y[:i+1]);  pt_y.set_data([ts[-1]], [com_y[i]])
    line_z.set_data(ts, com_z[:i+1]);     pt_z.set_data([ts[-1]], [com_z[i]])

    line_ax.set_data(ts, accx_hist[:i+1]); pt_ax.set_data([ts[-1]], [accx_hist[i]])
    line_ay.set_data(ts, accy_hist[:i+1]); pt_ay.set_data([ts[-1]], [accy_hist[i]])

    line_fz.set_data(ts, fz_hist[:i+1]);  pt_fz.set_data([ts[-1]], [fz_hist[i]])

    # predictions
    pred_t = pred_t_list[i]

    if pred_t.size > 0:
        pred_x.set_data(pred_t, pred_com_x_list[i])
        pred_pc_x.set_data(pred_t, pred_pc_x_list[i])
        pred_y.set_data(pred_t, pred_com_y_list[i])
        pred_pc_y.set_data(pred_t, pred_pc_y_list[i])
        pred_z.set_data(pred_t, pred_com_z_list[i])

        pred_ax.set_data(pred_t[:-1], pred_accx_list[i])
        pred_ay.set_data(pred_t[:-1], pred_accy_list[i])
        pred_fz.set_data(pred_t[:-1], pred_fz_list[i])
    else:
        pred_x.set_data([], []);  pred_pc_x.set_data([], [])
        pred_y.set_data([], []);  pred_pc_y.set_data([], [])
        pred_z.set_data([], [])
        pred_ax.set_data([], [])
        pred_ay.set_data([], [])
        pred_fz.set_data([], [])

    return (line_x, line_pc_x, pred_x, pred_pc_x, pt_x,
            line_y, line_pc_y, pred_y, pred_pc_y, pt_y,
            line_z, pred_z, pt_z,
            line_ax, pred_ax, pt_ax,
            line_ay, pred_ay, pt_ay,
            line_fz, pred_fz, pt_fz)


ani = FuncAnimation(fig, update, frames=len(times),
                    init_func=init, interval=1, blit=False, repeat=False)

# change blit to true if you want the animation faster

anim_running = True  # global flag

def toggle_animation(event):
    global anim_running
    # choose your key: here 's' for "start/stop"
    if event.key == ' ':
        if anim_running:
            ani.event_source.stop()
        else:
            ani.event_source.start()
        anim_running = not anim_running

# connect the callback to key-press events on the figure
fig.canvas.mpl_connect('key_press_event', toggle_animation)

plt.tight_layout()
plt.show()
