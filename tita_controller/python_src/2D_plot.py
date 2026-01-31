#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from pathlib import Path

# ==== SETTINGS ====
BASE_DIR = Path("/tmp/mpc_data")
X_FILE = "x.txt"


PCOM_X_COL, PCOM_Y_COL = 0, 1
C_X_COL,    C_Y_COL    = 6, 7

THETA_COL = 10
VEC_OFF_Y = 0.567/2   


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

folders.sort(key=lambda tp: tp[0])  # sort by time


# ---- load history + prediction horizons ----
times = []
pcom_x_hist, pcom_y_hist = [], []
c_x_hist,   c_y_hist     = [], []

# NEW: cL, cR history
cL_x_hist,  cL_y_hist = [], []
cR_x_hist,  cR_y_hist = [], []

# lists of arrays; one element per MPC step
pred_pcom_x_list, pred_pcom_y_list = [], []
pred_c_x_list,    pred_c_y_list    = [], []

# NEW: prediction lists for cL, cR
pred_cL_x_list, pred_cL_y_list = [], []
pred_cR_x_list, pred_cR_y_list = [], []

for t, p in folders:
    x_path = p / X_FILE
    if not x_path.exists():
        # skip if no x.txt (keep indices aligned: empty predictions)
        pred_pcom_x_list.append(np.array([]))
        pred_pcom_y_list.append(np.array([]))
        pred_c_x_list.append(np.array([]))
        pred_c_y_list.append(np.array([]))
        pred_cL_x_list.append(np.array([]))
        pred_cL_y_list.append(np.array([]))
        pred_cR_x_list.append(np.array([]))
        pred_cR_y_list.append(np.array([]))
        continue

    x_data = np.loadtxt(x_path, ndmin=2)  # (NH+1, NX) or (NX,) etc.

    if x_data.ndim == 1:
        x_data = x_data[np.newaxis, :]  # single row

    if x_data.shape[0] == 0:
        pred_pcom_x_list.append(np.array([]))
        pred_pcom_y_list.append(np.array([]))
        pred_c_x_list.append(np.array([]))
        pred_c_y_list.append(np.array([]))
        pred_cL_x_list.append(np.array([]))
        pred_cL_y_list.append(np.array([]))
        pred_cR_x_list.append(np.array([]))
        pred_cR_y_list.append(np.array([]))
        continue

    # ---- executed state = row 0 ----
    x0 = x_data[0]
    times.append(t)

    # CoM, c history
    pcom_x = x0[PCOM_X_COL]
    pcom_y = x0[PCOM_Y_COL]
    cx     = x0[C_X_COL]
    cy     = x0[C_Y_COL]
    theta0 = x0[THETA_COL]

    pcom_x_hist.append(pcom_x)
    pcom_y_hist.append(pcom_y)
    c_x_hist.append(cx)
    c_y_hist.append(cy)

    # NEW: cL, cR history (cL = c + R(theta)*[0,1]; cR = c - R(theta)*[0,1])
    # R(theta)*[0,1]^T = [-sin(theta), cos(theta)]^T
    off_x0 = -np.sin(theta0) * VEC_OFF_Y
    off_y0 =  np.cos(theta0) * VEC_OFF_Y

    cL_x_hist.append(cx + off_x0)
    cL_y_hist.append(cy + off_y0)
    cR_x_hist.append(cx - off_x0)
    cR_y_hist.append(cy - off_y0)

    # ---- prediction = rows 1..end ----
    if x_data.shape[0] >= 2:
        pred_pcom_x = x_data[1:, PCOM_X_COL]
        pred_pcom_y = x_data[1:, PCOM_Y_COL]
        pred_c_x    = x_data[1:, C_X_COL]
        pred_c_y    = x_data[1:, C_Y_COL]
        pred_theta  = x_data[1:, THETA_COL]

        # NEW: cL, cR predictions
        off_x = -np.sin(pred_theta) * VEC_OFF_Y
        off_y =  np.cos(pred_theta) * VEC_OFF_Y

        pred_cL_x = pred_c_x + off_x
        pred_cL_y = pred_c_y + off_y
        pred_cR_x = pred_c_x - off_x
        pred_cR_y = pred_c_y - off_y
    else:
        pred_pcom_x = np.array([])
        pred_pcom_y = np.array([])
        pred_c_x    = np.array([])
        pred_c_y    = np.array([])
        pred_cL_x   = np.array([])
        pred_cL_y   = np.array([])
        pred_cR_x   = np.array([])
        pred_cR_y   = np.array([])

    pred_pcom_x_list.append(pred_pcom_x)
    pred_pcom_y_list.append(pred_pcom_y)
    pred_c_x_list.append(pred_c_x)
    pred_c_y_list.append(pred_c_y)
    pred_cL_x_list.append(pred_cL_x)
    pred_cL_y_list.append(pred_cL_y)
    pred_cR_x_list.append(pred_cR_x)
    pred_cR_y_list.append(pred_cR_y)


times       = np.asarray(times)
pcom_x_hist = np.asarray(pcom_x_hist)
pcom_y_hist = np.asarray(pcom_y_hist)
c_x_hist    = np.asarray(c_x_hist)
c_y_hist    = np.asarray(c_y_hist)

# NEW: cL, cR history as arrays
cL_x_hist   = np.asarray(cL_x_hist)
cL_y_hist   = np.asarray(cL_y_hist)
cR_x_hist   = np.asarray(cR_x_hist)
cR_y_hist   = np.asarray(cR_y_hist)

if times.size == 0:
    raise RuntimeError("Found folders, but no readable data.")


# ---- figure & axes (single xy plot) ----
fig, ax = plt.subplots(figsize=(8, 7), layout="constrained")

# history lines
(line_com_hist,)  = ax.plot([], [], linestyle='-',  linewidth=2,   label="CoM history")
(line_c_hist,)    = ax.plot([], [], linestyle='--', linewidth=2,   label="c history")
# NEW: cL, cR history lines
(line_cL_hist,)   = ax.plot([], [], linestyle='-.', linewidth=1.5, label="cL history")
(line_cR_hist,)   = ax.plot([], [], linestyle=':',  linewidth=1.5, label="cR history")

# prediction lines (for current step)
(line_com_pred,)  = ax.plot([], [], linestyle='-',  linewidth=1, label="CoM prediction")
(line_c_pred,)    = ax.plot([], [], linestyle='--', linewidth=1, label="c prediction")
# NEW: cL, cR prediction lines
(line_cL_pred,)   = ax.plot([], [], linestyle='-.', linewidth=1, label="cL prediction")
(line_cR_pred,)   = ax.plot([], [], linestyle=':',  linewidth=1, label="cR prediction")

# current markers
(marker_c_curr,)    = ax.plot([], [], marker='^', linestyle='', markersize=10, label="c current")
(marker_com_curr,)  = ax.plot([], [], marker='x', linestyle='', markersize=10, label="CoM current")
# NEW: current markers for cL, cR
(marker_cL_curr,)   = ax.plot([], [], marker='o', linestyle='', markersize=7,  label="cL current")
(marker_cR_curr,)   = ax.plot([], [], marker='s', linestyle='', markersize=7,  label="cR current")

ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_title("MPC: trajectories of c, cL, cR and CoM in x–y plane")
ax.grid(True)
ax.legend()
# ax.set_aspect("equal", adjustable="datalim")

# ---- pre-compute axis limits over all history + predictions ----
all_x = [pcom_x_hist, c_x_hist, cL_x_hist, cR_x_hist]
all_y = [pcom_y_hist, c_y_hist, cL_y_hist, cR_y_hist]

for arr_list in (pred_pcom_x_list, pred_c_x_list, pred_cL_x_list, pred_cR_x_list):
    for arr in arr_list:
        if arr.size > 0:
            all_x.append(arr)

for arr_list in (pred_pcom_y_list, pred_c_y_list, pred_cL_y_list, pred_cR_y_list):
    for arr in arr_list:
        if arr.size > 0:
            all_y.append(arr)

all_x = np.concatenate(all_x)
all_y = np.concatenate(all_y)

xmin, xmax = all_x.min(), all_x.max()
ymin, ymax = all_y.min(), all_y.max()

dx = (xmax - xmin) * 0.1 + 1e-6
dy = (ymax - ymin) * 0.1 + 1e-6

ax.set_xlim(xmin - dx, xmax + dx)
ax.set_ylim(ymin - dy, ymax + dy)


# ---- animation ----
def init():
    line_c_hist.set_data([], [])
    line_com_hist.set_data([], [])
    line_cL_hist.set_data([], [])
    line_cR_hist.set_data([], [])

    line_c_pred.set_data([], [])
    line_com_pred.set_data([], [])
    line_cL_pred.set_data([], [])
    line_cR_pred.set_data([], [])

    marker_c_curr.set_data([], [])
    marker_com_curr.set_data([], [])
    marker_cL_curr.set_data([], [])
    marker_cR_curr.set_data([], [])

    return (line_c_hist, line_com_hist,
            line_cL_hist, line_cR_hist,
            line_c_pred, line_com_pred,
            line_cL_pred, line_cR_pred,
            marker_c_curr, marker_com_curr,
            marker_cL_curr, marker_cR_curr)


def update(i):
    # history up to i
    cx_hist     = c_x_hist[:i+1]
    cy_hist     = c_y_hist[:i+1]
    comx_hist   = pcom_x_hist[:i+1]
    comy_hist   = pcom_y_hist[:i+1]
    cLx_hist    = cL_x_hist[:i+1]
    cLy_hist    = cL_y_hist[:i+1]
    cRx_hist    = cR_x_hist[:i+1]
    cRy_hist    = cR_y_hist[:i+1]

    line_c_hist.set_data(cx_hist, cy_hist)
    line_com_hist.set_data(comx_hist, comy_hist)
    line_cL_hist.set_data(cLx_hist, cLy_hist)
    line_cR_hist.set_data(cRx_hist, cRy_hist)

    # current point markers
    marker_c_curr.set_data([cx_hist[-1]],  [cy_hist[-1]])
    marker_com_curr.set_data([comx_hist[-1]], [comy_hist[-1]])
    marker_cL_curr.set_data([cLx_hist[-1]], [cLy_hist[-1]])
    marker_cR_curr.set_data([cRx_hist[-1]], [cRy_hist[-1]])

    # prediction from current step i
    pred_cx    = pred_c_x_list[i]
    pred_cy    = pred_c_y_list[i]
    pred_comx  = pred_pcom_x_list[i]
    pred_comy  = pred_pcom_y_list[i]
    pred_cLx   = pred_cL_x_list[i]
    pred_cLy   = pred_cL_y_list[i]
    pred_cRx   = pred_cR_x_list[i]
    pred_cRy   = pred_cR_y_list[i]

    if pred_cx.size > 0:
        line_c_pred.set_data(pred_cx, pred_cy)
    else:
        line_c_pred.set_data([], [])

    if pred_comx.size > 0:
        line_com_pred.set_data(pred_comx, pred_comy)
    else:
        line_com_pred.set_data([], [])

    if pred_cLx.size > 0:
        line_cL_pred.set_data(pred_cLx, pred_cLy)
    else:
        line_cL_pred.set_data([], [])

    if pred_cRx.size > 0:
        line_cR_pred.set_data(pred_cRx, pred_cRy)
    else:
        line_cR_pred.set_data([], [])

    return (line_c_hist, line_com_hist,
            line_cL_hist, line_cR_hist,
            line_c_pred, line_com_pred,
            line_cL_pred, line_cR_pred,
            marker_c_curr, marker_com_curr,
            marker_cL_curr, marker_cR_curr)


ani = FuncAnimation(
    fig,
    update,
    frames=len(times),
    init_func=init,
    interval=1,   # ms between frames; adjust as you like
    blit=True,
    repeat=False,
)

# Optional: pause/resume with space bar
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

plt.show()
