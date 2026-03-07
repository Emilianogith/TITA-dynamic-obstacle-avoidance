#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# ==== SETTINGS ====
X_PATH = Path("/tmp/plan/x.txt")   # <-- change if needed
JUMP_PATH = Path("/tmp/plan/jump_traj.txt")

DT_MS = 2
NX = 14

PCOM_X_COL, PCOM_Y_COL, PCOM_Z_COL = 0, 1, 2
C_X_COL,    C_Y_COL,    C_Z_COL    = 6, 7, 8
THETA_COL = 10

VEC_OFF_Y = 0.567/2  # lateral offset to compute cL/cR from c and theta

def main():
    if not X_PATH.exists():
        raise FileNotFoundError(f"File not found: {X_PATH}")

    x = np.loadtxt(X_PATH, ndmin=2)
    if x.shape[1] < NX:
        raise RuntimeError(f"Expected at least {NX} columns, got {x.shape[1]}")

    # From x.txt
    pcom_x = x[:, PCOM_X_COL]
    pcom_y = x[:, PCOM_Y_COL]
    pcom_z = x[:, PCOM_Z_COL]

    c_x = x[:, C_X_COL]
    c_y = x[:, C_Y_COL]
    c_z = x[:, C_Z_COL]

    theta = x[:, THETA_COL]

    # cL/cR: offset in body-lateral direction (R(theta)*[0, 1])
    # R(theta)[0,1]^T = [-sin(theta), cos(theta)]
    off_x = -np.sin(theta) * VEC_OFF_Y
    off_y =  np.cos(theta) * VEC_OFF_Y

    cL_x, cL_y = c_x + off_x, c_y + off_y
    cR_x, cR_y = c_x - off_x, c_y - off_y

    # time for x.txt (assume starts at 0)
    t_ms = np.arange(x.shape[0]) * DT_MS

    fig, ax = plt.subplots(1, 2, figsize=(10, 6), layout="constrained")

    # --- Left: x-y plan
    ax0 = ax[0]
    ax0.plot(pcom_x, pcom_y, linewidth=2, label="CoM")
    ax0.plot(c_x, c_y, linestyle="--", linewidth=2, label="c")
    ax0.plot(cL_x, cL_y, linestyle="-.", linewidth=1.5, label="cL")
    ax0.plot(cR_x, cR_y, linestyle=":",  linewidth=1.5, label="cR")

    ax0.plot(pcom_x[0],  pcom_y[0],  marker="o", linestyle="", label="start")
    ax0.plot(pcom_x[-1], pcom_y[-1], marker="x", linestyle="", label="end")

    ax0.set_xlabel("x [m]")
    ax0.set_ylabel("y [m]")
    ax0.set_title("Plan for x–y reference trajectories")
    ax0.grid(True)
    ax0.legend()

    # nice limits for x-y
    all_x = np.concatenate([pcom_x, c_x, cL_x, cR_x])
    all_y = np.concatenate([pcom_y, c_y, cL_y, cR_y])
    xmin, xmax = all_x.min(), all_x.max()
    ymin, ymax = all_y.min(), all_y.max()
    pad_x = 0.1 * (xmax - xmin + 1e-9)
    pad_y = 0.1 * (ymax - ymin + 1e-9)
    ax0.set_xlim(xmin - pad_x, xmax + pad_x)
    ax0.set_ylim(ymin - pad_y, ymax + pad_y)

    # --- Right: z over time (from x.txt)
    ax1 = ax[1]
    ax1.plot(t_ms, pcom_z, linewidth=2, label="CoM z")
    ax1.plot(t_ms, c_z, linestyle="--", linewidth=2, label="c z")
    ax1.set_xlabel("t [ms]")
    ax1.set_ylabel("z [m]")
    ax1.set_title("Plan for z reference trajectories")
    ax1.grid(True)
    ax1.legend()

    # --- Optional: jump z plot (from jump_traj.txt)
    if JUMP_PATH.exists():
        lines = JUMP_PATH.read_text().strip().splitlines()
        first = lines[0].strip().split()
        t0 = float(first[0])

        x_jump = np.loadtxt(lines[1:], ndmin=2)
        t_jump_ms = np.arange(x_jump.shape[0]) * DT_MS + t0

        pcom_z_jump = x_jump[:, PCOM_Z_COL]
        c_z_jump = x_jump[:, C_Z_COL]

        fig_jump, ax_jump = plt.subplots(figsize=(8, 6), layout="constrained")
        ax_jump.plot(t_jump_ms, pcom_z_jump, linewidth=2, label="CoM z")
        ax_jump.plot(t_jump_ms, c_z_jump, linestyle="--", linewidth=2, label="c z")
        ax_jump.set_xlabel("t [ms]")
        ax_jump.set_ylabel("z [m]")
        ax_jump.set_title("Plan for z references in jump trajectory")
        ax_jump.grid(True)
        ax_jump.legend()

    plt.show()

if __name__ == "__main__":
    main()