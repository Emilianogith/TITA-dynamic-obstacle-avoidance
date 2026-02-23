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
C_X_COL,    C_Y_COL, C_Z_COL    = 6, 7, 8
THETA_COL = 10

VEC_OFF_Y = 0.567/2  # lateral offset to compute cL/cR from c and theta

def main():
    if not X_PATH.exists():
        raise FileNotFoundError(f"File not found: {X_PATH}")

    x = np.loadtxt(X_PATH, ndmin=2)

    if x.shape[1] < NX:
        raise RuntimeError(f"Expected at least {NX} columns, got {x.shape[1]}")

    pcom_x = x[:, PCOM_X_COL]
    pcom_y = x[:, PCOM_Y_COL]

    c_x = x[:, C_X_COL]
    c_y = x[:, C_Y_COL]

    theta = x[:, THETA_COL]

    # cL/cR: offset in body-lateral direction (R(theta)*[0, 1])
    # R(theta)[0,1]^T = [-sin(theta), cos(theta)]
    off_x = -np.sin(theta) * VEC_OFF_Y
    off_y =  np.cos(theta) * VEC_OFF_Y

    cL_x, cL_y = c_x + off_x, c_y + off_y
    cR_x, cR_y = c_x - off_x, c_y - off_y

    fig, ax = plt.subplots(figsize=(8, 7), layout="constrained")

    ax.plot(pcom_x, pcom_y, linewidth=2, label="CoM (pcom)")
    ax.plot(c_x, c_y, linestyle="--", linewidth=2, label="c")
    ax.plot(cL_x, cL_y, linestyle="-.", linewidth=1.5, label="cL")
    ax.plot(cR_x, cR_y, linestyle=":",  linewidth=1.5, label="cR")

    # start/end markers
    ax.plot(pcom_x[0], pcom_y[0], marker="o", linestyle="", label="start")
    ax.plot(pcom_x[-1], pcom_y[-1], marker="x", linestyle="", label="end")

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("Plan (x–y): CoM and c / cL / cR")
    ax.grid(True)
    ax.legend()

    # make axes nice
    all_x = np.concatenate([pcom_x, c_x, cL_x, cR_x])
    all_y = np.concatenate([pcom_y, c_y, cL_y, cR_y])
    xmin, xmax = all_x.min(), all_x.max()
    ymin, ymax = all_y.min(), all_y.max()
    pad_x = 0.1 * (xmax - xmin + 1e-9)
    pad_y = 0.1 * (ymax - ymin + 1e-9)
    ax.set_xlim(xmin - pad_x, xmax + pad_x)
    ax.set_ylim(ymin - pad_y, ymax + pad_y)

    # optional: equal aspect for geometry clarity
    # ax.set_aspect("equal", adjustable="box")


    if JUMP_PATH.exists():
        lines = JUMP_PATH.read_text().strip().splitlines()
        first = lines[0].strip().split()
        t0 = float(first[0])
 
        x_jump = np.loadtxt(lines[1:], ndmin=2)
        t_ms = np.arange(x_jump.shape[0]) * DT_MS + t0

        pcom_z = x_jump[:, PCOM_Z_COL]
        c_z = x_jump[:, C_Z_COL]

        fig_jump, ax_jump = plt.subplots(figsize=(8, 7), layout="constrained")

        ax_jump.plot(t_ms, pcom_z, linewidth=2, label="CoM z")
        ax_jump.plot(t_ms, c_z, linestyle="--", linewidth=2, label="c z")
    
        ax_jump.set_xlabel("time [ms]")
        ax_jump.set_ylabel("z [m]")
        ax_jump.set_title("Plan (z) in jump trajectory")
        ax_jump.grid(True)
        ax_jump.legend()



    plt.show()

if __name__ == "__main__":
    main()
