#!/usr/bin/env python3
"""
Plot only:
- pCOM Estimated vs Odom
- pCL Estimated
- pCR Estimated

No error plots.
"""

import pandas as pd
import matplotlib.pyplot as plt
import argparse

import numpy as np
from pathlib import Path

def plot_xyz(ax, t, x1, y1, z1, x2, y2, z2, title):
    ax.plot(t, x1, label=x1.name, color="C0")
    ax.plot(t, x2, "--", label=x2.name, color="C0")

    ax.plot(t, y1, label=y1.name, color="C1")
    ax.plot(t, y2, "--", label=y2.name, color="C1")

    ax.plot(t, z1, label=z1.name, color="C2")
    ax.plot(t, z2, "--", label=z2.name, color="C2")

    ax.set_title(title)
    ax.set_ylabel("Position [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(ncol=2, fontsize=9)


def plot_single_xyz(ax, t, x, y, z, title):
    ax.plot(t, x, label=x.name)
    ax.plot(t, y, label=y.name)
    ax.plot(t, z, label=z.name)

    ax.set_title(title)
    ax.set_ylabel("Position [m]")
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)


def main():

    parser = argparse.ArgumentParser()
    file_path = Path.home() / "Desktop/ros2_ws/robot_logs/kf_test.csv"
    parser.add_argument("--file", default=file_path)
    args = parser.parse_args()

    df = pd.read_csv(args.file)

    required_cols = [
        "t",

        "p_odom_x","p_odom_y","p_odom_z",
        "p_est_x","p_est_y","p_est_z",
        "v_est_x","v_est_y","v_est_z",
        "p_cL_est_x","p_cL_est_y","p_cL_est_z",
        "p_cR_est_x","p_cR_est_y","p_cR_est_z",
    ]

    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns: {missing}")

    t = df["t"].values

    # Numerical differentiation of estimated position
    px = df["p_est_x"].values
    py = df["p_est_y"].values
    pz = df["p_est_z"].values

    vx_diff = np.gradient(px, t)
    vy_diff = np.gradient(py, t)
    vz_diff = np.gradient(pz, t)

    fig, axes = plt.subplots(3,2, figsize=(16,8), sharex=True)

    # pCOM odom vs estimate
    # plot_xyz(
    #     axes[0,0],
    #     t,
    #     df["p_est_x"], df["p_est_y"], df["p_est_z"],
    #     df["p_odom_x"], df["p_odom_y"], df["p_odom_z"],
    #     "Base position: Estimate vs Odom"
    # )

    plot_single_xyz(
        axes[0,0],
        t,
        df["p_est_x"], df["p_est_y"], df["p_est_z"],
        "Base position Estimate"
    )

    # Left foot estimate
    plot_single_xyz(
        axes[1,0],
        t,
        df["p_cL_est_x"],
        df["p_cL_est_y"],
        df["p_cL_est_z"],
        "pCL Estimate"
    )

    # Right foot estimate
    plot_single_xyz(
        axes[2,0],
        t,
        df["p_cR_est_x"],
        df["p_cR_est_y"],
        df["p_cR_est_z"],
        "pCR Estimate"
    )

    plot_single_xyz(
        axes[0,1],
        t,
        df["v_est_x"],
        df["v_est_y"],
        df["v_est_z"],
        "Velocity Estimate"
    )

    axes[0,1].set_ylabel("Velocity [m/s]")
    axes[0,1].set_xlabel("Time [s]")
    axes[2,0].set_xlabel("Time [s]")


    # Velocity from differentiated position
    axes[1,1].plot(t, vx_diff, "--", label="v_diff_x")
    axes[1,1].plot(t, vy_diff, "--", label="v_diff_y")
    axes[1,1].plot(t, vz_diff, "--", label="v_diff_z")

    axes[1,1].set_title("Velocity: Differentiated Position")
    axes[1,1].set_ylabel("Velocity [m/s]")
    axes[1,1].grid(True, alpha=0.3)
    axes[1,1].legend(fontsize=9)

    # axes[1, 1].set_visible(False)
    axes[2, 1].set_visible(False)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()