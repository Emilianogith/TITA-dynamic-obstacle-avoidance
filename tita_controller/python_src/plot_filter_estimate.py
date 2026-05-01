#!/usr/bin/env python3
"""
Plot:
- Base position estimate
- pCL estimated
- pCR estimated
- Velocity estimate
- Velocity from differentiated position
- Continuous Roll Pitch Yaw from quaternions
"""

import pandas as pd
import matplotlib.pyplot as plt
import argparse
import numpy as np
from pathlib import Path

def quat_to_rpy(qx, qy, qz, qw):
    """Convert quaternions to roll, pitch, yaw (XYZ convention)."""
    # roll (x-axis rotation)
    roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    # pitch (y-axis rotation)
    pitch = np.arcsin(np.clip(2*(qw*qy - qz*qx), -1.0, 1.0))
    # yaw (z-axis rotation)
    yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw

def make_quaternion_continuous(q):
    """Fix quaternion sign flips for continuity."""
    q = q.copy()
    for i in range(1, len(q)):
        if np.dot(q[i-1], q[i]) < 0:
            q[i] = -q[i]
    return q

def plot_single_xyz(ax, t, x, y, z, title, ylabel=""):
    ax.plot(t, x, label="x")
    ax.plot(t, y, label="y")
    ax.plot(t, z, label="z")
    ax.set_title(title)
    if ylabel:
        ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)

def main():
    parser = argparse.ArgumentParser()
    file_path = Path.home() / "Desktop/ros2_ws/robot_logs/kf_test.csv"
    parser.add_argument("--file", default=file_path)
    args = parser.parse_args()

    df = pd.read_csv(args.file)

    t = df["t"].values

    # Compute velocities by differentiating position
    px = df["p_est_x"].values
    py = df["p_est_y"].values
    pz = df["p_est_z"].values

    vx_diff = np.gradient(px, t)
    vy_diff = np.gradient(py, t)
    vz_diff = np.gradient(pz, t)

    fig, axes = plt.subplots(3, 2, figsize=(16, 8), sharex=True)

    # Base position
    plot_single_xyz(
        axes[0,0], t,
        df["p_est_x"], df["p_est_y"], df["p_est_z"],
        "Base Position Estimate", "Position [m]"
    )

    # Left foot
    plot_single_xyz(
        axes[1,0], t,
        df["p_cL_est_x"], df["p_cL_est_y"], df["p_cL_est_z"],
        "pCL Estimate", "Position [m]"
    )

    # Right foot
    plot_single_xyz(
        axes[2,0], t,
        df["p_cR_est_x"], df["p_cR_est_y"], df["p_cR_est_z"],
        "pCR Estimate", "Position [m]"
    )
    axes[2,0].set_xlabel("Time [s]")

    # Velocity estimate
    plot_single_xyz(
        axes[0,1], t,
        df["v_est_x"], df["v_est_y"], df["v_est_z"],
        "Velocity Estimate", "Velocity [m/s]"
    )

    # Differentiated velocity
    axes[1,1].plot(t, vx_diff, "--", label="v_diff_x")
    axes[1,1].plot(t, vy_diff, "--", label="v_diff_y")
    axes[1,1].plot(t, vz_diff, "--", label="v_diff_z")
    axes[1,1].set_title("Velocity: Differentiated Position")
    axes[1,1].set_ylabel("Velocity [m/s]")
    axes[1,1].grid(True, alpha=0.3)
    axes[1,1].legend(fontsize=9)

    # --- Continuous Roll Pitch Yaw from quaternions ---
    q = df[["qx","qy","qz","qw"]].values
    q = make_quaternion_continuous(q)  # fix sign flips

    roll, pitch, yaw = quat_to_rpy(q[:,0], q[:,1], q[:,2], q[:,3])
    roll  = np.unwrap(roll)
    pitch = np.unwrap(pitch)
    yaw   = np.unwrap(yaw)

    axes[2,1].plot(t, roll,  label="roll")
    axes[2,1].plot(t, pitch, label="pitch")
    axes[2,1].plot(t, yaw,   label="yaw")
    axes[2,1].set_title("Orientation (Roll Pitch Yaw)")
    axes[2,1].set_ylabel("Angle [rad]")
    axes[2,1].set_xlabel("Time [s]")
    axes[2,1].grid(True, alpha=0.3)
    axes[2,1].legend(fontsize=9)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()