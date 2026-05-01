#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import argparse
import numpy as np
from pathlib import Path



angular_odometry = False

def plot_xyz(ax, t, x, y, z, title, ylabel):
    ax.plot(t, x, label=x.name)
    ax.plot(t, y, label=y.name)
    ax.plot(t, z, label=z.name)

    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=9)


def main():
    parser = argparse.ArgumentParser()
    log_path = Path.home() / "Desktop/ros2_ws/robot_logs/robot_odom.csv"
    parser.add_argument("--file", default=log_path)
    args = parser.parse_args()

    # Load CSV
    df = pd.read_csv(args.file)
    df.columns = df.columns.str.strip()  # clean column names

    if (not angular_odometry):
        required_cols = [
            "t",
            "p_odom_x","p_odom_y","p_odom_z",
            "v_odom_x","v_odom_y","v_odom_z",
            "p_cL_odom_x","p_cL_odom_y","p_cL_odom_z",
            "p_cR_odom_x","p_cR_odom_y","p_cR_odom_z",
            "w_l_odom","w_r_odom"
        ]
    else:
        required_cols = [
            "t",
            "p_odom_x","p_odom_y","p_odom_z",
            "v_odom_x","v_odom_y","v_odom_z",
            "omega_odom_x","omega_odom_y","omega_odom_z",
            "p_cL_odom_x","p_cL_odom_y","p_cL_odom_z",
            "p_cR_odom_x","p_cR_odom_y","p_cR_odom_z",
            "w_l_odom","w_r_odom"
        ]



    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns: {missing}")

    t = df["t"].values

    # Numerical differentiation of base position
    px = df["p_odom_x"].values
    py = df["p_odom_y"].values
    pz = df["p_odom_z"].values
    vx_num = np.gradient(px, t)
    vy_num = np.gradient(py, t)
    vz_num = np.gradient(pz, t)

    fig, axes = plt.subplots(4, 2, figsize=(14,10), sharex=True)

    # ---------------- Base position ----------------
    plot_xyz(
        axes[0,0], t,
        df["p_odom_x"], df["p_odom_y"], df["p_odom_z"],
        "Base Position (Odom)",
        "Position [m]"
    )

    # ---------------- Base velocity ----------------
    plot_xyz(
        axes[0,1], t,
        df["v_odom_x"], df["v_odom_y"], df["v_odom_z"],
        "Base Velocity (Odom)",
        "Velocity [m/s]"
    )

    if (angular_odometry):
    # ---------------- Base angular velocity ----------------
        plot_xyz(
            axes[1,0], t,
            df["omega_odom_x"], df["omega_odom_y"], df["omega_odom_z"],
            "Base Angular Velocity in base frame (Odom)",
            "Angular Velocity [rad/s]"
        )

    # ---------------- Left contact ----------------
    plot_xyz(
        axes[1,1], t,
        df["p_cL_odom_x"], df["p_cL_odom_y"], df["p_cL_odom_z"],
        "Left Contact Position (Odom)",
        "Position [m]"
    )

    # ---------------- Right contact ----------------
    plot_xyz(
        axes[2,0], t,
        df["p_cR_odom_x"], df["p_cR_odom_y"], df["p_cR_odom_z"],
        "Right Contact Position (Odom)",
        "Position [m]"
    )

    # ---------------- Differentiated velocity ----------------
    axes[2,1].plot(t, vx_num, "--", label="v_diff_x")
    axes[2,1].plot(t, vy_num, "--", label="v_diff_y")
    axes[2,1].plot(t, vz_num, "--", label="v_diff_z")
    axes[2,1].set_title("Base Velocity (Differentiated Position)")
    axes[2,1].set_ylabel("Velocity [m/s]")
    axes[2,1].grid(True, alpha=0.3)
    axes[2,1].legend(fontsize=9)

    # ---------------- Wheel angular velocities ----------------
    axes[3,0].plot(t, df["w_l_odom"], label="w_l_odom")
    axes[3,0].plot(t, df["w_r_odom"], label="w_r_odom")
    axes[3,0].set_title("Wheel Angular Velocity")
    axes[3,0].set_ylabel("Angular Velocity [rad/s]")
    axes[3,0].grid(True, alpha=0.3)
    w_l_avg = df["w_l_odom"].mean()
    w_r_avg = df["w_r_odom"].mean()
    axes[3,0].axhline(w_l_avg, linestyle="--", label=f"avg w_l={w_l_avg:.4f}")
    axes[3,0].axhline(w_r_avg, linestyle="--", label=f"avg w_r={w_r_avg:.4f}")
    axes[3,0].legend(fontsize=9)
    print(f"Average rolling angular velocity (left wheel):  {w_l_avg:.4f} rad/s")
    print(f"Average rolling angular velocity (right wheel): {w_r_avg:.4f} rad/s")


    # ---------------- X labels ----------------
    axes[3,0].set_xlabel("Time [s]")
    axes[3,1].set_xlabel("Time [s]")

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()