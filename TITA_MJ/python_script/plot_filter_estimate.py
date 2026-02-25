#!/usr/bin/env python3
"""
Read /tmp/kf_test.csv and plot:
1) pCOM true vs estimated
2) p_cL true vs estimated
3) p_cR true vs estimated
4) base velocity true vs estimated

If '-err' is passed, also open another figure with 4 subplots showing estimation errors:
- pCOM error
- p_cL error
- p_cR error
- base velocity error
"""

import argparse
import pandas as pd
import matplotlib.pyplot as plt


def plot_xyz(ax, t, x_true, y_true, z_true, x_est, y_est, z_est, title, ylabel):
    ax.plot(t, x_true, label=f"{x_true.name}")
    ax.plot(t, x_est, "--", label=f"{x_est.name}")
    ax.plot(t, y_true, label=f"{y_true.name}")
    ax.plot(t, y_est, "--", label=f"{y_est.name}")
    ax.plot(t, z_true, label=f"{z_true.name}")
    ax.plot(t, z_est, "--", label=f"{z_est.name}")
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", ncol=2, fontsize=9)


def plot_err_xyz(ax, t, ex, ey, ez, title, ylabel):
    ax.plot(t, ex, label=f"{ex.name}")
    ax.plot(t, ey, label=f"{ey.name}")
    ax.plot(t, ez, label=f"{ez.name}")
    ax.axhline(0.0, linestyle="--", linewidth=1)
    ax.set_title(title)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", ncol=1, fontsize=9)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-err", action="store_true", help="Plot estimation errors in a second window")
    parser.add_argument("--file", default="/tmp/kf_test.csv", help="Path to CSV file")
    args = parser.parse_args()

    file_path = args.file

    # Read CSV
    df = pd.read_csv(file_path)

    # Required columns for main plots
    required_cols = [
        "t",
        "p_true_x", "p_true_y", "p_true_z",
        "p_est_x", "p_est_y", "p_est_z",
        "p_cL_true_x", "p_cL_true_y", "p_cL_true_z",
        "p_cL_est_x", "p_cL_est_y", "p_cL_est_z",
        "p_cR_true_x", "p_cR_true_y", "p_cR_true_z",
        "p_cR_est_x", "p_cR_est_y", "p_cR_est_z",
        "v_true_x", "v_true_y", "v_true_z",
        "v_est_x", "v_est_y", "v_est_z",
    ]
    missing = [c for c in required_cols if c not in df.columns]
    if missing:
        raise ValueError(f"Missing columns in file: {missing}")

    t = df["t"].values

    # ---- Main figure ----
    fig, axes = plt.subplots(3, 2, figsize=(14, 7), sharex=True)

    # COM position
    plot_xyz(
        axes[0, 0], t,
        df["p_true_x"], df["p_true_y"], df["p_true_z"],
        df["p_est_x"],  df["p_est_y"],  df["p_est_z"],
        "COM Position: True vs Estimated", "Position [m]"
    )

    # Left contact position
    plot_xyz(
        axes[1, 0], t,
        df["p_cL_true_x"], df["p_cL_true_y"], df["p_cL_true_z"],
        df["p_cL_est_x"],  df["p_cL_est_y"],  df["p_cL_est_z"],
        "Left Contact Position: True vs Estimated", "Position [m]"
    )

    # Right contact position
    plot_xyz(
        axes[2, 0], t,
        df["p_cR_true_x"], df["p_cR_true_y"], df["p_cR_true_z"],
        df["p_cR_est_x"],  df["p_cR_est_y"],  df["p_cR_est_z"],
        "Right Contact Position: True vs Estimated", "Position [m]"
    )
    axes[2, 0].set_xlabel("Time [s]")

    # Base velocity
    plot_xyz(
        axes[0, 1], t,
        df["v_true_x"], df["v_true_y"], df["v_true_z"],
        df["v_est_x"],  df["v_est_y"],  df["v_est_z"],
        "Base velocity: True vs Estimated", "Velocity [m/s]"
    )

    axes[1, 1].set_visible(False)
    axes[2, 1].set_visible(False)

    plt.tight_layout()

    # ---- Error figure (optional) ----
    if args.err:
        # Compute errors = est - true
        df["e_p_x"] = df["p_est_x"] - df["p_true_x"]
        df["e_p_y"] = df["p_est_y"] - df["p_true_y"]
        df["e_p_z"] = df["p_est_z"] - df["p_true_z"]

        df["e_p_cL_x"] = df["p_cL_est_x"] - df["p_cL_true_x"]
        df["e_p_cL_y"] = df["p_cL_est_y"] - df["p_cL_true_y"]
        df["e_p_cL_z"] = df["p_cL_est_z"] - df["p_cL_true_z"]

        df["e_p_cR_x"] = df["p_cR_est_x"] - df["p_cR_true_x"]
        df["e_p_cR_y"] = df["p_cR_est_y"] - df["p_cR_true_y"]
        df["e_p_cR_z"] = df["p_cR_est_z"] - df["p_cR_true_z"]

        df["e_v_x"] = df["v_est_x"] - df["v_true_x"]
        df["e_v_y"] = df["v_est_y"] - df["v_true_y"]
        df["e_v_z"] = df["v_est_z"] - df["v_true_z"]

        fig_err, ax_err = plt.subplots(2, 2, figsize=(12, 6), sharex=True)

        plot_err_xyz(
            ax_err[0, 0], t,
            df["e_p_x"], df["e_p_y"], df["e_p_z"],
            "COM Position Error (est - true)", "Error [m]"
        )

        plot_err_xyz(
            ax_err[0, 1], t,
            df["e_p_cL_x"], df["e_p_cL_y"], df["e_p_cL_z"],
            "Left Contact Position Error (est - true)", "Error [m]"
        )

        plot_err_xyz(
            ax_err[1, 0], t,
            df["e_p_cR_x"], df["e_p_cR_y"], df["e_p_cR_z"],
            "Right Contact Position Error (est - true)", "Error [m]"
        )

        plot_err_xyz(
            ax_err[1, 1], t,
            df["e_v_x"], df["e_v_y"], df["e_v_z"],
            "Base Velocity Error (est - true)", "Error [m/s]"
        )

        ax_err[1, 0].set_xlabel("Time [s]")
        ax_err[1, 1].set_xlabel("Time [s]")

        plt.tight_layout()

    plt.show()


if __name__ == "__main__":
    main()