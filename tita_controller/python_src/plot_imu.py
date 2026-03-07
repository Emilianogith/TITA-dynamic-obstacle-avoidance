#!/usr/bin/env python3

import re
import matplotlib.pyplot as plt
import numpy as np

from pathlib import Path

timestamps = []
wx, wy, wz = [], [], []
ax, ay, az = [], [], []

# Regex to parse log
timestamp_re = re.compile(r"Timestamp:\s+([0-9]+\.[0-9]+)")
ang_vel_re = re.compile(r"angular_velocity:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)")
lin_acc_re = re.compile(r"linear_acceleration:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)")

# Read log file
log_path = Path.home() / "Desktop/ros2_ws/robot_logs/imu_logs.csv"
with open(log_path, "r") as f:
    for line in f:
        ts_match = timestamp_re.search(line)
        if not ts_match:
            continue

        timestamp = float(ts_match.group(1))
        ang_match = ang_vel_re.search(line)
        acc_match = lin_acc_re.search(line)

        if ang_match and acc_match:
            timestamps.append(timestamp)

            wx.append(float(ang_match.group(1)))
            wy.append(float(ang_match.group(2)))
            wz.append(float(ang_match.group(3)))

            ax.append(float(acc_match.group(1)))
            ay.append(float(acc_match.group(2)))
            az.append(float(acc_match.group(3)))

# Normalize time
t0 = timestamps[0]
time_sec = [t - t0 for t in timestamps]

# Compute mean values
mean_wx = np.mean(wx)
mean_wy = np.mean(wy)
mean_wz = np.mean(wz)

mean_ax_val = np.mean(ax)
mean_ay_val = np.mean(ay)
mean_az_val = np.mean(az)

# Create 3x2 subplot layout
fig, axs = plt.subplots(3, 2, sharex=True, figsize=(12, 8))

# Angular velocity (left column)
for data, mean_val, ylabel, ax_row in zip(
    [wx, wy, wz], [mean_wx, mean_wy, mean_wz],
    ["ωx [rad/s]", "ωy [rad/s]", "ωz [rad/s]"],
    [0,1,2]
):
    axs[ax_row, 0].plot(time_sec, data, label=ylabel.replace(" [rad/s]",""))
    axs[ax_row, 0].axhline(mean_val, color="red", linestyle="--", label=f"avg={mean_val:.3f}")
    axs[ax_row, 0].set_ylabel(ylabel)
    axs[ax_row, 0].grid(True)
    axs[ax_row, 0].legend(fontsize=9)

axs[2, 0].set_xlabel("time [s]")

# Linear acceleration (right column)
for data, mean_val, ylabel, ax_row in zip(
    [ax, ay, az], [mean_ax_val, mean_ay_val, mean_az_val],
    ["ax [m/s²]", "ay [m/s²]", "az [m/s²]"],
    [0,1,2]
):
    axs[ax_row, 1].plot(time_sec, data, label=ylabel.replace(" [m/s²]",""))
    axs[ax_row, 1].axhline(mean_val, color="red", linestyle="--", label=f"avg={mean_val:.3f}")
    axs[ax_row, 1].set_ylabel(ylabel)
    axs[ax_row, 1].grid(True)
    axs[ax_row, 1].legend(fontsize=9)

axs[2, 1].set_xlabel("time [s]")

fig.suptitle("IMU: Angular Velocity and Linear Acceleration vs Time")
plt.tight_layout()
plt.show()

# Print numeric averages
print(f"Mean IMU angular velocity ωx: {mean_wx:.6f} rad/s")
print(f"Mean IMU angular velocity ωy: {mean_wy:.6f} rad/s")
print(f"Mean IMU angular velocity ωz: {mean_wz:.6f} rad/s")

print(f"Mean IMU acceleration along x: {mean_ax_val:.6f} m/s²")
print(f"Mean IMU acceleration along y: {mean_ay_val:.6f} m/s²")
print(f"Mean IMU acceleration along z: {mean_az_val:.6f} m/s²")