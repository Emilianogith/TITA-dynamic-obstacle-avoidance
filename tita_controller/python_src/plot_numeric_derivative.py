#!/usr/bin/env python3
import argparse
import matplotlib.pyplot as plt
import re
import numpy as np

# ---------- Arguments ----------
parser = argparse.ArgumentParser(description="Plot joint position and numeric derivative")
parser.add_argument("joint_name", type=str, help="Joint name (e.g. joint_left_leg_4)")
args = parser.parse_args()

joint_name = args.joint_name
log_file = "/home/emiliano/Desktop/ros2_ws/robot_logs/numeric_derivative.txt"

# ---------- Data containers ----------
times = []
positions = []
numeric_vels = []

# Regex to capture joint_name, pos and numeric_vel
pattern = re.compile(
    rf"(?P<time>\d+\.\d+)\s+\d+\.\d+\s+{re.escape(joint_name)}:\s+pos:\s+(?P<pos>[-+]?\d*\.\d+)\s+numeric_vel:\s+(?P<numvel>[-+]?\d*\.\d+)"
)

# ---------- Parse file ----------
with open(log_file, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            times.append(float(match.group("time")))
            positions.append(float(match.group("pos")))
            numeric_vels.append(float(match.group("numvel")))

if not times:
    print(f"No data found for joint: {joint_name}")
    exit()

# Normalize time
t0 = times[0]
times = [t - t0 for t in times]

# Compute averages
avg_pos = np.mean(positions)
avg_numvel = np.mean(numeric_vels)

# ---------- Plot ----------
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(8, 6))

# Position
axs[0].plot(times, positions, label="Position")
axs[0].axhline(avg_pos, color='red', linestyle='--', label=f"Average = {avg_pos:.3f}")
axs[0].set_ylabel("Position [rad]")
axs[0].set_title(f"{joint_name} Position vs Time")
axs[0].grid(True)
axs[0].legend()

# Numeric derivative
axs[1].plot(times, numeric_vels, label="Numeric Derivative", color="green")
axs[1].axhline(avg_numvel, color='red', linestyle='--', label=f"Average = {avg_numvel:.3f}")
axs[1].set_ylabel("Numeric Velocity [rad/s]")
axs[1].set_xlabel("Time [s]")
axs[1].set_title(f"{joint_name} Numeric Derivative vs Time")
axs[1].grid(True)
axs[1].legend()

plt.tight_layout()
plt.show()