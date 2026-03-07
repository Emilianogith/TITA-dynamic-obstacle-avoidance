#!/usr/bin/env python3
import re
import argparse
import matplotlib.pyplot as plt
import numpy as np

from pathlib import Path

# ---------- Arguments ----------
log_path = Path.home() / "Desktop/ros2_ws/robot_logs/joint_state_log.txt"
parser = argparse.ArgumentParser(description="Plot joint position, velocity, and effort from log")
parser.add_argument("joint_name", type=str,
                    help="Joint name to plot (e.g. joint_right_leg_4)")
parser.add_argument("--file", type=str,
                    default=log_path,
                    help="Path to log file")
args = parser.parse_args()

joint_name = args.joint_name
log_file = args.file

# ---------- Data containers ----------
times = []
positions = []
velocities = []
efforts = []

# Regex dynamically uses joint name and captures effort
pattern = re.compile(
    rf"(?P<time>\d+\.\d+)\s+{re.escape(joint_name)}:\s+pos:\s+(?P<pos>[-+]?\d*\.\d+)\s+vel:\s+(?P<vel>[-+]?\d*\.\d+)\s+effort:\s+(?P<effort>[-+]?\d*\.\d+)"
)

# ---------- Parse file ----------
with open(log_file, "r") as f:
    for line in f:
        match = pattern.search(line)
        if match:
            times.append(float(match.group("time")))
            positions.append(float(match.group("pos")))
            velocities.append(float(match.group("vel")))
            efforts.append(float(match.group("effort")))

if not times:
    print(f"No data found for joint: {joint_name}")
    exit()

# Normalize time
t0 = times[0]
times = [t - t0 for t in times]

# ---------- Compute averages ----------
mean_pos = np.mean(positions)
mean_vel = np.mean(velocities)
mean_effort = np.mean(efforts)

# ---------- Plot ----------
fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 6))

# Position plot
axs[0].plot(times, positions, label="Position")
axs[0].axhline(mean_pos, color="red", linestyle="--", label=f"avg={mean_pos:.3f}")
axs[0].set_ylabel("Position [rad]")
axs[0].set_title(f"{joint_name} position")
axs[0].grid(True)
axs[0].legend(fontsize=9)

# Velocity plot
axs[1].plot(times, velocities, label="Velocity")
axs[1].axhline(mean_vel, color="red", linestyle="--", label=f"avg={mean_vel:.3f}")
axs[1].set_ylabel("Velocity [rad/s]")
axs[1].set_title(f"{joint_name} velocity")
axs[1].grid(True)
axs[1].legend(fontsize=9)

# Effort plot
axs[2].plot(times, efforts, label="Effort")
axs[2].axhline(mean_effort, color="red", linestyle="--", label=f"avg={mean_effort:.3f}")
axs[2].set_ylabel("Effort [Nm]")
axs[2].set_xlabel("Time [s]")
axs[2].set_title(f"{joint_name} effort")
axs[2].grid(True)
axs[2].legend(fontsize=9)

plt.tight_layout()
plt.show()

# ---------- Print averages ----------
print(f"{joint_name} average position: {mean_pos:.6f} rad")
print(f"{joint_name} average velocity: {mean_vel:.6f} rad/s")
print(f"{joint_name} average effort: {mean_effort:.6f} Nm")