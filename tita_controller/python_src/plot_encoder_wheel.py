import re
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

# Read file
log_file = Path.home() / "Desktop/ros2_ws/robot_logs/wheel_log.txt"
with open(log_file, "r") as f:
    lines = f.readlines()

pattern = r'(\d+\.\d+)\s+(joint_\w+):\s+pos:\s+([-\d\.]+):\s+filtered pos:\s+([-\d\.]+)\s+vel:\s+([-\d\.]+)\s+vel_diff:\s+([-\d\.]+)'

rows = []
for line in lines:
    m = re.search(pattern, line)
    if m:
        rows.append(m.groups())

df = pd.DataFrame(rows, columns=[
    "timestamp",
    "joint",
    "position",
    "filtered_position",
    "velocity",
    "vel_diff"
])

# Convert numeric columns to float
df[["timestamp", "position", "filtered_position", "velocity", "vel_diff"]] = \
    df[["timestamp", "position", "filtered_position", "velocity", "vel_diff"]].astype(float)

# Normalize time
df["timestamp"] = df["timestamp"] - df["timestamp"].min()

joints = df["joint"].unique()

fig, axes = plt.subplots(4, 1, figsize=(10,10), sharex=True)

for joint in joints:
    jdf = df[df["joint"] == joint]

    # Plot raw data
    axes[0].plot(jdf["timestamp"], jdf["position"], label=f"{joint}")
    axes[1].plot(jdf["timestamp"], jdf["filtered_position"], label=f"{joint}")
    axes[2].plot(jdf["timestamp"], jdf["velocity"], label=f"{joint}")
    axes[3].plot(jdf["timestamp"], jdf["vel_diff"], label=f"{joint}")

    # Plot average lines with legend
    axes[0].axhline(jdf["position"].mean(), color='C0', linestyle='--', alpha=0.5,
                    label=f"{joint} avg: {jdf['position'].mean():.4f}")
    axes[1].axhline(jdf["filtered_position"].mean(), color='C1', linestyle='--', alpha=0.5,
                    label=f"{joint} avg: {jdf['filtered_position'].mean():.4f}")
    axes[2].axhline(jdf["velocity"].mean(), color='C2', linestyle='--', alpha=0.5,
                    label=f"{joint} avg: {jdf['velocity'].mean():.4f}")
    axes[3].axhline(jdf["vel_diff"].mean(), color='C3', linestyle='--', alpha=0.5,
                    label=f"{joint} avg: {jdf['vel_diff'].mean():.4f}")

axes[0].set_ylabel("Position")
axes[1].set_ylabel("Filtered Position")
axes[2].set_ylabel("Velocity")
axes[3].set_ylabel("Velocity Difference")
axes[3].set_xlabel("Time (s)")

for ax in axes:
    ax.grid(True)
    ax.legend()

plt.tight_layout()
plt.show()