import re
import pandas as pd
import matplotlib.pyplot as plt
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

ROBOT_LOGS = Path('robot_logs').resolve()

WHEEL_LOG_PATH = ROBOT_LOGS / "wheel_log.txt"


with open(WHEEL_LOG_PATH, "r") as f:
    lines = f.readlines()

# Regex to capture all fields
pattern = (r'(\d+\.\d+)\s+(joint_\w+):\s+pos:\s+([-\d\.]+):\s+filtered pos:\s+([-\d\.]+)'
           r'\s+vel:\s+([-\d\.]+)\s+vel_diff:\s+([-\d\.]+)\s+filter_pos:\s+([-\d\.]+)\s+filter_vel:\s+([-\d\.]+)')

rows = []
for line in lines:
    m = re.search(pattern, line)
    if m:
        rows.append(m.groups())

# -----------------------------
# Create DataFrame
# -----------------------------
df = pd.DataFrame(rows, columns=[
    "timestamp", "joint", "position", "filtered_position", "velocity",
    "vel_diff", "filter_pos", "filter_vel"
])

# Convert numeric columns to float
numeric_cols = ["timestamp", "position", "filtered_position", "velocity",
                "vel_diff", "filter_pos", "filter_vel"]
df[numeric_cols] = df[numeric_cols].astype(float)

# Normalize timestamp to start from 0
df["timestamp"] -= df["timestamp"].min()

# -----------------------------
# Plotting
# -----------------------------
joints = df["joint"].unique()

# Create figure: 4 rows x 2 columns
fig, axes = plt.subplots(4, 2, figsize=(16, 12), sharex=False, gridspec_kw={'width_ratios':[3, 2]})

# Left column: position, velocity, filtered position, velocity difference
axes_left = axes[:, 0]

# Right column: top 2 plots for filter_pos and filter_vel
axes_right = axes[:, 1]

for joint in joints:
    jdf = df[df["joint"] == joint]

    # Left column
    axes_left[0].plot(jdf["timestamp"], jdf["position"], label=f"{joint}")
    axes_left[1].plot(jdf["timestamp"], jdf["velocity"], label=f"{joint}")
    axes_left[2].plot(jdf["timestamp"], jdf["filtered_position"], label=f"{joint}")
    axes_left[3].plot(jdf["timestamp"], jdf["vel_diff"], label=f"{joint}")

    # Left column average lines
    axes_left[0].axhline(jdf["position"].mean(), linestyle='--', alpha=0.5,
                          label=f"{joint} avg: {jdf['position'].mean():.4f}")
    axes_left[1].axhline(jdf["velocity"].mean(), linestyle='--', alpha=0.5,
                          label=f"{joint} avg: {jdf['velocity'].mean():.4f}")
    axes_left[2].axhline(jdf["filtered_position"].mean(), linestyle='--', alpha=0.5,
                          label=f"{joint} avg: {jdf['filtered_position'].mean():.4f}")
    axes_left[3].axhline(jdf["vel_diff"].mean(), linestyle='--', alpha=0.5,
                          label=f"{joint} avg: {jdf['vel_diff'].mean():.4f}")

    # Right column top 2 plots
    axes_right[0].plot(jdf["timestamp"], jdf["filter_pos"], label=f"{joint}")
    axes_right[0].axhline(jdf["filter_pos"].mean(), linestyle='--', alpha=0.5,
                           label=f"{joint} avg: {jdf['filter_pos'].mean():.4f}")

    axes_right[1].plot(jdf["timestamp"], jdf["filter_vel"], label=f"{joint}")
    axes_right[1].axhline(jdf["filter_vel"].mean(), linestyle='--', alpha=0.5,
                           label=f"{joint} avg: {jdf['filter_vel'].mean():.4f}")

# -----------------------------
# Labels
# -----------------------------
axes_left[0].set_ylabel("Position")
axes_left[1].set_ylabel("Velocity")
axes_left[2].set_ylabel("Filtered Position")
axes_left[3].set_ylabel("Velocity Diff")

axes_right[0].set_ylabel("Filter Position")
axes_right[1].set_ylabel("Filter Velocity")

# Time axis labels for bottom plots
axes_left[3].set_xlabel("Time (s)")
axes_right[0].set_xlabel("Time (s)")
axes_right[1].set_xlabel("Time (s)")

# Hide unused right column axes (3rd and 4th)
axes_right[2].set_visible(False)
axes_right[3].set_visible(False)

# -----------------------------
# Grid and legends
# -----------------------------
for ax in axes_left:
    ax.grid(True)
    ax.legend(fontsize=8)
for ax in axes_right[:2]:  # only top 2 visible
    ax.grid(True)
    ax.legend(fontsize=8)

plt.tight_layout()
plt.show()