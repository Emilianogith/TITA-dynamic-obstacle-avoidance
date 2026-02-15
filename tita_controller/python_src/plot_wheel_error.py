import re
import argparse
import matplotlib.pyplot as plt

# ---------- Arguments ----------
parser = argparse.ArgumentParser(description="Plot joint position, velocity, and effort from log")
parser.add_argument("joint_name", type=str,
                    help="Joint name to plot (e.g. joint_right_leg_4)")
parser.add_argument("--file", type=str,
                    default="/home/emiliano/Desktop/ros2_ws/src/joint_state_log.txt",
                    help="Path to log file")

args = parser.parse_args()

joint_name = args.joint_name
log_file = args.file

# ---------- Data containers ----------
times = []
positions = []
velocities = []
efforts = []  # new container for effort

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
            efforts.append(float(match.group("effort")))  # collect effort

if not times:
    print(f"No data found for joint: {joint_name}")
    exit()

# Normalize time
t0 = times[0]
times = [t - t0 for t in times]

# ---------- Plot ----------
fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 6))  # 3 subplots now

# Position plot
axs[0].plot(times, positions)
axs[0].set_ylabel("Position [rad]")
axs[0].set_title(f"{joint_name} position")
axs[0].grid(True)

# Velocity plot
axs[1].plot(times, velocities)
axs[1].set_ylabel("Velocity [rad/s]")
axs[1].set_title(f"{joint_name} velocity")
axs[1].grid(True)

# Effort plot
axs[2].plot(times, efforts)
axs[2].set_ylabel("Effort [Nm]")
axs[2].set_xlabel("Time [s]")
axs[2].set_title(f"{joint_name} effort")
axs[2].grid(True)

plt.tight_layout()
plt.show()
