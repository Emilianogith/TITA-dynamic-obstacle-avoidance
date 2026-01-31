import re
import matplotlib.pyplot as plt

timestamps = []
xs, ys, zs = [], [], []
wx, wy, wz = [], [], []

timestamp_re = re.compile(r"Timestamp:\s+([0-9]+\.[0-9]+)")
position_re = re.compile(r"Position:\s+x=([-\d\.]+),\s+y=([-\d\.]+),\s+z=([-\d\.]+)")
angular_vel_re = re.compile(
    r"Angular Velocity:\s+x=([-\d\.]+),\s+y=([-\d\.]+),\s+z=([-\d\.]+)"
)

current_timestamp = None
current_pos = None

with open("../../odom.txt", "r") as f:
    for line in f:
        ts_match = timestamp_re.search(line)
        if ts_match:
            current_timestamp = float(ts_match.group(1))
            continue

        pos_match = position_re.search(line)
        if pos_match:
            current_pos = (
                float(pos_match.group(1)),
                float(pos_match.group(2)),
                float(pos_match.group(3))
            )
            continue

        ang_match = angular_vel_re.search(line)
        if ang_match and current_timestamp is not None and current_pos is not None:
            timestamps.append(current_timestamp)

            xs.append(current_pos[0])
            ys.append(current_pos[1])
            zs.append(current_pos[2])

            wx.append(float(ang_match.group(1)))
            wy.append(float(ang_match.group(2)))
            wz.append(float(ang_match.group(3)))

            current_timestamp = None
            current_pos = None

# Normalize time
t0 = timestamps[0]
time_sec = [t - t0 for t in timestamps]

# Create 3x2 subplot layout
fig, axs = plt.subplots(3, 2, sharex=True, figsize=(12, 8))

# Position plots (left column)
axs[0, 0].plot(time_sec, xs)
axs[0, 0].set_ylabel("x [m]")
axs[0, 0].grid(True)

axs[1, 0].plot(time_sec, ys)
axs[1, 0].set_ylabel("y [m]")
axs[1, 0].grid(True)

axs[2, 0].plot(time_sec, zs)
axs[2, 0].set_ylabel("z [m]")
axs[2, 0].set_xlabel("time [s]")
axs[2, 0].grid(True)

# Angular velocity plots (right column)
axs[0, 1].plot(time_sec, wx)
axs[0, 1].set_ylabel("ωx [rad/s]")
axs[0, 1].grid(True)

axs[1, 1].plot(time_sec, wy)
axs[1, 1].set_ylabel("ωy [rad/s]")
axs[1, 1].grid(True)

axs[2, 1].plot(time_sec, wz)
axs[2, 1].set_ylabel("ωz [rad/s]")
axs[2, 1].set_xlabel("time [s]")
axs[2, 1].grid(True)

fig.suptitle("Robot Odometry: Position and Angular Velocity vs Time")
plt.tight_layout()
plt.show()
