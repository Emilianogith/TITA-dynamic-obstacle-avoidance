import re
import matplotlib.pyplot as plt

timestamps = []
wx, wy, wz = [], [], []
ax, ay, az = [], [], []

timestamp_re = re.compile(r"Timestamp:\s+([0-9]+\.[0-9]+)")
ang_vel_re = re.compile(
    r"angular_velocity:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)"
)
lin_acc_re = re.compile(
    r"linear_acceleration:\s+([-\d\.]+)\s+([-\d\.]+)\s+([-\d\.]+)"
)

with open("../../imu_log.txt", "r") as f:
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

# Create 3x2 subplot layout
fig, axs = plt.subplots(3, 2, sharex=True, figsize=(12, 8))

# Angular velocity (left column)
axs[0, 0].plot(time_sec, wx)
axs[0, 0].set_ylabel("ωx [rad/s]")
axs[0, 0].grid(True)

axs[1, 0].plot(time_sec, wy)
axs[1, 0].set_ylabel("ωy [rad/s]")
axs[1, 0].grid(True)

axs[2, 0].plot(time_sec, wz)
axs[2, 0].set_ylabel("ωz [rad/s]")
axs[2, 0].set_xlabel("time [s]")
axs[2, 0].grid(True)

# Linear acceleration (right column)
axs[0, 1].plot(time_sec, ax)
axs[0, 1].set_ylabel("ax [m/s²]")
axs[0, 1].grid(True)

axs[1, 1].plot(time_sec, ay)
axs[1, 1].set_ylabel("ay [m/s²]")
axs[1, 1].grid(True)

axs[2, 1].plot(time_sec, az)
axs[2, 1].set_ylabel("az [m/s²]")
axs[2, 1].set_xlabel("time [s]")
axs[2, 1].grid(True)

fig.suptitle("IMU: Angular Velocity and Linear Acceleration vs Time")
plt.tight_layout()
plt.show()
