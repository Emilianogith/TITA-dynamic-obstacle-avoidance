import os
import numpy as np
import matplotlib.pyplot as plt


def quat_to_rpy(qx, qy, qz, qw):
    """
    Convert quaternion (x, y, z, w) to roll, pitch, yaw.
    Returns angles in radians.
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# Load CSV
log_data = np.loadtxt("/tmp/wbc_log.txt", delimiter=",", skiprows=1)

# Column map:
# 0   time_ms
# 1   qx
# 2   qy
# 3   qz
# 4   qw
# 5   com_x
# 6   com_y
# 7   com_z
# 8   com_x_des
# 9   com_y_des
# 10  com_z_des
# 11  wheel_l_x
# 12  wheel_l_y
# 13  wheel_l_z
# 14  wheel_l_x_des
# 15  wheel_l_y_des
# 16  wheel_l_z_des
# 17  wheel_r_x
# 18  wheel_r_y
# 19  wheel_r_z
# 20  wheel_r_x_des
# 21  wheel_r_y_des
# 22  wheel_r_z_des

t_ms = log_data[:, 0]

qx = log_data[:, 1]
qy = log_data[:, 2]
qz = log_data[:, 3]
qw = log_data[:, 4]

roll, pitch, yaw = quat_to_rpy(qx, qy, qz, qw)

com_act = log_data[:, 5:8]
com_des = log_data[:, 8:11]

wheel_l_act = log_data[:, 11:14]
wheel_l_des = log_data[:, 14:17]

wheel_r_act = log_data[:, 17:20]
wheel_r_des = log_data[:, 20:23]

# -----------------------------------
# FIGURE 1: tracking errors
# -----------------------------------
fig, axs = plt.subplots(3, 1, figsize=(10, 7))

# CoM error
axs[0].plot(t_ms, com_des[:, 0] - com_act[:, 0], label="CoM x error", linewidth=2)
axs[0].plot(t_ms, com_des[:, 1] - com_act[:, 1], label="CoM y error", linewidth=2)
axs[0].plot(t_ms, com_des[:, 2] - com_act[:, 2], label="CoM z error", linewidth=2)
axs[0].set_xlabel("Time [ms]")
axs[0].set_ylabel("Position [m]")
axs[0].set_title("CoM tracking error")
axs[0].legend(loc="lower right")
axs[0].grid(True)

# Left wheel error
axs[1].plot(t_ms, wheel_l_des[:, 0] - wheel_l_act[:, 0], label="Left wheel x error", linewidth=2)
axs[1].plot(t_ms, wheel_l_des[:, 1] - wheel_l_act[:, 1], label="Left wheel y error", linewidth=2)
axs[1].plot(t_ms, wheel_l_des[:, 2] - wheel_l_act[:, 2], label="Left wheel z error", linewidth=2)
axs[1].set_xlabel("Time [ms]")
axs[1].set_ylabel("Position [m]")
axs[1].set_title("Left wheel tracking error")
axs[1].legend(loc="lower right")
axs[1].grid(True)

# Right wheel error
axs[2].plot(t_ms, wheel_r_des[:, 0] - wheel_r_act[:, 0], label="Right wheel x error", linewidth=2)
axs[2].plot(t_ms, wheel_r_des[:, 1] - wheel_r_act[:, 1], label="Right wheel y error", linewidth=2)
axs[2].plot(t_ms, wheel_r_des[:, 2] - wheel_r_act[:, 2], label="Right wheel z error", linewidth=2)
axs[2].set_xlabel("Time [ms]")
axs[2].set_ylabel("Position [m]")
axs[2].set_title("Right wheel tracking error")
axs[2].legend(loc="lower right")
axs[2].grid(True)

plt.subplots_adjust(hspace=0.6, top=0.95, bottom=0.08)

# -----------------------------------
# FIGURE 2: CoM error norm
# -----------------------------------
plan_file = "/tmp/plan/x.txt"
if os.path.exists(plan_file):
    plan_data = np.loadtxt(plan_file)

    if plan_data.ndim == 1:
        plan_data = plan_data.reshape(1, -1)

    com_plan = plan_data[:, 0:3]
    n_cur = len(com_act)

    if len(com_plan) < n_cur:
        last_row = com_plan[-1, :]
        extra = np.tile(last_row, (n_cur - len(com_plan), 1))
        com_plan_ext = np.vstack((com_plan, extra))
    else:
        com_plan_ext = com_plan[:n_cur, :]

    com_err_plan = com_plan_ext - com_act[:n_cur, :]
    com_err_norm = np.linalg.norm(com_err_plan, axis=1)
    t_common = t_ms[:n_cur]

    plt.figure(figsize=(10, 4))
    plt.plot(t_common, com_err_norm, linewidth=2)
    plt.xlabel("Time [ms]")
    plt.ylabel("Error norm [m]")
    plt.title("CoM error norm (reference - current)")
    plt.grid(True)

    plt.xlim(t_common[0], t_common[-1])   # asse x nei limiti reali
    plt.ylim(0, 0.04)                      # asse y fisso

    plt.tight_layout()


    fig_orient, ax_orient = plt.subplots(figsize=(10, 7))
    ax_orient.plot(t_ms, roll, label="Roll", linewidth=2)
    ax_orient.plot(t_ms, pitch, label="Pitch", linewidth=2)
    ax_orient.plot(t_ms, yaw, label="Yaw", linewidth=2)
    ax_orient.set_xlabel("Time [ms]")
    ax_orient.set_ylabel("Angle [rad]")
    ax_orient.set_title("Base orientation")
    ax_orient.legend(loc="lower right")
    ax_orient.grid(True)
    plt.tight_layout()



# -----------------------------------
# FIGURE 3: MPC prediction
# -----------------------------------
file_path = "/tmp/mpc_com.txt"
if os.path.exists(file_path):
    with open(file_path, "r") as f:
        first_line = f.readline().strip()

    mpc_data_com = np.loadtxt("/tmp/mpc_com.txt", skiprows=1)
    mpc_data_zmp = np.loadtxt("/tmp/mpc_zmp.txt", skiprows=1)

    t_msec = float(first_line.split(":")[1])

    mpc_t_com = np.arange(len(mpc_data_com))
    mpc_t_zmp = np.arange(len(mpc_data_zmp))

    plt.figure(figsize=(10, 7))
    plt.plot(mpc_t_com, mpc_data_com, label="CoM x position", linewidth=2)
    plt.plot(mpc_t_zmp, mpc_data_zmp, label="ZMP x position", linewidth=2, linestyle="--")
    plt.xlabel("Time step")
    plt.ylabel("Position [m]")
    plt.title(f"MPC CoM vs ZMP predicted trajectories at t_msec {t_msec}")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

plt.show()