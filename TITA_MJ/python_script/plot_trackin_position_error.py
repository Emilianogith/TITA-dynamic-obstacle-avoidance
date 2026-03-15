import os
import numpy as np
import matplotlib.pyplot as plt

# PLOT DES COM vs DESIRED 
# Load CSV, skip header row
log_data = np.loadtxt("/tmp/wbc_log.txt", delimiter=",", skiprows=1)

# Extract columns (based on your file)
t_ms  = log_data[:, 0]     # time in ms
com_x = log_data[:, 4]     # CoM x des
com_y = log_data[:, 5] 
left_wheel_x = log_data[:, 10]  
left_wheel_y = log_data[:, 11]  
right_wheel_x = log_data[:, 16]
right_wheel_y = log_data[:, 17]

# Plot
fig, axs = plt.subplots(3, 1, figsize=(10, 7))
# axs[0,0].plot(com_x, com_y, label="CoM x des", linewidth=2)
# axs[0,0].plot(left_wheel_x, left_wheel_y, "--", label="left wheel x des", linewidth=2)
# axs[0,0].plot(right_wheel_x, right_wheel_y,  "--", label="right wheel x des", linewidth=2)
# axs[0,0].set_xlabel("Time [ms]")
# axs[0,0].set_ylabel("Position [m]")
# axs[0,0].set_title("CoM x des vs ZMP x des")
# axs[0,0].legend()
# axs[0,0].grid(True)
# fig.delaxes(axs[0,0])  

# PLOT COM error
com_act = log_data[:, 1:4]
com_des = log_data[:, 4:7]

# Plot
axs[0].plot(t_ms, com_des[:,0]-com_act[:,0], label="COM x error", linewidth=2)
axs[0].plot(t_ms, com_des[:,1]-com_act[:,1], label="COM y error", linewidth=2)
axs[0].plot(t_ms, com_des[:,2]-com_act[:,2], label="COM z error", linewidth=2)
axs[0].set_xlabel("Time [ms]")
axs[0].set_ylabel("Position [m]")
axs[0].set_title("COM error")
axs[0].legend(loc='upper right', bbox_to_anchor=(1.02, 0.5))
axs[0].grid(True)

# PLOT l_wheel error
wheel_l_act = log_data[:, 7:10]
wheel_l_des = log_data[:, 10:13]

# Plot
axs[1].plot(t_ms, wheel_l_des[:,0]-wheel_l_act[:,0], label="LEFT wheel x error", linewidth=2)
axs[1].plot(t_ms, wheel_l_des[:,1]-wheel_l_act[:,1], label="LEFT wheel y error", linewidth=2)
axs[1].plot(t_ms, wheel_l_des[:,2]-wheel_l_act[:,2], label="LEFT wheel z error", linewidth=2)
axs[1].set_xlabel("Time [ms]")
axs[1].set_ylabel("Position [m]")
axs[1].set_title("LEFT wheel error")
axs[1].legend(loc='upper right')
axs[1].grid(True)

# PLOT r_wheel error
wheel_r_act = log_data[:, 13:16]
wheel_r_des = log_data[:, 16:19]

# Plot
axs[2].plot(t_ms, wheel_r_des[:,0]-wheel_r_act[:,0], label="RIGHT wheel x error", linewidth=2)
axs[2].plot(t_ms, wheel_r_des[:,1]-wheel_r_act[:,1], label="RIGHT wheel y error", linewidth=2)
axs[2].plot(t_ms, wheel_r_des[:,2]-wheel_r_act[:,2], label="RIGHT wheel z error", linewidth=2)
axs[2].set_xlabel("Time [ms]")
axs[2].set_ylabel("Position [m]")
axs[2].set_title("RIGHT wheel error")
axs[2].legend(loc='upper right')
axs[2].grid(True)

plt.subplots_adjust(hspace=0.6, top=0.93, bottom=0.08)

# PLOT REFERENCE COM PLAN ERROR = reference - current
plan_file = "/tmp/plan/x.txt"
if os.path.exists(plan_file):
    plan_data = np.loadtxt(plan_file)

    # Handle case with single row
    if plan_data.ndim == 1:
        plan_data = plan_data.reshape(1, -1)

    com_plan = plan_data[:, 0:3]   # reference CoM from x.txt
    com_act = log_data[:, 1:4]     # current CoM from wbc_log

    n_cur = len(com_act)

    # Extend reference with its last row if shorter than current trajectory
    if len(com_plan) < n_cur:
        last_row = com_plan[-1, :]
        extra = np.tile(last_row, (n_cur - len(com_plan), 1))
        com_plan_ext = np.vstack((com_plan, extra))
    else:
        com_plan_ext = com_plan[:n_cur, :]

    com_err_plan = com_plan_ext - com_act[:n_cur, :]
    t_common = t_ms[:n_cur]

    fig_plan, axs_plan = plt.subplots(3, 1, figsize=(10, 7))

    axs_plan[0].plot(t_common, com_err_plan[:, 0], linewidth=2)
    axs_plan[0].set_xlabel("Time [ms]")
    axs_plan[0].set_ylabel("Position [m]")
    axs_plan[0].set_title("CoM x error (reference - current)")
    axs_plan[0].grid(True)

    axs_plan[1].plot(t_common, com_err_plan[:, 1], linewidth=2)
    axs_plan[1].set_xlabel("Time [ms]")
    axs_plan[1].set_ylabel("Position [m]")
    axs_plan[1].set_title("CoM y error (reference - current)")
    axs_plan[1].grid(True)

    axs_plan[2].plot(t_common, com_err_plan[:, 2], linewidth=2)
    axs_plan[2].set_xlabel("Time [ms]")
    axs_plan[2].set_ylabel("Position [m]")
    axs_plan[2].set_title("CoM z error (reference - current)")
    axs_plan[2].grid(True)

    plt.subplots_adjust(hspace=0.6, top=0.93, bottom=0.08)

#######
# PLOT MPC REDICTION AT A GIVEN TIMESTAMP
# Read first line manually
file_path = "/tmp/mpc_com.txt"
if os.path.exists(file_path):
    with open("/tmp/mpc_com.txt", "r") as f:
        first_line = f.readline().strip()

    # Load data
    mpc_data_com = np.loadtxt("/tmp/mpc_com.txt", skiprows=1)
    mpc_data_zmp = np.loadtxt("/tmp/mpc_zmp.txt", skiprows=1)

    # Extract the numeric part
    t_msec = float(first_line.split(":")[1])

    # Create time vectors (just index-based, since lengths differ)
    mpc_t_com = np.arange(len(mpc_data_com))
    mpc_t_zmp = np.arange(len(mpc_data_zmp))

    # Plot both on the same figure
    plt.figure(figsize=(8, 4))
    plt.plot(mpc_t_com, mpc_data_com, label='CoM x position', linewidth=2)
    plt.plot(mpc_t_zmp, mpc_data_zmp, label='ZMP x position', linewidth=2, linestyle='--')

    plt.xlabel('Time step')
    plt.ylabel('Position [m]')
    plt.title(f'MPC CoM vs ZMP predicted trajectories at t_msec {t_msec}')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

plt.show()