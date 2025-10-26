import os
import numpy as np
import matplotlib.pyplot as plt

# PLOT DES COM vs DESIRED 
# Load CSV, skip header row
log_data = np.loadtxt("/tmp/state_log_file.txt", delimiter=",", skiprows=1)

# Extract columns (based on your file)
t_ms  = log_data[:, 0]     # time in ms
com_x = log_data[:, 4]     # CoM x des
zmp_x = log_data[:, 10]     # (left wheel / ZMP x) des

# Plot
fig, axs = plt.subplots(2, 2, figsize=(12, 6))
axs[0,0].plot(t_ms, com_x, label="CoM x des", linewidth=2)
axs[0,0].plot(t_ms, zmp_x, "--", label="ZMP x des", linewidth=2)
axs[0,0].set_xlabel("Time [ms]")
axs[0,0].set_ylabel("Position [m]")
axs[0,0].set_title("CoM x des vs ZMP x des")
axs[0,0].legend()
axs[0,0].grid(True)

# PLOT COM error
com_act = log_data[:, 1:4]
com_des = log_data[:, 4:7]

# Plot
axs[0,1].plot(t_ms, com_des[:,0]-com_act[:,0], label="COM x error", linewidth=2)
axs[0,1].plot(t_ms, com_des[:,1]-com_act[:,1], label="COM y error", linewidth=2)
axs[0,1].plot(t_ms, com_des[:,2]-com_act[:,2], label="COM z error", linewidth=2)
axs[0,1].set_xlabel("Time [ms]")
axs[0,1].set_ylabel("Position [m]")
axs[0,1].set_title("COM error")
axs[0,1].legend()
axs[0,1].grid(True)

# PLOT l_wheel error
wheel_l_act = log_data[:, 7:10]
wheel_l_des = log_data[:, 10:13]

# Plot
axs[1,0].plot(t_ms, wheel_l_des[:,0]-wheel_l_act[:,0], label="LEFT wheel x error", linewidth=2)
axs[1,0].plot(t_ms, wheel_l_des[:,1]-wheel_l_act[:,1], label="LEFT wheel y error", linewidth=2)
axs[1,0].plot(t_ms, wheel_l_des[:,2]-wheel_l_act[:,2], label="LEFT wheel z error", linewidth=2)
axs[1,0].set_xlabel("Time [ms]")
axs[1,0].set_ylabel("Position [m]")
axs[1,0].set_title("LEFT wheel error")
axs[1,0].legend()
axs[1,0].grid(True)

# PLOT r_wheel error
wheel_r_act = log_data[:, 13:16]
wheel_r_des = log_data[:, 16:19]

# Plot
axs[1,1].plot(t_ms, wheel_r_des[:,0]-wheel_r_act[:,0], label="RIGHT wheel x error", linewidth=2)
axs[1,1].plot(t_ms, wheel_r_des[:,1]-wheel_r_act[:,1], label="RIGHT wheel y error", linewidth=2)
axs[1,1].plot(t_ms, wheel_r_des[:,2]-wheel_r_act[:,2], label="RIGHT wheel z error", linewidth=2)
axs[1,1].set_xlabel("Time [ms]")
axs[1,1].set_ylabel("Position [m]")
axs[1,1].set_title("RIGHT wheel error")
axs[1,1].legend()
axs[1,1].grid(True)

plt.subplots_adjust(hspace=0.4, top=0.95, bottom=0.08)

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