import numpy as np
import matplotlib.pyplot as plt

# Load data
data_com = np.loadtxt("/tmp/com.txt")
data_zmp = np.loadtxt("/tmp/zmp.txt")

# Create time vectors (just index-based, since lengths differ)
t_com = np.arange(len(data_com))
t_zmp = np.arange(len(data_zmp))

print(data_com[100])
print(data_zmp[100])

# Plot both on the same figure
plt.figure(figsize=(8, 4))
plt.plot(t_com, data_com[:,0], label='CoM x position', linewidth=2)
plt.plot(t_zmp, data_zmp, label='ZMP x position', linewidth=2, linestyle='--')

plt.xlabel('Time step')
plt.ylabel('Position [m]')
plt.title('CoM vs ZMP actual trajectories')
plt.legend()
plt.grid(True)
# plt.tight_layout()
# plt.show()




#######
# Read first line manually
with open("/tmp/mpc_com.txt", "r") as f:
    first_line = f.readline().strip()

# Extract the numeric part
t_msec = float(first_line.split(":")[1])

# Load data
mpc_data_com = np.loadtxt("/tmp/mpc_com.txt", skiprows=1)
mpc_data_zmp = np.loadtxt("/tmp/mpc_zmp.txt", skiprows=1)

# Create time vectors (just index-based, since lengths differ)
mpc_t_com = np.arange(len(mpc_data_com))
mpc_t_zmp = np.arange(len(mpc_data_zmp))

print(mpc_data_com[3])
print(mpc_data_zmp[3])

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