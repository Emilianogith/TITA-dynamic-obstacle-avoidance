import numpy as np
import matplotlib.pyplot as plt

# Load data
data_com = np.loadtxt("/tmp/mpc_com.txt")
data_zmp = np.loadtxt("/tmp/mpc_zmp.txt")

# Create time vectors (just index-based, since lengths differ)
t_com = np.arange(len(data_com))
t_zmp = np.arange(len(data_zmp))

print(data_com[100])
print(data_zmp[100])

# Plot both on the same figure
plt.figure(figsize=(8, 4))
plt.plot(t_com, data_com, label='CoM x position', linewidth=2)
plt.plot(t_zmp, data_zmp, label='ZMP x position', linewidth=2, linestyle='--')

plt.xlabel('Time step')
plt.ylabel('Position [m]')
plt.title('MPC CoM vs ZMP trajectories')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()