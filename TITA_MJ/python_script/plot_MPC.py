import numpy as np
import matplotlib.pyplot as plt

# Load data from the file
data = np.loadtxt("/tmp/mpc_data/x.txt")

# Extract columns:
com_x = data[:, 0]
# com_y = data[:, 3]
# com_z = data[:, 6]
zmp_x = data[:, 2]
# zmp_y = data[:, 5]
# zmp_z = data[:, 8]

# Create a figure with 5 subplots (4 rows, 1 column)
fig, axs = plt.subplots(4, 1, figsize=(10, 12))

# Subplot 1: XY plane plot
# axs[0].plot(com_x, com_y, label='CoM', color='blue')
# axs[0].plot(zmp_x, zmp_y, label='ZMP', color='red')
# axs[0].legend()

# Subplot 2: X coordinates vs index
axs[1].plot(com_x, label='CoM X', color='blue')
axs[1].plot(zmp_x, label='ZMP X', color='red')
axs[1].legend()

# # Subplot 3: Y coordinates vs index
# axs[2].plot(com_y, label='CoM Y', color='blue')
# axs[2].plot(zmp_y, label='ZMP Y', color='red')
# axs[2].legend()

# # Subplot 4: Z coordinates vs index
# axs[3].plot(com_z, label='CoM Z', color='blue')
# axs[3].plot(zmp_z, label='ZMP Z', color='red')
# axs[3].legend()

# Adjust layout to prevent overlap
plt.tight_layout()
plt.show()
