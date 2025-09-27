import numpy as np
import matplotlib.pyplot as plt

# Load the data from file
# Each row = one timestep, each column = one joint
data = np.loadtxt("/tmp/joint_eff.txt")

n_steps, n_joints = data.shape
print(f"Loaded {n_steps} timesteps, {n_joints} joints")

# Time axis (just indices; scale if you know dt)
time = np.arange(n_steps)

# Plot each joint torque
plt.figure(figsize=(10, 6))
for j in range(n_joints):
    plt.plot(time, data[:, j], label=f"Joint {j+1}")

plt.xlabel("Time step")
plt.ylabel("Torque (tau)")
plt.title("Joint torques over time")
plt.legend(loc="best")
plt.grid(True)
plt.tight_layout()
plt.show()
