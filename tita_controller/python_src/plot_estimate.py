import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# ======================
# Load CSV
# ======================
csv_file = "../../kf_test.csv"   # change path if needed
df = pd.read_csv(csv_file)

t = df["t"] - df["t"].iloc[0]  # start time at 0

# ======================
# Helper function
# ======================
def plot_xyz(t, x, y, z, title, ylabel):
    plt.figure(figsize=(10, 4))
    plt.plot(t, x, label="x")
    plt.plot(t, y, label="y")
    plt.plot(t, z, label="z")
    plt.xlabel("Time [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend()
    plt.grid()
    plt.tight_layout()

# ======================
# 1️⃣ odom vs Estimated Position
# ======================
plt.figure(figsize=(10, 5))
plt.plot(t, df["p_odom_x"], "--", label="p_odom_x")
plt.plot(t, df["p_est_x"], label="p_est_x")

plt.plot(t, df["p_odom_y"], "--", label="p_odom_y")
plt.plot(t, df["p_est_y"], label="p_est_y")

plt.plot(t, df["p_odom_z"], "--", label="p_odom_z")
plt.plot(t, df["p_est_z"], label="p_est_z")

plt.xlabel("Time [s]")
plt.ylabel("Position [m]")
plt.title("odom vs Estimated Base Position")
plt.legend()
plt.grid()
plt.tight_layout()

# ======================
# 2️⃣ Position Error
# ======================
pos_err = np.sqrt(
    (df["p_est_x"] - df["p_odom_x"])**2 +
    (df["p_est_y"] - df["p_odom_y"])**2 +
    (df["p_est_z"] - df["p_odom_z"])**2
)

plt.figure(figsize=(10, 4))
plt.plot(t, pos_err)
plt.xlabel("Time [s]")
plt.ylabel("Position Error [m]")
plt.title("Base Position Error Norm")
plt.grid()
plt.tight_layout()

# ======================
# 3️⃣ Estimated Velocity
# ======================
plot_xyz(
    t,
    df["v_est_x"],
    df["v_est_y"],
    df["v_est_z"],
    "Estimated Base Velocity",
    "Velocity [m/s]"
)

# ======================
# 4️⃣ Contact Point p_cL (Left)
# ======================
plot_xyz(
    t,
    df["p_cL_est_x"],
    df["p_cL_est_y"],
    df["p_cL_est_z"],
    "Left Contact Point Position (p_cL)",
    "Position [m]"
)

# ======================
# 5️⃣ Contact Point p_cR (Right)
# ======================
plot_xyz(
    t,
    df["p_cR_est_x"],
    df["p_cR_est_y"],
    df["p_cR_est_z"],
    "Right Contact Point Position (p_cR)",
    "Position [m]"
)

# ======================
# 6️⃣ XY Trajectories (Top View)
# ======================
plt.figure(figsize=(6, 6))
plt.plot(df["p_odom_x"], df["p_odom_y"], "--", label="Base odom")
plt.plot(df["p_est_x"], df["p_est_y"], label="Base Estimated")
plt.plot(df["p_cL_est_x"], df["p_cL_est_y"], label="p_cL")
plt.plot(df["p_cR_est_x"], df["p_cR_est_y"], label="p_cR")

plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("XY Trajectories")
plt.axis("equal")
plt.legend()
plt.grid()
plt.tight_layout()

# ======================
# Show all plots
# ======================
plt.show()
