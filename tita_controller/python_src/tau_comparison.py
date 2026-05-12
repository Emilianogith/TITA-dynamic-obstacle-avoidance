#!/usr/bin/env python3
import matplotlib.pyplot as plt
import argparse
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent

ROBOT_LOGS = SCRIPT_DIR.parents[2] / "robot_logs"

TAU_COMMANDED_PATH = ROBOT_LOGS / "tau_commanded.txt"
JOINT_STATE_LOG = ROBOT_LOGS / "joint_state_log.txt"

# -----------------------------
# ARGUMENT PARSER
# -----------------------------
parser = argparse.ArgumentParser(description="Plot tau sent vs recorded for a joint")
parser.add_argument("joint_name", type=str, help="Name of the joint (e.g. joint_right_leg_4)")
args = parser.parse_args()

joint_name = args.joint_name

# -----------------------------
# JOINT INDEX MAP
# -----------------------------
joint_order = [
    "joint_left_leg_1",
    "joint_left_leg_2",
    "joint_left_leg_3",
    "joint_left_leg_4",
    "joint_right_leg_1",
    "joint_right_leg_2",
    "joint_right_leg_3",
    "joint_right_leg_4",
]

if joint_name not in joint_order:
    raise ValueError(f"Joint {joint_name} not found in known joint list")

joint_idx = joint_order.index(joint_name) + 1  # +1 because column 0 = time



# -----------------------------
# LOAD TAU SENT
# -----------------------------
t_cmd = []
tau_cmd = []

with open(TAU_COMMANDED_PATH, "r") as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) <= joint_idx:
            continue  # skip lines that don't have enough columns

        t = float(parts[0])             # first column = time
        tau = float(parts[joint_idx])   # correct joint column

        t_cmd.append(t)
        tau_cmd.append(tau)

if not t_cmd:
    raise RuntimeError(f"No tau sent data found for joint {joint_name}")

# -----------------------------
# LOAD TAU RECORDED
# -----------------------------
t_rec = []
tau_rec = []

with open(JOINT_STATE_LOG, "r") as f:
    for line in f:
        line = line.strip()
        if "--------------------------------" in line:
            continue
        if joint_name not in line:
            continue

        parts = line.split()
        if len(parts) < 3:
            continue

        t = float(parts[1])       # second timestamp = ROS time
        tau = float(parts[-1])    # effort/torque value

        t_rec.append(t)
        tau_rec.append(tau)

if not t_rec:
    raise RuntimeError(f"No tau recorded data found for joint {joint_name}")

# -----------------------------
# ALIGN TIME ORIGIN
# -----------------------------
t0 = min(t_cmd[0], t_rec[0])
t_cmd = [t - t0 for t in t_cmd]
t_rec = [t - t0 for t in t_rec]

# -----------------------------
# PLOT
# -----------------------------
plt.figure(figsize=(8, 4))
plt.plot(t_cmd, tau_cmd, label="tau sent", linestyle="--")
plt.plot(t_rec, tau_rec, label=f"tau recorded ({joint_name})")

plt.xlabel("Time [s]")
plt.ylabel("Torque [Nm]")
plt.title(f"Tau Sent vs Tau Recorded ({joint_name})")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()