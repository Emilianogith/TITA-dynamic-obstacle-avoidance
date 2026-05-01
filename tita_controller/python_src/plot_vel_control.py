import re
import sys
import matplotlib.pyplot as plt
from collections import defaultdict

# --------- CHECK ARGUMENT ---------
if len(sys.argv) != 2:
    print("Usage: python3 script.py <joint_name>")
    sys.exit(1)

joint_to_plot = sys.argv[1]

tau_file = "/tmp/tau_commanded.txt"
state_file = "/home/emiliano/Desktop/ros2_ws/robot_logs/joint_state_log.txt"

# --------- GET FIRST JOINT STATE TIMESTAMP ---------
t0_state = None

with open(state_file, "r") as f:
    for line in f:
        match = re.match(r"\d+\.\d+\s+(\d+\.\d+)", line)
        if match:
            t0_state = float(match.group(1))
            break

if t0_state is None:
    raise RuntimeError("Could not find timestamp in joint_state file")


# --------- PARSE TAU FILE ---------
tau_data = defaultdict(lambda: {
    "t": [],
    "vel": [],
    "target": [],
    "tau_cmd": []
})

with open(tau_file, "r") as f:
    current_time = None

    for line in f:
        ts_match = re.match(r"^(\d+\.\d+)", line)
        if ts_match:
            current_time = float(ts_match.group(1))

        match = re.search(
            r"(joint_\w+).*effort commanded:\s+([-\d\.eE]+).*vel_target:\s+([-\d\.eE]+)\s+vel_feedback:\s+([-\d\.eE]+)",
            line,
        )

        if match and current_time is not None:
            joint = match.group(1)

            if joint != joint_to_plot:
                continue

            tau_cmd = float(match.group(2))
            vel_target = float(match.group(3))
            vel_feedback = float(match.group(4))

            t_aligned = current_time - t0_state

            tau_data[joint]["t"].append(t_aligned)
            tau_data[joint]["tau_cmd"].append(tau_cmd)
            tau_data[joint]["vel"].append(vel_feedback)
            tau_data[joint]["target"].append(vel_target)


# --------- PARSE JOINT STATE ---------
state_data = defaultdict(lambda: {"t": [], "vel": [], "tau": []})

with open(state_file, "r") as f:
    for line in f:
        match = re.search(
            r"\d+\.\d+\s+(\d+\.\d+)\s+(joint_\w+):.*vel:\s+([-\d\.eE]+)\s+effort:\s+([-\d\.eE]+)",
            line,
        )
        if match:
            joint = match.group(2)

            if joint != joint_to_plot:
                continue

            t = float(match.group(1))
            vel = float(match.group(3))
            tau = float(match.group(4))

            state_data[joint]["t"].append(t - t0_state)
            state_data[joint]["vel"].append(vel)
            state_data[joint]["tau"].append(tau)


# --------- PLOT ---------
fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 6))

# ===== TOP: VELOCITY =====
if joint_to_plot in state_data:
    axs[0].plot(
        state_data[joint_to_plot]["t"],
        state_data[joint_to_plot]["vel"],
        label="measured vel",
    )

if joint_to_plot in tau_data:
    axs[0].plot(
        tau_data[joint_to_plot]["t"],
        tau_data[joint_to_plot]["vel"],
        label="vel_feedback",
    )

    axs[0].plot(
        tau_data[joint_to_plot]["t"],
        tau_data[joint_to_plot]["target"],
        linestyle="--",
        label="vel_target",
    )

axs[0].set_ylabel("velocity")
axs[0].set_title(joint_to_plot)
axs[0].legend()
axs[0].grid()

# ===== BOTTOM: TORQUE =====
if joint_to_plot in tau_data:
    axs[1].plot(
        tau_data[joint_to_plot]["t"],
        tau_data[joint_to_plot]["tau_cmd"],
        label="tau commanded",
    )

if joint_to_plot in state_data:
    axs[1].plot(
        state_data[joint_to_plot]["t"],
        state_data[joint_to_plot]["tau"],
        label="tau executed (measured)",
    )

axs[1].set_xlabel("time [s]")
axs[1].set_ylabel("torque")
axs[1].legend()
axs[1].grid()

plt.show()