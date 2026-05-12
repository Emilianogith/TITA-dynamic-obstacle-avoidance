from pathlib import Path
import argparse
import re
import numpy as np
import matplotlib.pyplot as plt

SCRIPT_DIR = Path(__file__).resolve().parent

ROBOT_LOGS = SCRIPT_DIR.parents[2] / "robot_logs"

EFF_PATH = ROBOT_LOGS / "joint_eff.txt"
JOINT_STATE_LOG = ROBOT_LOGS / "joint_state_log.txt"



def plot_joint_data(path, ylabel, title):

    data = np.loadtxt(path)

    n_steps, n_joints = data.shape

    print(f"Loaded {n_steps} timesteps, {n_joints} joints")

    time = np.arange(n_steps)

    plt.figure(figsize=(10, 6))

    for j in range(n_joints):

        plt.plot(
            time,
            data[:, j],
            label=f"Joint {j+1}"
        )

    plt.xlabel("Control cycle")
    plt.ylabel(ylabel)
    plt.title(title)

    plt.legend(loc="best")

    plt.grid(True)
    plt.tight_layout()


# ============================================================================
# Velocity parsing from joint_state_log.txt
# ============================================================================

def parse_joint_velocities(path):

    pattern = re.compile(
        r'(\S+):\s+pos:\s+[-\d.eE]+\s+vel:\s+([-\d.eE]+)'
    )

    joint_data = {}

    with open(path, "r") as f:

        for line in f:

            if line.startswith('-'):
                continue

            match = pattern.search(line)

            if match:

                joint_name = match.group(1)
                vel = float(match.group(2))

                if joint_name not in joint_data:
                    joint_data[joint_name] = []

                joint_data[joint_name].append(vel)

    return joint_data


# ============================================================================
# Velocity plotting
# ============================================================================

def plot_joint_velocities(joint_data):

    plt.figure(figsize=(12, 7))

    for joint_name, velocities in joint_data.items():

        time = np.arange(len(velocities))

        plt.plot(
            time,
            velocities,
            label=joint_name
        )

    plt.xlabel("Control cycle")
    plt.ylabel("Velocity [rad/s]")
    plt.title("Joint velocities over time")

    plt.legend(loc="best")

    plt.grid(True)
    plt.tight_layout()


# ============================================================================
# Main
# ============================================================================

def main():

    parser = argparse.ArgumentParser(
        description='Plot joint efforts / velocities'
    )

    parser.add_argument(
        '-eff',
        action='store_true',
        help='Plot efforts from robot_logs/joint_eff.txt'
    )

    parser.add_argument(
        '-vel',
        action='store_true',
        help='Plot velocities from robot_logs/joint_state_log.txt'
    )

    args = parser.parse_args()

    # ------------------------------------------------------------------------
    # Effort plot
    # ------------------------------------------------------------------------

    if args.eff:

        plot_joint_data(
            EFF_PATH,
            ylabel="Torque [Nm]",
            title="Joint torques over time"
        )

    # ------------------------------------------------------------------------
    # Velocity plot
    # ------------------------------------------------------------------------

    if args.vel:

        print(f"Loading: {JOINT_STATE_LOG}")

        joint_data = parse_joint_velocities(JOINT_STATE_LOG)

        if not joint_data:
            print("No joint data found.")
            return

        for joint_name, values in joint_data.items():
            print(f"{joint_name}: {len(values)} samples")

        plot_joint_velocities(joint_data)

    plt.show()


if __name__ == '__main__':
    main()