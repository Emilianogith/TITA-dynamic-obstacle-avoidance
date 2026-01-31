import argparse
import numpy as np
import matplotlib.pyplot as plt

def plot_joint_data(path, ylabel, title, n_joints_expected=8):
    """
    Plot joint data from a text file where the first column is time.
    If a row is shorter than expected (NaN case), fill missing joints with 0.
    :param n_joints_expected: number of joints expected per row
    """
    # Read file line by line to handle missing values
    time_list = []
    joint_list = []

    with open(path, 'r') as f:
        for line in f:
            # Split line into floats
            tokens = line.strip().split()
            if len(tokens) == 0:
                continue  # skip empty lines

            # First column = time
            t = float(tokens[0])
            time_list.append(t)

            # Remaining columns = joint values
            joints = [float(x) for x in tokens[1:]]

            # If row is short (e.g., NaN happened), fill zeros
            if len(joints) < n_joints_expected:
                joints += [0.0] * (n_joints_expected - len(joints))
            
            joint_list.append(joints)

    # Convert to numpy arrays
    time = np.array(time_list)
    joint_data = np.array(joint_list)
    n_steps, n_joints = joint_data.shape
    print(f"Loaded {n_steps} timesteps, {n_joints} joints (missing values filled with 0)")

    # Plot each joint
    plt.figure(figsize=(12, 6))
    for j in range(n_joints):
        plt.plot(time, joint_data[:, j], label=f"Joint {j+1}")

    plt.xlabel("Time [s]")
    plt.ylabel(ylabel)
    plt.title(title)
    plt.legend(loc="best")
    plt.grid(True)
    plt.tight_layout()


def main():
    parser = argparse.ArgumentParser(description='Plot joint efforts / velocities')
    parser.add_argument('-eff', action='store_true', help="Plot efforts from /tmp/joint_eff.txt")
    parser.add_argument('-vel', action='store_true', help="Plot velocities from /tmp/joint_vel.txt")
    args = parser.parse_args()

    if args.eff:
        path = "/tmp/joint_eff.txt"
        ylabel = "Torque [Nm]"
        title = "Joint torques over time"
        plot_joint_data(path, ylabel, title, n_joints_expected=8)  # set number of joints here

    if args.vel:
        path = "/tmp/joint_vel.txt"
        ylabel = "Velocity [rad/s]"
        title = "Joint velocities over time"
        plot_joint_data(path, ylabel, title, n_joints_expected=8)

    plt.show()

if __name__ == '__main__':
    main()
