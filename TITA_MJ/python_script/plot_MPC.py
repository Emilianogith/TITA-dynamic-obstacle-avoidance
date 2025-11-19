import numpy as np
import matplotlib.pyplot as plt
import argparse

# Load data from the file



def plot_mpc(t_msec):
    data = np.loadtxt(f"/tmp/mpc_data/{t_msec:.6f}/x.txt")

    data_u = np.loadtxt(f"/tmp/mpc_data/{t_msec:.6f}/u.txt")
    # fl_x = data_u[:,6]
    # fr_x = data_u[:,9]
    # fl_z = data_u[:,8]
    # fr_z = data_u[:,11]

    fz_ = data_u[:,-1]

    # Extract columns:
    # com_x = data[:, 0]
    # # com_y = data[:, 3]
    # com_z = data[:, 2]
    # pl_x = data[:, 6]
    # pr_x = data[:, 9]
    # pl_z = data[:, 8]

    # pl_y = data[:, 7]
    # pr_y = data[:, 10]

    # zmp_y = data[:, 5]
    # zmp_z = data[:, 8]


    com_x = data[:, 0]
    com_y = data[:, 1]
    com_z = data[:, 2]
    pc_x = data[:, 6]
    pc_y = data[:, 7]
    # pc_z = data[:, 8]


    # Create a figure with 5 subplots (4 rows, 1 column)
    fig, axs = plt.subplots(4, 1, figsize=(8, 7))

    # Subplot 1: XY plane plot
    # axs[0].plot(com_x, com_y, label='CoM', color='blue')
    # axs[0].plot(zmp_x, zmp_y, label='ZMP', color='red')
    # axs[0].legend()





    # axs[0].plot(fl_z, label='FL Z', color='blue')
    # axs[0].plot(fr_z, label='FR Z', color='red')
    # axs[0].plot(fl_x, label='FL X', color='green')
    # axs[0].plot(fr_x, label='FR X', color='orange')
    # axs[0].legend()


    # # Subplot 2: X coordinates vs index
    # axs[1].plot(com_x, label='CoM X', color='blue')
    # axs[1].plot(pl_x, label='PL X', color='red')
    # axs[1].legend()


    # axs[2].plot(com_x, label='CoM X', color='blue')
    # axs[2].plot(pr_x, label='PR X', color='red')
    # axs[2].legend()


    # axs[3].plot(com_z, label='CoM Z', color='blue')
    # axs[3].plot(pl_z, label='PL Z', color='red')
    # axs[3].legend()




    axs[0].plot(fz_, label='FZ', color='blue')
    # axs[0].plot(fc_x, label='Fc X', color='green')
    axs[0].legend()

    # Subplot 2: X coordinates vs index
    axs[1].plot(com_x, label='CoM X', color='blue')
    axs[1].plot(pc_x, label='Pc X', color='red')
    axs[1].legend()

    axs[2].plot(com_y, label='CoM Y', color='blue')
    axs[2].plot(pc_y, label='Pc Y', color='red')
    axs[2].legend()

    axs[3].plot(com_z, label='CoM Z', color='blue')
    # axs[3].plot(pc_z, label='Pc Z', color='red')
    axs[3].legend()





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
    # plt.show()




def main():
    parser = argparse.ArgumentParser(description='plot MPC')
    parser.add_argument('--t', type=float, required=True,
                        help="timestep matching the folder name (e.g., 482 or 482.000000)")
    args = parser.parse_args()

    plot_mpc(args.t)

    plt.show()
    
if __name__ == '__main__':
    main()
