"""MPC-CBF controller for a differential drive mobile robot.

Author: Elena Oikonomou
Date: Fall 2022
"""

import numpy as np

from mpc_cbf import MPC
from plotter import Plotter
import util


def main():
    np.random.seed(99)

    # Define controller & run simulation
    controller = MPC()
    controller.run_simulation()  # Closed-loop control simulation

    # Plots
    plotter = Plotter(controller)
    plotter.plot_results()          # trajectories.png
    plotter.plot_path()             # path.png
    plotter.plot_cbf()              # distance.png
    # plotter.plot_predictions(15)
    # plotter.create_trajectories_animation()
    plotter.create_path_animation()

    # Store results
    # util.save_mpc_results(controller)


if __name__ == '__main__':
    main()
