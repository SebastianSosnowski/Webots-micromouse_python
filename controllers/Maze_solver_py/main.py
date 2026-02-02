"""flood_fill_py controller."""

from pathlib import Path

from config.enums import Algorithms
from config.loader import load_config, load_config
from config.world import world

from utils.my_robot import MyRobot

# from robot.robot_base import MyRobot
from maze_solver import MazeSolver

import main_functions as main_f


def run_robot(robot: MyRobot):
    """Run the robot based on the configured algorithm.

    Args:
        robot: The robot instance to control.
    """


if __name__ == "__main__":
    config_path = Path("config.yaml")

    config = load_config(config_path)
    maze_solver = MazeSolver(config)

    main_f.interface_main(maze_solver)
