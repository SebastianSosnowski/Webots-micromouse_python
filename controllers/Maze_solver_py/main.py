"""flood_fill_py controller."""

from pathlib import Path

from config.enums import Algorithms
from config.loader import load_config
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

    match world.sim.algorithm:
        case Algorithms.KEYBOARD:
            main_f.keyboard_main(robot)
        case Algorithms.FLOODFILL:
            main_f.floodfill_main(robot)
        case Algorithms.DFS:
            main_f.DFS_main(robot)
        case Algorithms.BFS:
            main_f.BFS_main(robot)
        case Algorithms.A_STAR:
            main_f.A_star_main(robot)
        case Algorithms.A_STAR_MOD:
            main_f.A_star_main_modified(robot)


if __name__ == "__main__":
    config_path = Path("config.yaml")
    config = load_config(config_path)
    world.init(config["simulation"], config["maze"])

    robot = MyRobot(config["robot"])
    run_robot(robot)
    # maze_solver = MazeSolver(config["robot"], config["simulation"])

    # main_f.interface_main(maze_solver)
