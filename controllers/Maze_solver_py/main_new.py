"""flood_fill_py controller."""

from pathlib import Path

from config.enums import Algorithm
from config.loader import load_config
from config.world import world
from utils.my_robot import MyRobot

import main_functions as main_f


def run_robot(robot: MyRobot):

    match world.sim.algorithm:
        case Algorithm.KEYBOARD:
            main_f.keyboard_main(robot)
        case Algorithm.FLOODFILL:
            main_f.floodfill_main(robot)
        case Algorithm.DFS:
            main_f.DFS_main(robot)
        case Algorithm.BFS:
            main_f.BFS_main(robot)
        case Algorithm.A_STAR:
            main_f.A_star_main(robot)
        case Algorithm.A_STAR_MOD:
            main_f.A_star_main_modified(robot)


if __name__ == "__main__":
    config_path = Path("config.yaml")
    config = load_config(config_path)
    world.init(config["simulation"], config["maze"])

    robot = MyRobot(config["robot"])

    run_robot(robot)
