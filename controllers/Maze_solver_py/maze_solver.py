from robot.robot_base import MyRobot
from algorithm.algorithm_base import AlgorithmV2
from config.world import world
from config.enums import Mode
from config.models import AppConfig
from algorithm_functions import read_results


class MazeSolver:
    def __init__(self, config: AppConfig) -> None:
        self.robot = MyRobot(config)
        self.algorithm = AlgorithmV2(config)

    def init_drawer_values(self) -> tuple[list[int], list | dict, list | dict]:
        if world.sim.mode == Mode.SEARCH:
            return [], self.algorithm.maze_map, self.algorithm.distance
        else:
            return read_results()
