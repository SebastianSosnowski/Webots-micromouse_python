from robot.robot_base import MyRobot
from algorithm.algorithm_base import AlgorithmV2
from config.world import world
from config.enums import Mode
from algorithm_functions import read_results


class MazeSolver:
    def __init__(self, robot_cfg: dict, sim_cfg: dict) -> None:
        self.robot = MyRobot(robot_cfg)
        self.algorithm = AlgorithmV2(sim_cfg)

    def init_drawer_values(self) -> tuple[list[int], list | dict, list | dict]:
        if world.sim.mode == Mode.SEARCH:
            return [], self.algorithm.maze_map, self.algorithm.position_values
        else:
            return read_results()
