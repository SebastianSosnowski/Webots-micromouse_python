from robot.robot_base import MyRobot
from algorithm.algorithm_base import AlgorithmV2


class MazeSolver:
    def __init__(self, robot_cfg: dict, sim_cfg: dict) -> None:
        self.robot = MyRobot(robot_cfg)
        self.algorithm = AlgorithmV2(sim_cfg)

    
