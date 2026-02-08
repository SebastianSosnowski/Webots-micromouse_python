from robot.robot_base import MyRobot
from algorithm.algorithm_base import Algorithm
from config.enums import Mode
from config.models import AppConfig
from read_files.storage import read_results


class MazeSolver:
    def __init__(self, config: AppConfig) -> None:
        self._cfg = config
        self.robot = MyRobot(self._cfg)
        self.algorithm = Algorithm(self._cfg)
        self.visited: set[int] = set()
        self.visited.add(self.robot.state.pos)

    def init_drawer_values(self) -> tuple[list[int], list | dict, list | dict]:
        if self._cfg.simulation.mode == Mode.SEARCH:
            return [], self.algorithm.maze_map, self.algorithm.position_values
        else:
            return read_results(self._cfg.simulation.maze_layout, self._cfg.simulation.algorithm)
