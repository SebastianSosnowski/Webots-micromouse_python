from config.logging_config import setup_logging

setup_logging()
import logging

from robot.robot_base import MyRobot
from algorithm.algorithm_base import Algorithm
from config.enums import Mode
from config.models import AppConfig
from read_files.storage import read_results

logger = logging.getLogger(__name__)


class MazeSolver:
    def __init__(self, config: AppConfig) -> None:
        self._cfg = config
        self.robot = MyRobot(self._cfg)
        self.algorithm = Algorithm(self._cfg)
        self.visited: set[int] = set()
        self.visited.add(self.robot.state.pos)
        self.prev_pos: int | None = None

    def init_drawer_values(self) -> tuple[list[int], list | dict, list | dict]:
        if self._cfg.simulation.mode == Mode.SEARCH:
            return [], self.algorithm.maze_map, self.algorithm.position_values
        else:
            path, maze_map, values = read_results(
                self._cfg.simulation.maze_layout, self._cfg.simulation.algorithm
            )
            logger.info("Path length: %d", len(path))

            return path, maze_map, values
