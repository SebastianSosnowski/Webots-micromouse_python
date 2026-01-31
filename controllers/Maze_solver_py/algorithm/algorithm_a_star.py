from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls
from config.models import AppConfig


class AStar(AlgorithmInterface):
    def __init__(self, cfg: AppConfig):
        pass

    def init(self):
        raise NotImplementedError()

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        raise NotImplementedError()

    def finish(self):
        raise NotImplementedError()

    def prepare_results(self) -> tuple[list[int], list | dict, list | dict]:
        raise NotImplementedError()

    @property
    def maze_map(self) -> list[int] | dict[int, list[int]]:
        raise NotImplementedError()

    @property
    def position_values(self) -> list:
        raise NotImplementedError()

    @property
    def pos(self) -> int:
        raise NotImplementedError()
