from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls
from config.models import AppConfig


class BFS(AlgorithmInterface):
    def __init__(self, cfg: AppConfig):
        pass

    def init(self):
        return ([], [])

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        pass

    def finish(self):
        pass

    def prepare_results(self) -> tuple[list | dict, list[int]]:
        pass
