from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls


class AStar(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        pass

    def init(self):
        return ([], [])

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        pass

    def finish(self):
        pass

    def prepare_results(self) -> tuple[list | dict, list[int]]:
        pass
