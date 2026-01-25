from algorithm import AlgorithmInterface
from utils.params import RobotState


class DFS(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        pass

    def init(self):
        return ([], [])

    def update(self, maze_map, state: RobotState) -> list[int]:
        pass

    def finish(self):
        pass
