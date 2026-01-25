from abc import ABC, abstractmethod
from utils.params import RobotState


class AlgorithmInterface(ABC):
    @abstractmethod
    def init(self) -> tuple[list | dict, list[int]]:
        pass

    @abstractmethod
    def update(self, maze_map, state: RobotState) -> list[int]:
        pass

    @abstractmethod
    def finish(self):
        pass
