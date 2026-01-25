from abc import ABC, abstractmethod
from utils.params import RobotState, DetectedWalls


class AlgorithmInterface(ABC):
    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        pass

    @abstractmethod
    def finish(self):
        pass

    @abstractmethod
    def prepare_results(self) -> tuple[list | dict, list[int]]:
        pass

    @property
    @abstractmethod
    def maze_map(self) -> list | dict:
        pass

    @property
    @abstractmethod
    def distance(self) -> list:
        pass

    @property
    @abstractmethod
    def pos(self) -> int:
        pass
