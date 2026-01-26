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
    def prepare_results(self) -> tuple[list[int], list | dict, list | dict]:
        """Prepare algorithm output for robot and visualization to use.

        Args:
            None

        Returns:
            path: Positions list from start to target. Exclude robot start position.
            map: A structure with scanned map during maze exploration to draw on maze.
            values: A values of algorithm to draw on maze cells. Return empty list if none.
        """
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
