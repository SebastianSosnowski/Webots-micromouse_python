from abc import ABC, abstractmethod
from utils.types import RobotState, DetectedWalls


class AlgorithmInterface(ABC):
    """Interface class for algorithm implementation."""

    @abstractmethod
    def init(self) -> None:
        """Initialize all algorithm-related and simulation parameters."""
        pass

    @abstractmethod
    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        """
        Execute a step of algorithm based on robot sensors reading and its state.

        Args:
            detected: A dataclass with information about which walls were detected by robot in current position.
            state: A dataclass with information about current robot state.

        Returns:
            A list of positions to which robot should move next.
        """
        pass

    @abstractmethod
    def finish(self) -> bool:
        """
        Verify if algorithm finished operation.

        Returns:
            True if end, otherwise False.
        """
        pass

    @abstractmethod
    def prepare_results(self) -> tuple[list[int], list | dict, list | dict]:
        """Prepare algorithm output for robot and visualization to use.

        Returns:
            tuple:
                - path (list[int])
                    Sequence of maze cell indices from start to target.
                    The start cell itself is excluded.
                - maze_map (list | dict)
                    Representation of the explored maze used for visualization.
                    The concrete structure depends on the algorithm.
                - values (list | dict)
                    Algorithm-specific values associated with maze cells
                    (e.g. distances, visit order, costs).
                    Returns an empty list if not applicable.
        """
        pass

    @property
    @abstractmethod
    def maze_map(self) -> list[int] | dict[int, list[int]]:
        """Return maze map, which represents currently discovered part of maze."""
        pass

    @property
    @abstractmethod
    def position_values(self) -> list | dict:
        """
        Return algorithm values, which some algorithms assign to each position.

        An example is distance map for floodfill algorithm, where each position has value,
        which represents distance from the target.
        If algorithm doesn't use such values (e.g. DFS), return empty list.
        """
        pass

    @property
    @abstractmethod
    def pos(self) -> int:
        """
        Return current position in which algorithm is.

        It's either a current robot position or
        position, to which currently robot moves to after update().
        """
        pass
