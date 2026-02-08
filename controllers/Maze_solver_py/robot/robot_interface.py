from abc import ABC, abstractmethod

from controller import Robot

from utils.types import DetectedWalls, RobotState


class RobotInterface(ABC):
    """Interface class for robot implementation."""

    @abstractmethod
    def read_sensors(self) -> DetectedWalls:
        """
        Read and process sensors to detect walls.

        Returns:
            DetectedWalls: Respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """
        pass

    @abstractmethod
    def move(self, target: int):
        """
        Move robot to target position.

        Args:
            target: index of the position to move.

        Returns:
            None
        """
        pass

    @property
    @abstractmethod
    def robot(self) -> Robot:
        """return Robot() instance."""
        pass

    @property
    @abstractmethod
    def state(self) -> RobotState:
        """return RobotState data."""
        pass
