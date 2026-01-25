from abc import ABC, abstractmethod

from controller import Robot

from utils.params import DetectedWalls, RobotState


class RobotInterface(ABC):
    @abstractmethod
    def read_sensors(self) -> DetectedWalls:
        """
        Read and process sensors to detect walls.

        Returns:
            tuple[bool, bool, bool, bool]: Variables which indicate respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """
        pass

    @abstractmethod
    def move(self, target: int):
        """
        Move robot to target position.
        Args:
            target: list of positions to travel through.

        Returns:
            None
        """
        pass

    @property
    @abstractmethod
    def robot(self) -> Robot:
        """return Robot() instance"""
        pass

    @property
    @abstractmethod
    def state(self) -> RobotState:
        """return Robot() instance"""
        pass
