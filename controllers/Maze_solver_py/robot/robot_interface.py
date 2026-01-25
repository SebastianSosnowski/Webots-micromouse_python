from abc import ABC, abstractmethod


class RobotInterface(ABC):
    @abstractmethod
    def read_sensors(self) -> tuple[bool, bool, bool, bool]:
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
