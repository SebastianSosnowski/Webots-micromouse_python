from controller import Robot

from robot import RobotInterface


class Epuck(RobotInterface):
    def __init__(self):
        self._robot = Robot()
        super().__init__()

    def read_sensors(self, number_of_reads: int):
        """
        Read and process sensors to detect walls.

        Returns:
            tuple[bool, bool, bool, bool]: Variables which indicate respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """
        pass

    def move_to_position(self, target: list[int]):
        """
        Move robot to target position.
        Args:
            target: list of positions to travel through.

        Returns:
            None
        """
        pass
