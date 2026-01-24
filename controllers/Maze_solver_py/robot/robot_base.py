from robot import RobotInterface, Epuck

class MyRobotV2(RobotInterface):
    def __init__(self, robot_cfg: dict):
        if robot_cfg["model"] == "epuck":
            self.impl = Epuck()

    def read_sensors(self, number_of_reads: int):
        """
        Read and process sensors to detect walls.

        Returns:
            tuple[bool, bool, bool, bool]: Variables which indicate respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """
        self.impl.read_sensors(number_of_reads)

    def move_to_position(self, target: list[int]):
        """
        Move robot to target position.
        Args:
            target: list of positions to travel through.

        Returns:
            None
        """
        self.impl.move_to_position(target)
