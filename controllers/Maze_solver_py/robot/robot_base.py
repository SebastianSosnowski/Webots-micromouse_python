from robot import RobotInterface, Epuck


class MyRobot(RobotInterface):
    def __init__(self, robot_cfg: dict):
        if robot_cfg["model"] == "epuck":
            self.impl = Epuck(robot_cfg)
        else:
            raise ValueError(f"Unknown robot: {robot_cfg['model']}")

    def read_sensors(self) -> tuple[bool, bool, bool, bool]:
        """
        Read and process sensors to detect walls.

        Returns:
            tuple[bool, bool, bool, bool]: Variables which indicate respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """
        return self.impl.read_sensors()

    def move(self, target: int):
        """
        Move robot to target position.
        Args:
            target: list of positions to travel through.

        Returns:
            None
        """
        self.impl.move(target)
