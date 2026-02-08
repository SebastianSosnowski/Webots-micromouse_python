from controller import Robot

from robot import RobotInterface, Epuck
from utils.types import DetectedWalls, RobotState
from config.models import AppConfig


class MyRobot(RobotInterface):
    """Base class for robot interface, working as a wrapper to use.

    The class chooses correct robot implementation based on provided robot model.
    To add new robot, update __init__ if statement and implement corresponding class.
    """

    def __init__(self, config: AppConfig):
        if config.robot.model == "epuck":
            self.impl = Epuck(config)
        else:
            raise ValueError(f"Unknown robot: {config.robot.model}")

    def read_sensors(self) -> DetectedWalls:
        return self.impl.read_sensors()

    def move(self, target: int):
        self.impl.move(target)

    @property
    def robot(self) -> Robot:
        return self.impl.robot

    @property
    def state(self) -> RobotState:
        return self.impl._state
