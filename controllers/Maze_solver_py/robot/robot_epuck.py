from typing import cast

from controller import DistanceSensor, Motor, PositionSensor, Robot

from robot import RobotInterface
from utils.params import RobotParams, RobotState
from config.world import world


class Epuck(RobotInterface):
    def __init__(self, robot_cfg: dict):
        self._robot = Robot()
        self.params = RobotParams(robot_cfg["axle"], robot_cfg["wheel"], robot_cfg["speed"])
        self.state = RobotState(world.maze.start_cell, world.maze.target_cell)
        self._init_devices()

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

    def _init_devices(self):
        """Initialize robot devices such as motors and sensors."""
        self.left_motor = cast(Motor, self._robot.getDevice("left wheel motor"))
        self.right_motor = cast(Motor, self._robot.getDevice("right wheel motor"))
        self.left_motor.setVelocity(self.params.speed)
        self.right_motor.setVelocity(self.params.speed)

        self.ps_left = cast(PositionSensor, self._robot.getDevice("left wheel sensor"))
        self.ps_left.enable(world.sim.time_step)
        self.ps_right = cast(PositionSensor, self._robot.getDevice("right wheel sensor"))
        self.ps_right.enable(world.sim.time_step)

        ps_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
        self.ps = [cast(DistanceSensor, self._robot.getDevice(n)) for n in ps_names]
        for sensor in self.ps:
            sensor.enable(world.sim.time_step)

        self.tof = cast(DistanceSensor, self._robot.getDevice("tof"))
        self.tof.enable(world.sim.time_step)
