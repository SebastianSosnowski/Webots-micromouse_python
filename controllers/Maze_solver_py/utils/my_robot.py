from controller import Robot, Motor, PositionSensor, DistanceSensor
from typing import cast
from .params import RobotParams, RobotState
from config.world import world


class MyRobot(Robot):
    def __init__(self, robot_cfg: dict):
        super().__init__()
        self.params = RobotParams(robot_cfg["axle"], robot_cfg["wheel"], robot_cfg["speed"])
        self.state = RobotState(world.maze.start_cell, world.maze.target_cell)
        self._init_devices()

    def _init_devices(self):
        self.left_motor = cast(Motor, self.getDevice("left wheel motor"))
        self.right_motor = cast(Motor, self.getDevice("right wheel motor"))
        self.left_motor.setVelocity(self.params.speed)
        self.right_motor.setVelocity(self.params.speed)

        self.ps_left = cast(PositionSensor, self.getDevice("left wheel sensor"))
        self.ps_left.enable(world.sim.time_step)
        self.ps_right = cast(PositionSensor, self.getDevice("right wheel sensor"))
        self.ps_right.enable(world.sim.time_step)

        ps_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
        self.ps = [cast(DistanceSensor, self.getDevice(n)) for n in ps_names]
        for sensor in self.ps:
            sensor.enable(world.sim.time_step)

        self.tof = cast(DistanceSensor, self.getDevice("tof"))
        self.tof.enable(world.sim.time_step)
