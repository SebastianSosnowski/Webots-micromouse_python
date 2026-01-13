from controller import Robot, Motor, PositionSensor, DistanceSensor
from typing import cast


class MyRobot(Robot):
    def __init__(self, config: dict):
        super().__init__()
        self.algorithm = config["simulation"]["algorithm"]
        self.axle = config["robot"]["axle"]
        self.wheel = config["robot"]["wheel"]
        self.speed = config["robot"].get("speed", 4)
        self.time_step = config["simulation"].get("time_step", 64)
        self._init_devices()

    def _init_devices(self):
        self.left_motor = cast(Motor, self.getDevice("left wheel motor"))
        self.right_motor = cast(Motor, self.getDevice("right wheel motor"))
        self.left_motor.setVelocity(self.speed)
        self.right_motor.setVelocity(self.speed)

        self.ps_left = cast(PositionSensor, self.getDevice("left wheel sensor"))
        self.ps_left.enable(self.time_step)
        self.ps_right = cast(PositionSensor, self.getDevice("right wheel sensor"))
        self.ps_right.enable(self.time_step)

        ps_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
        self.ps = [cast(DistanceSensor, self.getDevice(n)) for n in ps_names]
        for sensor in self.ps:
            sensor.enable(self.time_step)

        self.tof = cast(DistanceSensor, self.getDevice("tof"))
        self.tof.enable(self.time_step)
