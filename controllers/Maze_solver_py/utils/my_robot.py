from controller import Robot, Motor, PositionSensor, DistanceSensor
from typing import cast
from config.enums import Direction, Mode
from .params import RobotParams, SimulationParams, MazeParams, RobotState


class MyRobot(Robot):
    def __init__(self, config: dict):
        super().__init__()
        self._init_simulation_params(config["simulation"])
        self._init_robot_params(config["robot"])
        self._init_maze_params(config["maze"])
        self.state = RobotState(self.maze.start_cell)
        self._init_devices()

    def _init_simulation_params(self, sim_cfg: dict):
        self.sim = SimulationParams(
            sim_cfg["mode"],
            sim_cfg["algorithm"],
            sim_cfg["maze_layout"],
            sim_cfg["testing"],
            sim_cfg["whole_search"],
            sim_cfg["time_step"],
        )

    def _init_robot_params(self, robot_cfg: dict):
        self.robot = RobotParams(
            robot_cfg["axle"], robot_cfg["wheel"], robot_cfg["speed"]
        )

    def _init_maze_params(self, maze_cfg: dict):
        self.maze = MazeParams(
            maze_cfg["rows"],
            maze_cfg["columns"],
            maze_cfg["start_cell"],
            maze_cfg["target_cell"],
            maze_cfg["tile_length"],
            maze_cfg["visited_flag"],
        )

    def _init_devices(self):
        self.left_motor = cast(Motor, self.getDevice("left wheel motor"))
        self.right_motor = cast(Motor, self.getDevice("right wheel motor"))
        self.left_motor.setVelocity(self.robot.speed)
        self.right_motor.setVelocity(self.robot.speed)

        self.ps_left = cast(PositionSensor, self.getDevice("left wheel sensor"))
        self.ps_left.enable(self.sim.time_step)
        self.ps_right = cast(PositionSensor, self.getDevice("right wheel sensor"))
        self.ps_right.enable(self.sim.time_step)

        ps_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
        self.ps = [cast(DistanceSensor, self.getDevice(n)) for n in ps_names]
        for sensor in self.ps:
            sensor.enable(self.sim.time_step)

        self.tof = cast(DistanceSensor, self.getDevice("tof"))
        self.tof.enable(self.sim.time_step)
