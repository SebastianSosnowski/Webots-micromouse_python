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
        self._number_of_reads = 5
        self._front_obstacle = False
        self._init_devices()

    def read_sensors(self) -> tuple[bool, bool, bool, bool]:
        """
        Read and process sensors to detect walls.

        Returns:
            tuple[bool, bool, bool, bool]: Variables which indicate respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """
        return self._detect_walls()

    def move_to_position(self, target: list[int]):
        """
        Move robot to target position.
        Args:
            target: list of positions to travel through.

        Returns:
            None
        """
        _, front_wall, _, _ = map_f.detect_walls(robot, 5)

        if front_wall:
            self._move_front_correct()

        drive(robot, move_direction)

        robot.state.pos = algorithm_f.change_position(robot.state.pos, robot.state.orientation)

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

    def _detect_walls(self):
        """Read and process sensors to detect walls.

        Args:
            robot: MyRobot object with robot devices.
            number_of_reads: Variable which indicates how many times to read sensors.

        Returns:
            tuple[bool, bool, bool, bool]: Variables which indicate respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """

        avg2_right_sensor = 0  # ps2
        avg4_back_sensor = 0  # ps4
        avg5_left_sensor = 0  # ps5
        # avg7_front_sensor = 0    #ps7
        avg_front_sensor = 0  # tof

        ps_values = [0.0] * 8
        tof_value = 0
        sensors_indexes = [2, 4, 5]

        # read distance sensors
        for i in range(0, self._number_of_reads):  # more scans for better accuracy

            for i in sensors_indexes:
                ps_values[i] = self.ps[i].getValue()
            tof_value = self.tof.getValue()

            avg2_right_sensor += ps_values[2]

            avg4_back_sensor += ps_values[4]

            avg5_left_sensor += ps_values[5]

            # avg7_front_sensor += ps_values[7]

            avg_front_sensor += tof_value

            self._robot.step(world.sim.time_step)  # simulation update

        # average score of sensors measurements
        avg2_right_sensor = avg2_right_sensor / self._number_of_reads
        avg4_back_sensor = avg4_back_sensor / self._number_of_reads
        avg5_left_sensor = avg5_left_sensor / self._number_of_reads
        # avg7_front_sensor = avg7_front_sensor / number_of_reads
        avg_front_sensor = avg_front_sensor / self._number_of_reads

        # Wall detection
        left_wall = avg5_left_sensor > 80.0
        # front_wall = avg7_front_sensor > 80.0
        right_wall = avg2_right_sensor > 80.0
        back_wall = avg4_back_sensor > 80.0
        front_wall = avg_front_sensor < 55  # different bcs its TOF, not IR
        self._front_obstacle = front_wall

        return (left_wall, front_wall, right_wall, back_wall)

    def _move_front_correct(self):
        """Corrects robot position in reference to front wall.
        Uses front angles IR sensors to straight up robot
        and front tof sensor to correct distance.

        Args:
            robot (MyRobot): MyRobot object with robot devices.

        Returns:
            None
        """

        self.left_motor.setPosition(float("inf"))
        self.right_motor.setPosition(float("inf"))

        left = self.ps[7].getValue()
        right = self.ps[0].getValue()

        if left < right:
            while left < right:

                self.left_motor.setVelocity(self.params.speed * 0.1)
                self.right_motor.setVelocity(self.params.speed * -0.1)
                self._robot.step(world.sim.time_step)
                left = self.ps[7].getValue()
                right = self.ps[0].getValue()
                if world.sim.testing:
                    print("sensor angle %.2f" % left, "%.2f" % right)

        elif left > right:
            while left > right:

                self.left_motor.setVelocity(self.params.speed * -0.1)
                self.right_motor.setVelocity(self.params.speed * 0.1)
                self._robot.step(world.sim.time_step)
                left = self.ps[7].getValue()
                right = self.ps[0].getValue()
                if world.sim.testing:
                    print("sensor angle %.2f" % left, "%.2f" % right)

        front = self.tof.getValue()
        desired_distance = 40.0

        if front > desired_distance:
            while front > desired_distance:

                self.left_motor.setVelocity(self.params.speed * 0.1)
                self.right_motor.setVelocity(self.params.speed * 0.1)

                self._robot.step(world.sim.time_step)

                front = self.tof.getValue()

                if world.sim.testing:
                    print("sensor tof %.2f" % front)

        elif front < desired_distance:
            while front < desired_distance:

                self.left_motor.setVelocity(self.params.speed * -0.1)
                self.right_motor.setVelocity(self.params.speed * -0.1)

                self._robot.step(world.sim.time_step)

                front = self.tof.getValue()

                if world.sim.testing:
                    print("sensor tof %.2f" % front)

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
