from typing import cast
from math import pi

from controller import DistanceSensor, Motor, PositionSensor, Robot

from robot import RobotInterface
from utils.params import RobotParams, RobotState, Direction, SensorSnapshot, DetectedWalls
from config.enums import Move
from config.models import AppConfig

import logging

logger = logging.getLogger(__name__)
print("robot handlers:", logger.handlers)


class Epuck(RobotInterface):
    """Epuck robot implementation."""

    def __init__(self, config: AppConfig):
        self._cfg = config
        self._robot = Robot()
        self.params = RobotParams(
            self._cfg.robot.axle, self._cfg.robot.wheel, self._cfg.robot.speed
        )
        self._state = RobotState(self._cfg.maze.start_position, self._cfg.maze.target_position)
        self._number_of_reads = 5
        self._front_obstacle = False
        self._init_devices()

    def read_sensors(self) -> DetectedWalls:
        sensors = self._read_sensors_avg(self._number_of_reads)
        return self._detect_walls(sensors)

    def move(self, target: int):
        """
        Move robot to target position and update its state.

        Relative position to the walls is corrected:
            1. During the movement using the PID regulator.
            2. After detecting front wall.

        Args:
            target: index of the position to move.

        Returns:
            None
        """
        move_direction = self._dir_to_move(target)
        front_wall = self._read_and_detect_front()
        if front_wall:
            self._move_front_correct()

        action = self._drive(move_direction)
        self._state.orientation = self.change_orientation(self._state.orientation, action)
        self._state.pos = self._change_position(self._state.pos, self._state.orientation)

    @property
    def robot(self) -> Robot:
        return self._robot

    @property
    def state(self) -> RobotState:
        return self._state

    def _init_devices(self):
        """Initialize robot devices such as motors and sensors."""
        self.left_motor = cast(Motor, self._robot.getDevice("left wheel motor"))
        self.right_motor = cast(Motor, self._robot.getDevice("right wheel motor"))
        self.left_motor.setVelocity(self.params.speed)
        self.right_motor.setVelocity(self.params.speed)
        self._cfg.simulation
        self.ps_left = cast(PositionSensor, self._robot.getDevice("left wheel sensor"))
        self.ps_left.enable(self._cfg.simulation.time_step)
        self.ps_right = cast(PositionSensor, self._robot.getDevice("right wheel sensor"))
        self.ps_right.enable(self._cfg.simulation.time_step)

        ps_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
        self.ps = [cast(DistanceSensor, self._robot.getDevice(n)) for n in ps_names]
        for sensor in self.ps:
            sensor.enable(self._cfg.simulation.time_step)

        self.tof = cast(DistanceSensor, self._robot.getDevice("tof"))
        self.tof.enable(self._cfg.simulation.time_step)

    def _detect_walls(self, sensors: SensorSnapshot):
        """Read and process sensors to detect walls.

        Args:
            robot: MyRobot object with robot devices.
            number_of_reads: Variable which indicates how many times to read sensors.

        Returns:
            tuple[bool, bool, bool, bool]: Variables which indicate respective walls presence (left_wall, front_wall, right_wall, back_wall).
        """
        left_wall = sensors.ps[5] > 80.0
        # front_wall = avg7_front_sensor > 80.0
        right_wall = sensors.ps[2] > 80.0
        back_wall = sensors.ps[4] > 80.0
        front_wall = sensors.tof < 55  # different bcs its TOF, not IR

        self._front_obstacle = front_wall

        return DetectedWalls(left_wall, front_wall, right_wall, back_wall)

    def _read_sensors_avg(self, reads: int) -> SensorSnapshot:
        ps_values = [0.0] * 8
        tof_value = 0
        sensors_indexes = [1, 2, 4, 5, 6]

        # read distance sensors
        for _ in range(reads):  # more scans for better accuracy
            for i in sensors_indexes:
                ps_values[i] += self.ps[i].getValue()
            tof_value += self.tof.getValue()

            self._robot.step(self._cfg.simulation.time_step)

        # average score of sensors measurements
        ps_avg = [v / reads for v in ps_values]
        tof_avg = tof_value / reads
        return SensorSnapshot(ps_avg, tof_avg)

    def _read_and_detect_front(self, reads: int = 1) -> bool:
        """Detect front wall using tof sensor."""
        # tof_value = 0
        # for _ in range(reads):  # more scans for better accuracy
        #     tof_value += self.tof.getValue()

        #     self._robot.step(self._cfg.simulation.time_step)
        # tof_avg = tof_value / reads
        # front_wall = tof_avg < 55
        tof_value = self.tof.getValue()
        front_wall = tof_value < 55
        return front_wall

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
                self._robot.step(self._cfg.simulation.time_step)
                left = self.ps[7].getValue()
                right = self.ps[0].getValue()
                logger.debug("Correct angle to front wall %.2f %.2f", left, right)

        elif left > right:
            while left > right:

                self.left_motor.setVelocity(self.params.speed * -0.1)
                self.right_motor.setVelocity(self.params.speed * 0.1)
                self._robot.step(self._cfg.simulation.time_step)
                left = self.ps[7].getValue()
                right = self.ps[0].getValue()
                logger.debug("Correct angle to front wall %.2f %.2f", left, right)

        front = self.tof.getValue()
        desired_distance = 40.0

        if front > desired_distance:
            while front > desired_distance:

                self.left_motor.setVelocity(self.params.speed * 0.1)
                self.right_motor.setVelocity(self.params.speed * 0.1)

                self._robot.step(self._cfg.simulation.time_step)

                front = self.tof.getValue()
                logger.debug("Correct distance to front wall (Tof) %.2f", front)

        elif front < desired_distance:
            while front < desired_distance:

                self.left_motor.setVelocity(self.params.speed * -0.1)
                self.right_motor.setVelocity(self.params.speed * -0.1)

                self._robot.step(self._cfg.simulation.time_step)

                front = self.tof.getValue()
                logger.debug("Correct distance to front wall (Tof) %.2f", front)

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)

    def _dir_to_move(self, target_cell: int) -> Direction:
        """
        Convert target cell index into a global movement direction.

        Returns:
            Direction: Global direction of movement.
        """
        pos = self._state.pos
        cols = self._cfg.maze.columns

        diff = target_cell - pos

        if diff == cols:
            return Direction.NORTH
        elif diff == 1:
            return Direction.EAST
        elif diff == -cols:
            return Direction.SOUTH
        elif diff == -1:
            return Direction.WEST
        else:
            raise ValueError(f"Target cell {target_cell} is not a direct neighbor of {pos}")

    def _drive(self, move_direction: Direction):
        """Drive robot motors etc. to actually move based on its orientation and move direction.

        Args:
            robot: MyRobot object with robot devices.
            move_direction: Variable direction where to move in global directions.

        Returns:
            None
        """
        action = Move.FORWARD
        if self._state.orientation == move_direction:  # move forward
            self._move_1_tile()

        elif (
            not (
                (self._state.orientation == Direction.WEST) and (move_direction == Direction.NORTH)
            )
        ) != (
            not ((self._state.orientation // 2) == move_direction)
        ):  # right, XOR, 'not' to avoid nonzero values

            action = Move.RIGHT
            self._turn(action)
            self._move_1_tile()

        elif (
            not (
                (self._state.orientation == Direction.NORTH) and (move_direction == Direction.WEST)
            )
        ) != (
            not ((self._state.orientation * 2) == move_direction)
        ):  # left, XOR
            action = Move.LEFT
            self._turn(action)
            self._move_1_tile()

        elif (not ((self._state.orientation * 4) == move_direction)) != (
            not ((self._state.orientation // 4) == move_direction)
        ):  # back, XOR
            action = Move.BACK
            self._turn(action)
            self._move_1_tile()
        return action

    def _move_1_tile(self):
        """Drive robot motors and set encoders position to move forward by distance of exactly one tile.

        Args:
            robot: MyRobot object with robot devices.

        Returns:
            None
        """

        revolutions = self._cfg.maze.tile_length / self.params.wheel  # rev in radians

        left_wheel_revolutions = self.ps_left.getValue()
        right_wheel_revolutions = self.ps_right.getValue()

        left_wheel_revolutions += revolutions
        right_wheel_revolutions += revolutions

        self.left_motor.setVelocity(self.params.speed)
        self.right_motor.setVelocity(self.params.speed)

        self.left_motor.setPosition(left_wheel_revolutions)
        self.right_motor.setPosition(right_wheel_revolutions)
        self._PID_correction()

        logger.debug("Move forward")

    def _turn(self, move_direction: Move):
        """Drive robot motors and set encoders position to turn by exactly 90 or 180 degrees.

        Args:
            robot: MyRobot object with robot devices.
            move_direction: Variable with direction where to move in global directions.

        Returns:
            None
        """

        revolutions = (pi / 2) * self.params.axle / 2 / self.params.wheel  # in radians

        left_wheel_revolutions = self.ps_left.getValue()
        right_wheel_revolutions = self.ps_right.getValue()

        self.left_motor.setVelocity(self.params.speed * 0.33)
        self.right_motor.setVelocity(self.params.speed * 0.33)

        match move_direction:
            case Move.RIGHT:
                left_wheel_revolutions += revolutions
                right_wheel_revolutions -= revolutions
                self.left_motor.setPosition(left_wheel_revolutions)
                self.right_motor.setPosition(right_wheel_revolutions)
                logger.debug("Turn right")
            case Move.LEFT:
                left_wheel_revolutions -= revolutions
                right_wheel_revolutions += revolutions
                self.left_motor.setPosition(left_wheel_revolutions)
                self.right_motor.setPosition(right_wheel_revolutions)
                logger.debug("Turn left")
            case Move.BACK:
                revolutions *= 2
                left_wheel_revolutions += revolutions
                right_wheel_revolutions -= revolutions
                self.left_motor.setPosition(left_wheel_revolutions)
                self.right_motor.setPosition(right_wheel_revolutions)
                logger.debug("Move back")

        self._wait_move_end()

    def _wait_move_end(self):
        """Stops main loop execution until robot ends move.

        Args:
            robot: MyRobot object with robot devices.

        Returns:
            None
        """

        while True:
            distance_left_now = self.ps_left.getValue()
            distance_right_now = self.ps_right.getValue()

            self._robot.step(self._cfg.simulation.time_step)

            distance_left_later = self.ps_left.getValue()
            distance_right_later = self.ps_right.getValue()

            if (distance_left_now == distance_left_later) and (
                distance_right_now == distance_right_later
            ):
                break

    def _PID_correction(self, reads: int = 2):
        """Correct robot position according to distance sensors by changing motors speed.

        Args:
            robot: MyRobot object with robot devices.

        Returns:
            None
        """
        while True:
            distance_left_now = self.ps_left.getValue()
            distance_right_now = self.ps_right.getValue()

            sensors = self._read_sensors_avg(reads)
            right_angle_sensor = sensors.ps[1]
            left_angle_sensor = sensors.ps[6]
            left_wall = sensors.ps[5] > 80.0
            right_wall = sensors.ps[2] > 80.0

            previous_error = 0.00
            error_integral = 0.00
            P = 0.005  # 0.005
            I = 0.002  # 0.0005  0.0001
            D = 0.0008  # 0.0002
            Middle = 75

            if left_wall and right_wall:

                error = left_angle_sensor - right_angle_sensor
                logger.debug("PID error both walls %.3f", error)

                error_integral += error
                error_derivative = previous_error - error
                previous_error = error
                MotorSpeed = P * error + I * error_integral + D * error_derivative
                if MotorSpeed > 0.2:
                    MotorSpeed = 0.2
                elif MotorSpeed < -0.2:
                    MotorSpeed = -0.2

                logger.debug("PID motor speed %.3f", MotorSpeed)

                self.left_motor.setVelocity(self.params.speed + MotorSpeed)
                self.right_motor.setVelocity(self.params.speed - MotorSpeed)
            elif left_wall:
                error = left_angle_sensor - Middle
                logger.debug("PID error left wall %.3f", error)

                error_integral += error
                error_derivative = previous_error - error
                previous_error = error
                MotorSpeed = P * error + I * error_integral + D * error_derivative
                if MotorSpeed > 0.06:
                    MotorSpeed = 0.06
                elif MotorSpeed < -0.06:
                    MotorSpeed = -0.06

                logger.debug("PID motor speed %.3f", MotorSpeed)

                self.left_motor.setVelocity(self.params.speed + MotorSpeed)
                self.right_motor.setVelocity(self.params.speed - MotorSpeed)
            elif right_wall:
                error = right_angle_sensor - Middle
                logger.debug("PID error right wall %.3f", error)

                error_integral += error
                error_derivative = previous_error - error
                previous_error = error
                MotorSpeed = P * error + I * error_integral + D * error_derivative
                if MotorSpeed > 0.06:
                    MotorSpeed = 0.06
                elif MotorSpeed < -0.06:
                    MotorSpeed = -0.06

                logger.debug("PID motor speed %.3f", MotorSpeed)

                self.left_motor.setVelocity(self.params.speed - MotorSpeed)
                self.right_motor.setVelocity(self.params.speed + MotorSpeed)

            distance_left_later = self.ps_left.getValue()
            distance_right_later = self.ps_right.getValue()

            if (distance_left_now == distance_left_later) and (
                distance_right_now == distance_right_later
            ):
                break

    @staticmethod
    def change_orientation(robot_orientation: Direction, action: Move):
        """Change robot orientation basing on last orientation and last turn.

        Args:
            robot_orientation: Variable with actual robot orientation in global directions.
            action: Variable with information where robot turns.

        Returns:
            Direction: Variable with updated robot orientation.
        """
        orientation_value: int = robot_orientation.value
        match action:
            case Move.RIGHT:  # turn right
                if orientation_value == Direction.WEST:
                    orientation_value = Direction.NORTH
                else:
                    orientation_value //= 2
            case Move.LEFT:  # turn left
                if orientation_value == Direction.NORTH:
                    orientation_value = Direction.WEST
                else:
                    orientation_value *= 2
            case Move.BACK:  # turn back
                if orientation_value == Direction.NORTH or orientation_value == Direction.EAST:
                    orientation_value //= 4
                else:
                    orientation_value *= 4

        return Direction(orientation_value)

    def _change_position(self, robot_position: int, robot_orientation: Direction):
        """Update position of the robot basing on current orientation of the robot.

        Args:
            robot_position: Variable with robot position.
            robot_orientation: Variable with actual robot orientation in global directions.

        Returns:
            int: Variable with updated robot position.
        """

        if robot_orientation == Direction.NORTH:
            robot_position = robot_position + self._cfg.maze.columns

        elif robot_orientation == Direction.EAST:
            robot_position = robot_position + 1
        elif robot_orientation == Direction.SOUTH:
            robot_position = robot_position - self._cfg.maze.columns

        elif robot_orientation == Direction.WEST:
            robot_position = robot_position - 1

        return robot_position
