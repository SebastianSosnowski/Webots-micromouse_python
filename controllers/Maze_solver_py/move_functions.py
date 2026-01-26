# Move related functions

from math import pi

import algorithm_functions as algorithm_f
from algorithm_functions import change_orientation
import map_functions as map_f

from config.enums import Direction, Move
from config.world import world

from utils.my_robot import MyRobot

# from robot.robot_base import MyRobot


def move_one_position_graph(robot: MyRobot, current_destination: int):
    """Executes robot movement to next position. Used in graph algorithms.

    Args:
        robot: MyRobot object with robot devices.
        current_destination: Variable with cell to which robot moves.

    Returns:
        None
    """

    move_direction = algorithm_f.where_to_move_graph(robot.state.pos, current_destination)

    _, front_wall, _, _ = map_f.detect_walls(robot, 5)
    if front_wall:
        move_front_correct(robot)

    drive(robot, move_direction)

    robot.state.pos = current_destination


def move_one_position(robot: MyRobot, walls: int, distance: list[int]):
    """Executes robot movement to next position. Used in floodfill.

    Args:
        robot: MyRobot object with robot devices.
        walls: Variable with walls in current robot position.
        distance: List with actual distances values/path.

    Returns:
        None
    """

    move_direction = algorithm_f.where_to_move(robot, walls, distance)

    _, front_wall, _, _ = map_f.detect_walls(robot, 5)

    if front_wall:
        move_front_correct(robot)

    drive(robot, move_direction)

    robot.state.pos = algorithm_f.change_position(robot.state.pos, robot.state.orientation)


def drive(robot: MyRobot, move_direction: Direction):
    """Drive robot motors etc. to actually move based on its orientation and move direction.

    Args:
        robot: MyRobot object with robot devices.
        move_direction: Variable direction where to move in global directions.

    Returns:
        None
    """
    if robot.state.orientation == move_direction:  # move forward
        move_1_tile(robot)

    elif (
        not ((robot.state.orientation == Direction.WEST) and (move_direction == Direction.NORTH))
    ) != (
        not ((robot.state.orientation // 2) == move_direction)
    ):  # right, XOR, 'not' to avoid nonzero values

        robot.state.orientation = change_orientation(robot.state.orientation, Move.RIGHT)
        turn(robot, Move.RIGHT)
        move_1_tile(robot)

    elif (
        not ((robot.state.orientation == Direction.NORTH) and (move_direction == Direction.WEST))
    ) != (
        not ((robot.state.orientation * 2) == move_direction)
    ):  # left, XOR

        robot.state.orientation = change_orientation(robot.state.orientation, Move.LEFT)
        turn(robot, Move.LEFT)
        move_1_tile(robot)

    elif (not ((robot.state.orientation * 4) == move_direction)) != (
        not ((robot.state.orientation // 4) == move_direction)
    ):  # back, XOR

        robot.state.orientation = change_orientation(robot.state.orientation, Move.BACK)
        turn(robot, Move.BACK)
        move_1_tile(robot)


def read_sensors(robot: MyRobot, number_of_reads: int):
    """Read and process left and right sensors for a PID controller.

    Args:
        robot: MyRobot object with robot devices.
        number_of_reads: Variable which indicates how many times to read sensors.

    Returns:
        tuple[float, float, bool, bool]: Variables with right angle sensor value, left angle sensor value, left wall presence, right wall presence.
    """

    avg1_right_angle_sensor = 0  # ps1
    avg6_left_angle_sensor = 0  # ps6

    avg2_right_sensor = 0  # ps2
    avg5_left_sensor = 0  # ps5

    # read distance sensors
    for i in range(0, number_of_reads):  # more scans for better accuracy

        avg1_right_angle_sensor += robot.ps[1].getValue()
        avg6_left_angle_sensor += robot.ps[6].getValue()

        avg2_right_sensor += robot.ps[2].getValue()
        avg5_left_sensor += robot.ps[5].getValue()

        robot.step(world.sim.time_step)  # simulation update

    # average score of sensors measurements
    avg1_right_angle_sensor = avg1_right_angle_sensor / number_of_reads
    avg6_left_angle_sensor = avg6_left_angle_sensor / number_of_reads

    avg2_right_sensor = avg2_right_sensor / number_of_reads
    avg5_left_sensor = avg5_left_sensor / number_of_reads

    left_wall = avg5_left_sensor > 80.0
    right_wall = avg2_right_sensor > 80.0

    return avg1_right_angle_sensor, avg6_left_angle_sensor, left_wall, right_wall


def PID_correction(robot: MyRobot):
    """Correct robot position according to distance sensors by changing motors speed.

    Args:
        robot: MyRobot object with robot devices.

    Returns:
        None
    """
    while True:
        distance_left_now = robot.ps_left.getValue()
        distance_right_now = robot.ps_right.getValue()

        right_angle_sensor, left_angle_sensor, left_wall, right_wall = read_sensors(robot, 2)

        previous_error = 0.00
        error_integral = 0.00
        P = 0.005  # 0.005
        I = 0.002  # 0.0005  0.0001
        D = 0.0008  # 0.0002
        Middle = 75

        if left_wall and right_wall:

            error = left_angle_sensor - right_angle_sensor

            if world.sim.testing:
                print("error %.3f" % error)

            error_integral += error
            error_derivative = previous_error - error
            previous_error = error
            MotorSpeed = P * error + I * error_integral + D * error_derivative
            if MotorSpeed > 0.2:
                MotorSpeed = 0.2
            elif MotorSpeed < -0.2:
                MotorSpeed = -0.2

            if world.sim.testing:
                print("speed %.3f" % MotorSpeed)

            robot.left_motor.setVelocity(robot.params.speed + MotorSpeed)
            robot.right_motor.setVelocity(robot.params.speed - MotorSpeed)
        elif left_wall:
            error = left_angle_sensor - Middle

            if world.sim.testing:
                print("errorL %.3f" % error)

            error_integral += error
            error_derivative = previous_error - error
            previous_error = error
            MotorSpeed = P * error + I * error_integral + D * error_derivative
            if MotorSpeed > 0.06:
                MotorSpeed = 0.06
            elif MotorSpeed < -0.06:
                MotorSpeed = -0.06

            if world.sim.testing:
                print("speed %.3f" % MotorSpeed)

            robot.left_motor.setVelocity(robot.params.speed + MotorSpeed)
            robot.right_motor.setVelocity(robot.params.speed - MotorSpeed)
        elif right_wall:
            error = right_angle_sensor - Middle

            if world.sim.testing:
                print("errorR %.3f" % error)

            error_integral += error
            error_derivative = previous_error - error
            previous_error = error
            MotorSpeed = P * error + I * error_integral + D * error_derivative
            if MotorSpeed > 0.06:
                MotorSpeed = 0.06
            elif MotorSpeed < -0.06:
                MotorSpeed = -0.06

            if world.sim.testing:
                print("speed %.3f" % MotorSpeed)

            robot.left_motor.setVelocity(robot.params.speed - MotorSpeed)
            robot.right_motor.setVelocity(robot.params.speed + MotorSpeed)

        distance_left_later = robot.ps_left.getValue()
        distance_right_later = robot.ps_right.getValue()

        if (distance_left_now == distance_left_later) and (
            distance_right_now == distance_right_later
        ):
            break


def move_1_tile(robot: MyRobot):
    """Drive robot motors and set encoders position to move forward by distance of exactly one tile.

    Args:
        robot: MyRobot object with robot devices.

    Returns:
        None
    """

    revolutions = world.maze.tile_length / robot.params.wheel  # rev in radians

    left_wheel_revolutions = robot.ps_left.getValue()
    right_wheel_revolutions = robot.ps_right.getValue()

    left_wheel_revolutions += revolutions
    right_wheel_revolutions += revolutions

    robot.left_motor.setVelocity(robot.params.speed)
    robot.right_motor.setVelocity(robot.params.speed)

    robot.left_motor.setPosition(left_wheel_revolutions)
    robot.right_motor.setPosition(right_wheel_revolutions)
    PID_correction(robot)

    if world.sim.testing:
        print("forward")

    # wait_move_end(robot, ps_left, ps_right)


def move_front_correct(robot: MyRobot):
    """Corrects robot position in reference to front wall.
    Uses front angles IR sensors to straight up robot
    and front tof sensor to correct distance.

    Args:
        robot (MyRobot): MyRobot object with robot devices.

    Returns:
        None
    """

    robot.left_motor.setPosition(float("inf"))
    robot.right_motor.setPosition(float("inf"))

    left = robot.ps[7].getValue()
    right = robot.ps[0].getValue()

    if left < right:
        while left < right:

            robot.left_motor.setVelocity(robot.params.speed * 0.1)
            robot.right_motor.setVelocity(robot.params.speed * -0.1)
            robot.step(world.sim.time_step)
            left = robot.ps[7].getValue()
            right = robot.ps[0].getValue()
            if world.sim.testing:
                print("sensor angle %.2f" % left, "%.2f" % right)

    elif left > right:
        while left > right:

            robot.left_motor.setVelocity(robot.params.speed * -0.1)
            robot.right_motor.setVelocity(robot.params.speed * 0.1)
            robot.step(world.sim.time_step)
            left = robot.ps[7].getValue()
            right = robot.ps[0].getValue()
            if world.sim.testing:
                print("sensor angle %.2f" % left, "%.2f" % right)

    front = robot.tof.getValue()
    desired_distance = 40.0

    if front > desired_distance:
        while front > desired_distance:

            robot.left_motor.setVelocity(robot.params.speed * 0.1)
            robot.right_motor.setVelocity(robot.params.speed * 0.1)

            robot.step(world.sim.time_step)

            front = robot.tof.getValue()

            if world.sim.testing:
                print("sensor tof %.2f" % front)

    elif front < desired_distance:
        while front < desired_distance:

            robot.left_motor.setVelocity(robot.params.speed * -0.1)
            robot.right_motor.setVelocity(robot.params.speed * -0.1)

            robot.step(world.sim.time_step)

            front = robot.tof.getValue()

            if world.sim.testing:
                print("sensor tof %.2f" % front)

    robot.left_motor.setVelocity(0)
    robot.right_motor.setVelocity(0)


def turn(robot: MyRobot, move_direction: Move):
    """Drive robot motors and set encoders position to turn by exactly 90 or 180 degrees.

    Args:
        robot: MyRobot object with robot devices.
        move_direction: Variable with direction where to move in global directions.

    Returns:
        None
    """

    revolutions = (pi / 2) * robot.params.axle / 2 / robot.params.wheel  # in radians

    left_wheel_revolutions = robot.ps_left.getValue()
    right_wheel_revolutions = robot.ps_right.getValue()

    robot.left_motor.setVelocity(robot.params.speed * 0.33)
    robot.right_motor.setVelocity(robot.params.speed * 0.33)

    match move_direction:
        case Move.RIGHT:
            left_wheel_revolutions += revolutions
            right_wheel_revolutions -= revolutions
            robot.left_motor.setPosition(left_wheel_revolutions)
            robot.right_motor.setPosition(right_wheel_revolutions)

            if world.sim.testing:
                print("right")
        case Move.LEFT:
            left_wheel_revolutions -= revolutions
            right_wheel_revolutions += revolutions
            robot.left_motor.setPosition(left_wheel_revolutions)
            robot.right_motor.setPosition(right_wheel_revolutions)

            if world.sim.testing:
                print("left")
        case Move.BACK:
            revolutions *= 2
            left_wheel_revolutions += revolutions
            right_wheel_revolutions -= revolutions
            robot.left_motor.setPosition(left_wheel_revolutions)
            robot.right_motor.setPosition(right_wheel_revolutions)

            if world.sim.testing:
                print("back")

    wait_move_end(robot)


def move_back_DFS(
    robot: MyRobot,
    destination: int,
    maze_map: dict,
    fork: dict,
    fork_number: int,
    unused_routes: dict,
    path: list,
):
    """Moves robot back to previous valid fork (Depth first search).

    Args:
        robot: MyRobot object with robot devices.
        destination: Variable with robot destination cell.
        maze_map: Dictionary with maze map graph.
        fork: Dictionary with paths to each fork from current position.
        fork_number: Variable with number of last used fork.
        unused_routes: Dictionary with number of unused routes for each fork.
        path: List with actual path from start to current position.

    Returns:
        tuple[dict, int, dict, list]: Updated fork dictionary, fork number, unused routes dictionary, path list.
    """

    while destination not in maze_map[robot.state.pos]:
        for cell in reversed(fork[fork_number]):
            move_one_position_graph(robot, cell)
            fork[fork_number].pop()
            path.pop()

        unused_routes[fork_number] -= 1
        if unused_routes[fork_number] == 0:  # go back to previous fork
            if fork_number != 0:
                fork_number -= 1

    return fork, fork_number, unused_routes, path


def wait_move_end(robot: MyRobot):
    """Stops main loop execution until robot ends move.

    Args:
        robot: MyRobot object with robot devices.

    Returns:
        None
    """

    while True:
        distance_left_now = robot.ps_left.getValue()
        distance_right_now = robot.ps_right.getValue()

        robot.step(world.sim.time_step)

        distance_left_later = robot.ps_left.getValue()
        distance_right_later = robot.ps_right.getValue()

        if (distance_left_now == distance_left_later) and (
            distance_right_now == distance_right_later
        ):
            break
