# Map updating functions
from config.enums import Direction
from config.world import world

from utils.my_robot import MyRobot

# from robot.robot_base import MyRobot


def detect_walls(robot: MyRobot, number_of_reads: int):
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
    for i in range(0, number_of_reads):  # more scans for better accuracy

        for i in sensors_indexes:
            ps_values[i] = robot.ps[i].getValue()
        tof_value = robot.tof.getValue()

        avg2_right_sensor += ps_values[2]

        avg4_back_sensor += ps_values[4]

        avg5_left_sensor += ps_values[5]

        # avg7_front_sensor += ps_values[7]

        avg_front_sensor += tof_value

        robot.step(world.sim.time_step)  # simulation update

    # average score of sensors measurements
    avg2_right_sensor = avg2_right_sensor / number_of_reads
    avg4_back_sensor = avg4_back_sensor / number_of_reads
    avg5_left_sensor = avg5_left_sensor / number_of_reads
    # avg7_front_sensor = avg7_front_sensor / number_of_reads
    avg_front_sensor = avg_front_sensor / number_of_reads

    # Wall detection
    left_wall = avg5_left_sensor > 80.0
    # front_wall = avg7_front_sensor > 80.0
    right_wall = avg2_right_sensor > 80.0
    back_wall = avg4_back_sensor > 80.0
    front_wall = avg_front_sensor < 55  # different bcs its TOF, not IR

    return left_wall, front_wall, right_wall, back_wall


def add_wall(robot: MyRobot, maze_map: list[int], detected_wall: int):
    """Add wall according to distance sensors.
    Depending on robot orientation, value in variable detected_wall
    is changed so it match global directions. Then wall is added
    to maze map on robot field and respective neighboring field.

    Args:
        robot: MyRobot object with robot state.
        maze_map: List with actual maze map with walls.
        detected_wall: Value which indicates on which side of robot wall was detected.

    Returns:
        list: List with updated maze_map.
    """
    robot_position = robot.state.pos
    orientation = robot.state.orientation

    # shift wall value
    if orientation == Direction.EAST:
        if detected_wall != Direction.WEST:
            detected_wall //= 2
        else:
            detected_wall = Direction.NORTH
    elif orientation == Direction.SOUTH:
        if detected_wall == Direction.WEST or detected_wall == Direction.SOUTH:
            detected_wall *= 4
        else:
            detected_wall //= 4
    elif orientation == Direction.WEST:
        if detected_wall != Direction.NORTH:
            detected_wall *= 2
        else:
            detected_wall = Direction.WEST

    maze_map[robot_position] |= detected_wall  # add sensed wall

    # add wall in neighbor field
    if detected_wall == Direction.NORTH:

        robot_position = robot_position + world.maze.columns  # upper field
        check = robot_position in range(0, world.maze.size)

        if check:
            maze_map[robot_position] = maze_map[robot_position] | Direction.SOUTH

    if detected_wall == Direction.EAST:

        robot_position = robot_position + 1  # left field
        check = robot_position in range(0, world.maze.size)

        if check:
            maze_map[robot_position] = maze_map[robot_position] | Direction.WEST

    if detected_wall == Direction.SOUTH:

        robot_position = robot_position - world.maze.columns  # lower field
        check = robot_position in range(0, world.maze.size)

        if check:
            maze_map[robot_position] = maze_map[robot_position] | Direction.NORTH

    if detected_wall == Direction.WEST:

        robot_position = robot_position - 1  # right field
        check = robot_position in range(0, world.maze.size)

        if check:
            maze_map[robot_position] = maze_map[robot_position] | Direction.EAST

    return maze_map


def add_walls_graph_old(
    maze_map: dict[int, list[int]],
    robot_position: int,
    robot_orientation: Direction,
    detected_walls: dict[str, bool],
):
    """Substitute for add_wall for graphs.
    Add connected cells according to detected walls.
    Then remove connected cells in respective neighboring fields according to detected walls.

    Args:
        maze_map: Dictionary with current maze map with walls.
        robot_position: Current robot position in maze.
        robot_orientation: Current robot orientation in global directions.
        detected_walls: Dictionary which indicates on which side of robot wall was detected.

    Returns:
        dict: Updated maze map.
    """

    rows = world.maze.rows

    up = robot_position + rows
    down = robot_position - rows
    left = robot_position - 1
    right = robot_position + 1

    walls = []

    up_in = up in maze_map
    down_in = down in maze_map
    right_in = right in maze_map
    left_in = left in maze_map

    for i in detected_walls.keys():
        if not detected_walls[i]:  # wall absent - add connected cells in node
            match i:
                case "front wall":
                    if robot_orientation == Direction.NORTH:
                        walls.append(up)
                    elif robot_orientation == Direction.EAST:
                        walls.append(right)
                    elif robot_orientation == Direction.SOUTH:
                        walls.append(down)
                    elif robot_orientation == Direction.WEST:
                        walls.append(left)
                case "left wall":
                    if robot_orientation == Direction.NORTH:
                        walls.append(left)
                    elif robot_orientation == Direction.EAST:
                        walls.append(up)
                    elif robot_orientation == Direction.SOUTH:
                        walls.append(right)
                    elif robot_orientation == Direction.WEST:
                        walls.append(down)
                case "right wall":
                    if robot_orientation == Direction.NORTH:
                        walls.append(right)
                    elif robot_orientation == Direction.EAST:
                        walls.append(down)
                    elif robot_orientation == Direction.SOUTH:
                        walls.append(left)
                    elif robot_orientation == Direction.WEST:
                        walls.append(up)
                case "back wall":
                    if robot_orientation == Direction.NORTH:
                        walls.append(down)
                    elif robot_orientation == Direction.EAST:
                        walls.append(left)
                    elif robot_orientation == Direction.SOUTH:
                        walls.append(up)
                    elif robot_orientation == Direction.WEST:
                        walls.append(right)
        else:  # wall present - remove connected cells in neighbor node
            match i:
                case "front wall":
                    if robot_orientation == Direction.NORTH:
                        if (up_in) and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)
                    elif robot_orientation == Direction.EAST:
                        if (right_in) and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)
                    elif robot_orientation == Direction.SOUTH:
                        if (down_in) and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)
                    elif robot_orientation == Direction.WEST:
                        if (left_in) and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)
                case "left wall":
                    if robot_orientation == Direction.NORTH:
                        if (left_in) and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)
                    elif robot_orientation == Direction.EAST:
                        if (up_in) and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)
                    elif robot_orientation == Direction.SOUTH:
                        if (right_in) and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)
                    elif robot_orientation == Direction.WEST:
                        if (down_in) and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)
                case "right wall":
                    if robot_orientation == Direction.NORTH:
                        if (right_in) and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)
                    elif robot_orientation == Direction.EAST:
                        if (down_in) and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)
                    elif robot_orientation == Direction.SOUTH:
                        if (left_in) and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)
                    elif robot_orientation == Direction.WEST:
                        if (up_in) and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)
                case "back wall":
                    if robot_orientation == Direction.NORTH:
                        if (down_in) and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)
                    elif robot_orientation == Direction.EAST:
                        if (left_in) and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)
                    elif robot_orientation == Direction.SOUTH:
                        if (up_in) and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)
                    elif robot_orientation == Direction.WEST:
                        if (right_in) and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)

    maze_map[robot_position] = walls

    return maze_map


def add_walls_graph(
    maze_map: dict[int, list[int]],
    robot_position: int,
    robot_orientation: Direction,
    detected_walls: dict[str, bool],
):
    """Substitute add_wall function for graphs.
    Remove connected cells in robot position and respective neighboring cells according to detected walls.

    Args:
        maze_map: Dictionary with current maze map with walls.
        robot_position: Current robot position in maze.
        robot_orientation: Current robot orientation in global directions.
        detected_walls: Dictionary which indicates on which side of robot wall was detected.

    Returns:
        dict: Updated maze map.
    """

    rows = world.maze.rows

    # neighbors positions
    up = robot_position + rows
    down = robot_position - rows
    left = robot_position - 1
    right = robot_position + 1

    # Checks if neighbors are in maze graph
    up_in_maze = up in maze_map
    down_in_maze = down in maze_map
    right_in_maze = right in maze_map
    left_in_maze = left in maze_map

    walls = [up, down, left, right]

    for i in detected_walls.keys():
        if detected_walls[
            i
        ]:  # wall present - remove connected cell in node and respective neighbor
            match i:
                case "front wall":
                    if robot_orientation == Direction.NORTH:
                        walls.remove(up)

                        if up_in_maze and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)

                    elif robot_orientation == Direction.EAST:
                        walls.remove(right)

                        if right_in_maze and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)

                    elif robot_orientation == Direction.SOUTH:
                        walls.remove(down)

                        if down_in_maze and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)

                    elif robot_orientation == Direction.WEST:
                        walls.remove(left)

                        if left_in_maze and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)

                case "left wall":
                    if robot_orientation == Direction.NORTH:
                        walls.remove(left)

                        if left_in_maze and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)

                    elif robot_orientation == Direction.EAST:
                        walls.remove(up)

                        if up_in_maze and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)

                    elif robot_orientation == Direction.SOUTH:
                        walls.remove(right)

                        if right_in_maze and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)

                    elif robot_orientation == Direction.WEST:
                        walls.remove(down)

                        if down_in_maze and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)

                case "right wall":
                    if robot_orientation == Direction.NORTH:
                        walls.remove(right)

                        if right_in_maze and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)

                    elif robot_orientation == Direction.EAST:
                        walls.remove(down)

                        if down_in_maze and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)

                    elif robot_orientation == Direction.SOUTH:
                        walls.remove(left)

                        if left_in_maze and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)

                    elif robot_orientation == Direction.WEST:
                        walls.remove(up)

                        if up_in_maze and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)

                case "back wall":
                    if robot_orientation == Direction.NORTH:
                        walls.remove(down)

                        if down_in_maze and (robot_position in maze_map[down]):
                            maze_map[down].remove(robot_position)

                    elif robot_orientation == Direction.EAST:
                        walls.remove(left)

                        if left_in_maze and (robot_position in maze_map[left]):
                            maze_map[left].remove(robot_position)

                    elif robot_orientation == Direction.SOUTH:
                        walls.remove(up)

                        if up_in_maze and (robot_position in maze_map[up]):
                            maze_map[up].remove(robot_position)

                    elif robot_orientation == Direction.WEST:
                        walls.remove(right)

                        if right_in_maze and (robot_position in maze_map[right]):
                            maze_map[right].remove(robot_position)

    maze_map[robot_position] = walls

    return maze_map


def init_maze_map(maze_map: list[int]):
    """Initialize maze map with external walls.

    Args:
        maze_map: List which contains maze map values.

    Returns:
        list: Initialized maze map list.
    """
    maze_map[0] = maze_map[0] | world.maze.visited  # mark start as visited

    for i in range(0, 16):
        maze_map[i] = maze_map[i] | Direction.SOUTH

    for i in range(240, 256):
        maze_map[i] = maze_map[i] | Direction.NORTH

    for i in range(0, 241, 16):
        maze_map[i] = maze_map[i] | Direction.WEST

    for i in range(15, 256, 16):
        maze_map[i] = maze_map[i] | Direction.EAST

    # print_array(maze_map, 0)

    return maze_map


def init_maze_map_graph():
    """Initialize maze map with external walls as graph.
    Border cells are initialized with respective walls.
    Inside cells are initialized without any walls i.e. 4 connections.

    Returns:
        dict: Initialized maze map dictionary.
    """

    maze_map = {}
    rows = world.maze.rows
    cols = world.maze.columns
    size = rows * cols
    left_down_corner = 0
    right_down_corner = rows - 1
    left_up_corner = rows * (cols - 1)
    right_up_corner = size - 1

    # corners
    maze_map[left_down_corner] = [rows, 1]
    maze_map[right_down_corner] = [right_down_corner + rows, right_down_corner - 1]
    maze_map[left_up_corner] = [left_up_corner + 1, left_up_corner - rows]
    maze_map[right_up_corner] = [right_up_corner - rows, right_up_corner - 1]

    # down wall cells
    for cell in range(left_down_corner + 1, right_down_corner):
        maze_map[cell] = [cell + rows, cell + 1, cell - 1]
    # up wall cells
    for cell in range(left_up_corner + 1, right_up_corner):
        maze_map[cell] = [cell + 1, cell - rows, cell - 1]
    # left wall cells
    for cell in range(left_down_corner + rows, left_up_corner, 16):
        maze_map[cell] = [cell + rows, cell + 1, cell - rows]
    # right wall cells
    for cell in range(right_down_corner + rows, right_up_corner, 16):
        maze_map[cell] = [cell + rows, cell - rows, cell - 1]

    cell = 17
    # inside cells
    while True:

        maze_map[cell] = [cell + rows, cell + 1, cell - rows, cell - 1]

        end = cell == (right_up_corner - 1 - rows)
        if end:
            break

        row_end = (cell % rows) == 14  # without border cells

        if row_end:  # next row
            cell += 3
        else:  # next column
            cell += 1

    return maze_map


def init_distance_map(distance: list[int], target: int):
    """Initialize distance map with max values and 0 as target.
    Target is 0 for floodfill algorithm working properly.

    Args:
        distance: List which contains distance values.
        target: Value which contains targeted cell.

    Returns:
        list: Initialized distance list.
    """
    distance = [world.maze.size - 1] * world.maze.size
    distance[target] = 0

    return distance


def print_array(list: list[int], action: int):
    """Print 256 element list as 16x16 in terminal.

    Args:
        list: List which contains map or distance values.
        action: Value which indicates to print map walls without visited cells.

    Returns:
        None
    """

    print("")
    if action == 0:  # just print array

        index = 240
        row_index = 0
        while index >= 0:
            print("{0:>3}".format(list[index]), end=" ")
            if row_index == 15:
                print("\n")
                index -= 31  # 32 - 1 cuz no i++ in this loop iteration
                row_index = 0
            else:
                row_index += 1
                index += 1
    elif action == 1:  # print array without visited mark to read just walls

        list_temp = list.copy()

        for index in range(len(list_temp)):
            if list_temp[index] and world.maze.visited:
                list_temp[index] -= 64  # version to avoid negative values(errors etc.)

        index = 240
        row_index = 0
        while index >= 0:
            print("{0:>3}".format(list_temp[index]), end=" ")
            if row_index == 15:
                print("\n")
                index -= 31  # 32 - 1 cuz no i++ in this loop iteration
                row_index = 0
            else:
                row_index += 1
                index += 1
