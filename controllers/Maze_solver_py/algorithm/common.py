from config.world import world

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