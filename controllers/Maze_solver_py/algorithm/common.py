from config.world import world
from utils.params import RobotState, DetectedWalls, Direction


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


def add_walls_graph(maze_map: dict[int, list[int]], detected: DetectedWalls, state: RobotState):
    """Update maze map.

    Remove connected positions with robot position
    and respective neighboring positions according to detected walls.

    Args:
        maze_map: Dictionary with current maze map with walls.
        detected: Information about which walls were detected by robot in current position.
        state: Information about current robot state.
    """

    rows = world.maze.rows

    neighbors = {
        Direction.NORTH: state.pos + rows,
        Direction.SOUTH: state.pos - rows,
        Direction.EAST: state.pos + 1,
        Direction.WEST: state.pos - 1,
    }

    # Only valid neighbors
    valid_neighbors = [pos for pos in neighbors.values() if pos in maze_map]

    for rel_dir, wall in detected.items():
        if not wall:
            continue

        global_dir = state.relative_to_global(rel_dir)
        neighbor_with_wall = neighbors[global_dir]

        if neighbor_with_wall in valid_neighbors:
            valid_neighbors.remove(neighbor_with_wall)

        if neighbor_with_wall in maze_map and state.pos in maze_map[neighbor_with_wall]:
            maze_map[neighbor_with_wall].remove(state.pos)

    maze_map[state.pos] = valid_neighbors
