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

    for row in range(rows):
        for col in range(cols):
            pos = row * cols + col
            neighbors: list[int] = []

            if row < rows - 1:  # NORTH
                neighbors.append((row + 1) * cols + col)
            if row > 0:  # SOUTH
                neighbors.append((row - 1) * cols + col)
            if col < cols - 1:  # EAST
                neighbors.append(row * cols + (col + 1))
            if col > 0:  # WEST
                neighbors.append(row * cols + (col - 1))
            maze_map[pos] = neighbors
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
