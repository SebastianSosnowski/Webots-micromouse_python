"""Common functions that can be used in different algorithms."""

from utils.params import RobotState, DetectedWalls, Direction


def init_maze_map_graph(rows: int, cols: int):
    """Initialize maze map with external walls as graph.

    Each position is a list of neighboring positions, that are connected with it.
    Border positions are initialized with respective walls.
    Inside positions are initialized without any walls i.e. 4 neighbors.
    The order of neighbors is following: [N, S, E, W]


    Returns:
        dict: Initialized maze map dictionary.
    """
    maze_map = {}

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
            # print(f"maze_map[{pos}]= {maze_map[pos]}")
    return maze_map


def add_walls_graph(
    maze_map: dict[int, list[int]], rows: int, detected: DetectedWalls, state: RobotState
):
    """Update maze map.

    Remove connected positions with robot position
    and respective neighboring positions according to detected walls.

    Args:
        maze_map: Dictionary with current maze map with walls.
        detected: Information about which walls were detected by robot in current position.
        state: Information about current robot state.
    """
    neighbors = {
        Direction.NORTH: state.pos + rows,
        Direction.SOUTH: state.pos - rows,
        Direction.EAST: state.pos + 1,
        Direction.WEST: state.pos - 1,
    }

    valid_neighbors = maze_map[state.pos]
    # print(f"orientation ({state.orientation}), valid_neighbors ({valid_neighbors})")
    for rel_dir, wall in detected.items():
        if not wall:
            continue

        global_dir = state.relative_to_global(rel_dir)
        neighbor_with_wall = neighbors[global_dir]

        if neighbor_with_wall in valid_neighbors:
            valid_neighbors.remove(neighbor_with_wall)

        if neighbor_with_wall in maze_map and state.pos in maze_map[neighbor_with_wall]:
            maze_map[neighbor_with_wall].remove(state.pos)
    # print(f"orientation ({state.orientation}), valid_neighbors after ({valid_neighbors})")
    maze_map[state.pos] = valid_neighbors


def build_path_to_next_target(
    maze_map: dict[int, list[int]], pos: int, parent: dict[int, int | None], target: int
) -> list[int]:
    if target in maze_map[pos]:
        return [target]

    # current pos parents list
    ancestors = set()
    cur = pos
    while cur is not None:
        ancestors.add(cur)
        cur = parent[cur]

    # find common parent
    path_up = []
    cur = target
    while cur not in ancestors:
        path_up.append(cur)
        cur = parent[cur]
        if cur is None:
            raise ValueError("Target not reachable via parent chain")

    common_ancestor = cur

    # back from current pos to common ancestor
    path_down = []
    cur = pos
    while cur != common_ancestor:
        cur = parent[cur]
        if cur is None:
            raise ValueError("Target not reachable via parent chain")
        path_down.append(cur)

    return path_down + list(reversed(path_up))


def reconstruct_full_path(parent: dict[int, int | None], target: int) -> list[int]:
    """
    Reconstruct full path from start to target using parent map.
    Used after exploration is finished.

    Args:
        target: Target position

    Returns:
        path: Path to target position for robot.
    """
    path = []
    cur = target

    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.pop()  # remove start
    return list(reversed(path))
