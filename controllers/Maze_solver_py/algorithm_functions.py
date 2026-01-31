# algorithm related functions
from collections import deque
from pathlib import Path
import pickle

from config.enums import Algorithms, Direction, MazeLayout, Move
from config.world import world
from map_functions import init_distance_map

from utils.my_robot import MyRobot

# from robot.robot_base import MyRobot


def floodfill(maze_map: list[int], distance: list[int]):
    """Floodfill algorithm which calculates shortest path to actual target based on actual maze map.

    Args:
        maze_map: List with actual maze map with walls.
        distance: List with actual distances values/path.

    Returns:
        list: Updated distance list.
    """

    search = True

    while search:
        search = False

        for i in range(0, world.maze.size):
            if distance[i] < 255:

                if (maze_map[i] & Direction.NORTH) != Direction.NORTH:
                    if distance[i + world.maze.columns] == 255 or (
                        (distance[i] + 1) < distance[i + world.maze.columns]
                    ):
                        # update distance value on north tile
                        distance[i + world.maze.columns] = distance[i] + 1
                        search = True

                if (maze_map[i] & Direction.EAST) != Direction.EAST:
                    if distance[i + 1] == 255 or ((distance[i] + 1) < distance[i + 1]):
                        distance[i + 1] = distance[i] + 1  # update distance value on EAST tile
                        search = True

                if (maze_map[i] & Direction.WEST) != Direction.WEST:
                    if distance[i - 1] == 255 or ((distance[i] + 1) < distance[i - 1]):
                        distance[i - 1] = distance[i] + 1  # update distance value on WEST tile
                        search = True
                # prop unnecessary cuz robot doesn't move backward
                if (maze_map[i] & Direction.SOUTH) != Direction.SOUTH:
                    if distance[i - world.maze.columns] == 255 or (
                        (distance[i] + 1) < distance[i - world.maze.columns]
                    ):
                        distance[i - world.maze.columns] = (
                            distance[i] + 1
                        )  # update distance value on SOUTH tile
                        search = True

    # print('\n Path ')
    # print_array(distance, 0)
    # print(' Path ')

    return distance


def where_to_move(robot: MyRobot, walls: int, distance: list[int]):
    """Decide where to move by checking distance values in neighbors cells.

    Depending on robot orientation, value in variable move_direction
    is changed so it match global directions.

    Args:
        robot: MyRobot object with robot state.
        walls: Variable with walls in current robot position.
        distance: List with actual distances values/path.

    Returns:
        Direction: Variable with move direction to do.
    """

    position = robot.state.pos
    orientation = robot.state.orientation
    best_neighbor = 255
    move_direction: Direction = Direction.NORTH

    if (walls & Direction.NORTH) != Direction.NORTH:

        if distance[position + world.maze.columns] <= best_neighbor:

            if distance[position + world.maze.columns] < best_neighbor:

                best_neighbor = distance[position + world.maze.columns]
                move_direction = Direction.NORTH

            elif orientation == Direction.NORTH:
                move_direction = Direction.NORTH

    if (walls & Direction.EAST) != Direction.EAST:

        if distance[position + 1] <= best_neighbor:

            if distance[position + 1] < best_neighbor:

                best_neighbor = distance[position + 1]
                move_direction = Direction.EAST

            elif orientation == Direction.EAST:
                move_direction = Direction.EAST

    if (walls & Direction.SOUTH) != Direction.SOUTH:

        if distance[position - world.maze.columns] <= best_neighbor:

            if distance[position - world.maze.columns] < best_neighbor:

                best_neighbor = distance[position - world.maze.columns]
                move_direction = Direction.SOUTH

            elif orientation == Direction.SOUTH:
                move_direction = Direction.SOUTH

    if (walls & Direction.WEST) != Direction.WEST:

        if distance[position - 1] <= best_neighbor:

            if distance[position - 1] < best_neighbor:

                best_neighbor = distance[position - 1]
                move_direction = Direction.WEST

            elif orientation == Direction.WEST:
                move_direction = Direction.WEST

    return move_direction


def where_to_move_graph(robot_position: int, current_destination: int):
    """Substitute of where_to_move function made for graphs. WORKS only for 16x16 maze.

    Decide global move direction by calculating where, in reference to actual position node,
    is placed current destination node.

    Args:
        robot_position: Variable with actual robot position in maze.
        current_destination: Variable with position to which robot want's to go.

    Returns:
        Direction: Variable with move direction to do.
    """

    x = current_destination - robot_position
    match x:
        case -16:
            move_direction = Direction.SOUTH
        case -1:
            move_direction = Direction.WEST
        case 1:
            move_direction = Direction.EAST
        case 16:
            move_direction = Direction.NORTH

    return move_direction


def check_possible_routes_BFS(
    adjacent_cells: list[int], visited: list[int], queue: deque, fork: bool
):
    """Add possible adjacent cells to queue and then decides to which cell move next.

    First item in queue is chosen when robot current position is
    a fork or dead end (breadth first search). Otherwise last item in queue is chosen.

    Args:
        adjacent_cells: List with cells accessible from current robot position.
        visited: List with cells already added to queue.
        queue: Queue with cells which will be visited.
        fork: Bool variable which informs if current cell is a fork.

    Returns:
        tuple: (current_destination, visited, queue, dead_end, searching_end)
            - current_destination: Variable with a cell to which move next.
            - visited: Updated list with cells already added to queue.
            - queue: Updated queue with cells which will be visited.
            - dead_end (bool): Bool variable which informs if current cell is a dead end and robot need's to move back.
            - searching_end (bool): Bool variable which informs if target was found i.e. run is ended.
    """
    dead_end = True
    searching_end = False
    for cell in adjacent_cells:
        if cell not in visited:
            visited.append(cell)
            queue.append(cell)
            dead_end = False
            if cell == world.maze.target_cell:
                searching_end = True
                break

    if searching_end:
        current_destination = queue[-1]
    elif fork or dead_end:
        current_destination = queue[0]
    else:
        current_destination = queue[-1]

    return current_destination, visited, queue, dead_end, searching_end


def check_possible_routes_DFS(adjacent_cells: list[int], visited: list[int], stack: list[int]):
    """Add possible adjacent cells to stack and then decides to which cell move next.

    Last item in stack is chosen.

    Args:
        adjacent_cells: List with cells accessible from current robot position.
        visited: List with cells already added to stack.
        stack: Stack with cells which will be visited.

    Returns:
        tuple: (current_destination, visited, stack)
            - current_destination: Variable with a cell to which move next.
            - visited: Updated list with cells already added to stack.
            - stack: Updated stack with cells which will be visited.
    """

    for cell in reversed(adjacent_cells):
        if cell not in visited:
            visited.append(cell)
            stack.append(cell)
            if cell == world.maze.target_cell:
                break

    current_destination = stack[-1]

    return current_destination, visited, stack

    visited = []
    stack = []
    path = []
    parent = {}
    visited.append(start)
    stack.append(start)
    parent[start] = start
    search_end = False

    while stack:
        # popleft is O(1)
        s = stack.pop()

        if search_end:
            break

        if s in graph:  # to not add nodes to queue which are not in graph
            for n in graph[s]:
                if n not in visited:
                    visited.append(n)
                    stack.append(n)
                    parent[n] = s
                    if n == target:
                        while parent[n] != n:
                            path.append(n)
                            n = parent[n]
                        search_end = True
                        break
    return path


def get_path_A_star(maze_map: dict, start: int, target: int):
    """A* algorithm function which is used to create path between 2 cells.

    It is used to determine final path as well as paths between current position
    and current destination in main program.

    Args:
        maze_map: Dictionary with discovered maze map (or any graph).
        start: Variable with starting cell number.
        target: Variable with targeted cell number.

    Returns:
        list: Path from start to target.
    """
    # A* vars
    open = []  # list of unvisited nodes
    closed = []  # list of visited nodes
    cost = {}
    parent = {}
    path = []
    current_position = start
    cost[start] = [0, calc_cost(start, target)]
    parent[start] = start
    open.append(start)

    while open:

        open.remove(current_position)
        closed.append(current_position)

        open, parent, cost = update_neighbors_costs(
            maze_map[current_position], open, closed, parent, cost, current_position
        )

        if current_position == target:
            while current_position != start:
                path.append(current_position)
                current_position = parent[current_position]
            path.reverse()
            return path

        current_position = check_possible_routes_A_star(open, cost)

    return []


def check_possible_routes_A_star(open: list[int], cost: dict[int, list[int]]):
    """Decides to which cell move next. TODO Implement heap to make it much faster.

    Cell with the lowest overall cost (Fcost) is chosen. If more cells have equal
    Fcost, then cell with lowest Hcost (closer to target) is chosen.

    Args:
        open: List with cells to visit.
        cost: Dictionary of costs, which contain Gcost and Hcost.

    Returns:
        int: Variable with a cell to which move next.
    """
    current_destination = open[
        -1
    ]  # if there are 2 cells with same Fcost and Hcost, pick last added to open i.e. neighbor
    for i in open:
        Fcost_i = cost[i][0] + cost[i][1]
        Fcost_curr = cost[current_destination][0] + cost[current_destination][1]
        if (Fcost_i < Fcost_curr) or (
            Fcost_i == Fcost_curr and cost[i][1] < cost[current_destination][1]
        ):
            current_destination = i

    return current_destination


def update_neighbors_costs(
    neighbors: list[int],
    open: list[int],
    closed: list[int],
    parent: dict[int, int],
    cost: dict[int, list[int]],
    current_position: int,
):
    """Used in A* algorithm. Assign and/or update costs of neighbor nodes.

    New cost is assigned to node when it's not in open list or new cost is lower than actual.
    In addition parent of the node is assigned, which allows to create path later.
    If target is found, function breaks.

    Args:
        neighbors: List with cells adjacent to current position.
        open: List with cells to visit.
        closed: List with cells already visited.
        parent: Dictionary with parent nodes used to create path.
        cost: Dictionary of costs, which contain Gcost and Hcost.
        current_position: Variable with current robot position.

    Returns:
        tuple: (open, parent, cost)
            - open: Updated list with cells to visit.
            - parent: Updated dictionary with parent nodes used to create path.
            - cost: Updated dictionary of costs, which contain Gcost and Hcost.
    """
    for neighbor in neighbors:
        if neighbor in closed:
            continue

        new_move_to_neighbor_cost = cost[current_position][0] + calc_cost(
            current_position, neighbor
        )

        # if (neighbor not in open) or new_cost < (cost[neighbor][0] + cost[neighbor][1]):
        if (neighbor not in open) or new_move_to_neighbor_cost < cost[neighbor][0]:
            neighbor_Gcost = new_move_to_neighbor_cost
            neighbor_Hcost = calc_cost(neighbor, world.maze.target_cell)
            cost[neighbor] = [neighbor_Gcost, neighbor_Hcost]
            parent[neighbor] = current_position

            if neighbor not in open:
                open.append(neighbor)

            if neighbor == world.maze.target_cell:
                break

    return open, parent, cost


def get_back_path_A_star(maze_map: dict, target: int, robot_position: int, parent: list[int]):
    """Creates a path from current robot position to its current destination. NOT USED anymore, but kept in code :).

    First path from target to start is created. Then path from current position to start is created.
    If path to current target wasn't found yet, both paths are combined to create path.

    Args:
        maze_map: Dictionary graph with current known maze configuration.
        target: Variable with position to which robot want's to go.
        robot_position: Variable with current robot position in maze.
        parent: List with parent nodes used to create path.

    Returns:
        list: Path to target.
    """
    path_to_target = []
    path_to_current = []
    path = []
    node = target
    while True:  # check path from target
        path_to_target.append(node)

        if node in maze_map[robot_position]:  # path found
            return path_to_target

        if parent[node] == node:  # node is start position
            break
        node = parent[node]

    node = robot_position
    while True:  # check path from current position
        path_to_current.append(node)

        if target in maze_map[node]:  # path found
            path_to_current.append(target)
            path_to_current.pop(0)
            path_to_current.reverse()
            return path_to_current

        if parent[node] == node:  # node is start position
            break
        node = parent[node]

    # combine both paths to get final path
    while path_to_target[-1] == path_to_current[-1]:
        last = path_to_target[-1]
        path_to_target.pop()
        path_to_current.pop()

    path_to_target.append(last)
    path_to_current.reverse()
    path_to_current.pop()

    path = path_to_target + path_to_current

    return path


def check_fork_DFS(
    connections: list[int],
    robot_position: int,
    fork: dict[int, list[int]],
    fork_number: int,
    fork_count: dict[int, int],
):
    """Detect's fork, assign number to it and monitor possible unused routes from each one which is used for DFS algorithm operation.

    Also detect's dead-ends.

    Args:
        connections: List with cells available from current position.
        robot_position: Variable with current robot position in maze.
        fork: Dictionary with paths to each fork from current position.
        fork_number: Variable with number of last used fork.
        fork_count: Dictionary with number of unused routes for each fork.

    Returns:
        tuple: (fork, fork_number, fork_count, dead_end)
            - fork: Updated fork dictionary.
            - fork_number: Updated fork number.
            - fork_count: Updated fork count.
            - dead_end (bool): Bool variable with info if current cell is dead-end.
    """

    dead_end = False

    routes = len(connections)
    if routes >= 3:
        fork_number += 1

    if fork_number > -1:
        if routes >= 3:
            fork[fork_number] = [robot_position]
            if routes == 3:
                fork_count[fork_number] = 1
            else:
                fork_count[fork_number] = 2
        elif routes == 2:
            fork[fork_number].append(robot_position)
        else:
            dead_end = True

    return fork, fork_number, fork_count, dead_end


def get_path_BFS(graph: dict[int, list[int]], start: int, current_target: int):
    """Creates a path from current robot position to its current destination.

    It is used when current destination is not in adjacent cell.

    Args:
        graph: Dictionary graph with visited cells.
        start: Variable with current robot position in maze.
        current_target: Variable with position to which robot want's to go.

    Returns:
        list: Path to target.
    """

    visited = []
    queue = deque()
    path = []
    parent = {}
    visited.append(start)
    queue.append(start)
    parent[start] = start
    search_end = False

    while queue:
        # popleft is O(1)
        s = queue.popleft()

        if search_end:
            break

        if s in graph:  # to not add nodes to queue which are not in graph
            for n in graph[s]:
                if n not in visited:
                    visited.append(n)
                    queue.append(n)
                    parent[n] = s
                    if n == current_target:
                        while n != start:  # or parent[n] != n
                            path.append(n)
                            n = parent[n]
                        search_end = True
                        break
    return path


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

    if world.sim.testing:
        print("Orientation:", Direction(orientation_value))

    return Direction(orientation_value)


def change_position(robot_position: int, robot_orientation: Direction):
    """Update position of the robot basing on current orientation of the robot.

    Args:
        robot_position: Variable with robot position.
        robot_orientation: Variable with actual robot orientation in global directions.

    Returns:
        int: Variable with updated robot position.
    """

    if robot_orientation == Direction.NORTH:
        robot_position = robot_position + world.maze.columns

    elif robot_orientation == Direction.EAST:
        robot_position = robot_position + 1
    elif robot_orientation == Direction.SOUTH:
        robot_position = robot_position - world.maze.columns

    elif robot_orientation == Direction.WEST:
        robot_position = robot_position - 1

    return robot_position


def change_target(robot: MyRobot, maze_map: list[int], distance: list[int]):
    """Marks every visited cell, after reaching targeted cell, change cell to first unvisited cell.

    When reaching final target, saves distance map to file.

    Args:
        robot: MyRobot object with robot state.
        maze_map: List with actual maze map with walls.
        distance: List with actual distances values/path.

    Returns:
        tuple: (maze_map, shortest_path)
            - maze_map: Updated maze map.
            - shortest_path (bool): Bool variable which informs if shortest path was found.
    """
    distance = init_distance_map(distance, robot.state.current_target)  # reset path
    distance = floodfill(maze_map, distance)  # path

    # fill unvisited cells with 4 walls to verify if the shortest path was find
    shortest_path = check_distance(distance, maze_map, robot.state.current_target)

    if robot.state.pos == world.maze.target_cell:

        maze_map = mark_center(maze_map)

        # distance = init_distance_map(distance, target) #reset path
        # distance = floodfill(maze_map, distance) #path

        # #fill unvisited cells with 4 walls to verify if the shortest path was find
        # shortest_path = check_distance(distance, maze_map, target)

        if shortest_path:
            print("This is the shortest/ one of the shortest paths")
        else:
            print("There might be a shorter path, keep going")
            robot.state.current_target = world.maze.start_cell

    elif robot.state.pos == world.maze.start_cell:
        # shortest_path = check_distance(distance, maze_map, target)

        if shortest_path:
            print("This is the shortest/ one of the shortest paths")
        else:
            print("There might be a shorter path, keep going")

        robot.state.current_target = world.maze.target_cell

    return maze_map, shortest_path


def mark_center(maze_map: list[int]):
    """Adds walls to unvisited cells in center.

    Args:
        maze_map: List with actual maze map with walls.

    Returns:
        list: List with updated maze map.
    """

    center = [119, 120, 135]

    for center_cell in center:
        if (maze_map[center_cell] & world.maze.visited) != world.maze.visited:
            match center_cell:
                case 119:
                    maze_map[center_cell] = 3
                    maze_map[center_cell - 1] |= Direction.EAST
                    maze_map[center_cell - 16] |= Direction.NORTH
                case 120:
                    maze_map[center_cell] = 6
                    maze_map[center_cell + 1] |= Direction.WEST
                    maze_map[center_cell - 16] |= Direction.NORTH
                case 135:
                    maze_map[center_cell] = 9
                    maze_map[center_cell - 1] |= Direction.EAST
                    maze_map[center_cell + 16] |= Direction.SOUTH

    return maze_map


def mark_center_graph(maze_map: dict[int, list[int]], path: list[int]):
    """Adds walls to unvisited cells in center.

    Path is used to determine which cells weren't visited.

    Args:
        maze_map: Dictionary with actual maze map with walls.
        path: List with path.

    Returns:
        dict: Dictionary with updated maze map.
    """

    center = [119, 120, 135]
    rows = world.maze.rows

    for center_cell in center:
        up = center_cell + rows
        down = center_cell - rows
        left = center_cell - 1
        right = center_cell + 1
        match center_cell:
            case 119:
                if center_cell not in path:
                    maze_map[center_cell] = [center_cell + 1, center_cell + rows]
            case 120:
                if center_cell not in path:
                    maze_map[center_cell] = [center_cell - 1, center_cell + rows]
            case 135:
                if center_cell not in path:
                    maze_map[center_cell] = [center_cell + 1, center_cell - rows]

    return maze_map


def check_distance(distance: list[int], maze_map: list[int], target: int):
    """Fills unvisited cells with 4 walls to verify if the shortest path was find.

    Args:
        distance: List with actual distances values/path.
        maze_map: List with actual maze map with walls.
        target: Variable with field number to which robot tries to get.

    Returns:
        bool: Bool variable which informs if shortest path was found.
    """

    distance_check = distance.copy()
    maze_map_check = maze_map.copy()

    for i in range(0, world.maze.size):
        if (maze_map_check[i] & world.maze.visited) != world.maze.visited:
            maze_map_check[i] = maze_map_check[i] | 15 | world.maze.visited

    distance_check = init_distance_map(distance_check, target)  # reset path
    distance_check = floodfill(maze_map_check, distance_check)  # path
    if target == 136:
        shortest_path = distance[0] >= distance_check[0]  # could be just equal'
    elif target == 0:
        shortest_path = distance[136] >= distance_check[136]  # could be just equal'

    return shortest_path


def calc_cost(start: int, target: int):
    """Calculates Manhattan's distance which is used as cost in A* algorithm.

    Args:
        start: Variable with first position.
        target: Variable with second position.

    Returns:
        int: Manhattan's distance/cost.
    """

    # index to matrix/grid
    point1 = [start % 16, start // 16]
    point2 = [target % 16, target // 16]

    distance = 0
    for x1, x2 in zip(point1, point2):
        difference = x2 - x1
        absolute_difference = abs(difference)
        distance += absolute_difference

    return distance


def read_file(path: Path):
    """Read a pickle file.

    Args:
        path: Path to the file.

    Returns:
        Content of a file.
    """
    with path.open("rb") as file:
        return pickle.load(file)


def write_file(path: Path, values):
    """Write data to a pickle file.

    Args:
        path: Path to the file.
        values: Any type of object with a content to write file.

    Returns:
        None
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as file:
        pickle.dump(values, file)


def create_files_directories(layout: MazeLayout, algorithm: Algorithms) -> tuple[Path, Path, Path]:
    """Create appropriate directory and file name to save results.

    Args:
        layout: Maze layout enumeration used in simulation.
        algorithm: Algorithm enumeration used in simulation.

    Returns:
        tuple[Path, Path]: Appropriate files directory and names (path_file, map_file, values_file).
    """
    base_dir = Path("Results")
    maze_dir = layout.name.capitalize()
    algo_name = algorithm.name.lower()
    path_dir = base_dir / maze_dir / f"{algo_name}_path.pkl"
    map_dir = base_dir / maze_dir / f"{algo_name}_maze.pkl"
    values_dir = base_dir / maze_dir / f"{algo_name}_algorithm_values.pkl"

    return path_dir, map_dir, values_dir


def save_results(path: list[int], maze_map: list | dict, values: list | dict):
    path_dir, map_dir, values_dir = create_files_directories(
        world.sim.maze_layout, world.sim.algorithm
    )
    write_file(map_dir, maze_map)
    write_file(path_dir, path)
    write_file(values_dir, values)


def read_results() -> tuple[list[int], list | dict, list | dict]:
    path_dir, map_dir, values_dir = create_files_directories(
        world.sim.maze_layout, world.sim.algorithm
    )
    path = read_file(path_dir)
    maze_map = read_file(map_dir)
    values = read_file(values_dir)

    return path, maze_map, values
