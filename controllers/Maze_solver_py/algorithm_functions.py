# Algorythm related functions
import pickle
from collections import deque

from Constants import *
from map_functions import init_distance_map
import var


""" floodfill
# @brief Floodfill algorithm which calculates shortest path to actual target based on actual maze map.
#
# @param maze_map: list with actual maze map with walls
# @param distance: list with actual distances values/path
#
# @retval move_direction: variable with move direction to do
"""


def floodfill(maze_map, distance):

    search = True

    while search:
        search = False

        for i in range(0, maze_parameters.MAZE_SIZE):
            if distance[i] < 255:

                if (maze_map[i] & direction.NORTH) != direction.NORTH:
                    if distance[i + maze_parameters.COLUMNS] == 255 or (
                        (distance[i] + 1) < distance[i + maze_parameters.COLUMNS]
                    ):
                        distance[i + maze_parameters.COLUMNS] = (
                            distance[i] + 1
                        )  # update distance value on north tile
                        search = True

                if (maze_map[i] & direction.EAST) != direction.EAST:
                    if distance[i + 1] == 255 or ((distance[i] + 1) < distance[i + 1]):
                        distance[i + 1] = (
                            distance[i] + 1
                        )  # update distance value on EAST tile
                        search = True

                if (maze_map[i] & direction.WEST) != direction.WEST:
                    if distance[i - 1] == 255 or ((distance[i] + 1) < distance[i - 1]):
                        distance[i - 1] = (
                            distance[i] + 1
                        )  # update distance value on WEST tile
                        search = True
                # prop unnecessary cuz robot doesnt move backward
                if (maze_map[i] & direction.SOUTH) != direction.SOUTH:
                    if distance[i - maze_parameters.COLUMNS] == 255 or (
                        (distance[i] + 1) < distance[i - maze_parameters.COLUMNS]
                    ):
                        distance[i - maze_parameters.COLUMNS] = (
                            distance[i] + 1
                        )  # update distance value on SOUTH tile
                        search = True

    # print('\n Path ')
    # print_array(distance, 0)
    # print(' Path ')

    return distance


""" where_to_move
# @brief Decide where to move by checking distance values in neighbours cells.
# Depending on robot orientation, value in variable move_direction 
# is changed so it match global directions. 
#
# @param walls: variable with walls in current robot position
# @param robot_position: variable with actual robot position in maze
# @param distance: list with actual distances values/path
# @param robot_orientation: variable with actual robot orientation in global directions
#
# @retval move_direction: variable with move direction to do
"""


def where_to_move(walls, robot_position, distance, robot_orientation):

    best_neighbour = 255
    move_direction = direction.NORTH

    if (walls & direction.NORTH) != direction.NORTH:

        if distance[robot_position + maze_parameters.COLUMNS] <= best_neighbour:

            if distance[robot_position + maze_parameters.COLUMNS] < best_neighbour:

                best_neighbour = distance[robot_position + maze_parameters.COLUMNS]
                move_direction = direction.NORTH

            elif robot_orientation == direction.NORTH:
                move_direction = direction.NORTH

    if (walls & direction.EAST) != direction.EAST:

        if distance[robot_position + 1] <= best_neighbour:

            if distance[robot_position + 1] < best_neighbour:

                best_neighbour = distance[robot_position + 1]
                move_direction = direction.EAST

            elif robot_orientation == direction.EAST:
                move_direction = direction.EAST

    if (walls & direction.SOUTH) != direction.SOUTH:

        if distance[robot_position - maze_parameters.COLUMNS] <= best_neighbour:

            if distance[robot_position - maze_parameters.COLUMNS] < best_neighbour:

                best_neighbour = distance[robot_position - maze_parameters.COLUMNS]
                move_direction = direction.SOUTH

            elif robot_orientation == direction.SOUTH:
                move_direction = direction.SOUTH

    if (walls & direction.WEST) != direction.WEST:

        if distance[robot_position - 1] <= best_neighbour:

            if distance[robot_position - 1] < best_neighbour:

                best_neighbour = distance[robot_position - 1]
                move_direction = direction.WEST

            elif robot_orientation == direction.WEST:
                move_direction = direction.WEST

    return move_direction


""" where_to_move_graph  WORKS only for 16x16 maze
# @brief Substitute of where_to_move function made for graphs.
# Decide global move direction by calculating where, in reference to actual position node,
# is placed current destination node.
#
# @param robot_position: variable with actual robot position in maze
# @param current_destination: variable with position to which robot want's to go
#
# @retval move_direction: variable with move direction to do
"""


def where_to_move_graph(robot_position, current_destination):

    x = current_destination - robot_position
    match x:
        case -16:
            move_direction = direction.SOUTH
        case -1:
            move_direction = direction.WEST
        case 1:
            move_direction = direction.EAST
        case 16:
            move_direction = direction.NORTH

    return move_direction


""" check_possible_routes_BFS  
# @brief Add possible adjacent cells to queue and then decides to which cell move next.
# First item in queue is chosen when robot current position is
# a fork or dead end (breadth first search). Otherwise last item in queue is chosen.
#
# @param adjacent_cells: list with cells accesible from current robot position
# @param visited: list with cells already added to queue
# @param queue: queue with cells which will be visited
# @param fork: bool variable which informs if current cell is a fork
# @param target: variable with a cell which is a target
#
# @retval current_destination: variable with a cell to which move next
# @retval visited: updated list with cells already added to queue
# @retval queue: updated queue with cells which will be visited
# @retval deadend: bool variable which informs if current cell is a dead end and robot need's to move back
# @retval searching_end: bool variable which informs if target was found i.e. run is ended
"""


def check_possible_routes_BFS(adjacent_cells, visited, queue, fork, target):
    deadend = True
    searching_end = False
    for cell in adjacent_cells:
        if cell not in visited:
            visited.append(cell)
            queue.append(cell)
            deadend = False
            if cell == target:
                searching_end = True
                break

    if searching_end:
        current_destination = queue[-1]
    elif fork or deadend:
        current_destination = queue[0]
    else:
        current_destination = queue[-1]

    return current_destination, visited, queue, deadend, searching_end


""" check_possible_routes_DFS  
# @brief Add possible adjacent cells to stack and then decides to which cell move next.
# Last item in stack is chosen.
#
# @param adjacent_cells: list with cells accesible from current robot position
# @param visited: list with cells already added to stack
# @param stack: stack with cells which will be visited
# @param target: variable with a cell which is a target
#
# @retval current_destination: variable with a cell to which move next
# @retval visited: updated list with cells already added to queue
# @retval stack: updated stack with cells which will be visited
"""


def check_possible_routes_DFS(adjacent_cells, visited, stack, target):

    for cell in reversed(adjacent_cells):
        if cell not in visited:
            visited.append(cell)
            stack.append(cell)
            if cell == target:
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


""" A_star  
# @brief A* algorithm function which is used to create path between 2 cells.
# It is used to determine final path as well as paths between current position 
# and current destination in main program.
#
# @param maze_map: dictionary with discovered maze map (or any graph)
# @param start: variable with starting cell number
# @param target: variable with targeted cell number
#
# @retval path: list withpath from start to target
"""


def get_path_A_star(maze_map, start, target):
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


""" check_possible_routes_A_star TODO Implement heap to make it much faster.
# @brief Decides to which cell move next.
# Cell with the lowest overall cost (Fcost) is chosen. If more cells have equal
# Fcost, then cell with lowest Hcost (closer to target) is chosen.
#
# @param open: list with cells to visit
# @param cost: list of costs, which contain Gcost and Hcost
#
# @retval current_destination: variable with a cell to which move next
"""


def check_possible_routes_A_star(open, cost):
    current_destination = open[
        -1
    ]  # if there are 2 cells with same Fcost and Hcost, pick last added to open i.e. neighbour
    for i in open:
        Fcost_i = cost[i][0] + cost[i][1]
        Fcost_curr = cost[current_destination][0] + cost[current_destination][1]
        if (Fcost_i < Fcost_curr) or (
            Fcost_i == Fcost_curr and cost[i][1] < cost[current_destination][1]
        ):
            current_destination = i

    return current_destination


""" update_neighbors_costs  
# @brief Used in A* algorithm. Assign and/or update costs of neighbor nodes.
# New cost is assigned to node when it's not in open list or new cost is lower than actual.
# In addition parent of the node is assigned, which allows to create path later.
# If target is found, function breaks.
#
# @param neighbors: list with cells adjacent to current position
# @param open: list with cells to visit
# @param closed: list with cells already visited
# @param parent: dictionary with parent nodes used to create path
# @param cost: dictionary of costs, which contain Gcost and Hcost
# @param current_position: variable with current robot position
#
# @retval open: updated list with cells to visit
# @retval parent: updated dictionary with parent nodes used to create path
# @retval cost: updated dictionary of costs, which contain Gcost and Hcost
"""


def update_neighbors_costs(neighbors, open, closed, parent, cost, current_position):
    for neighbor in neighbors:
        if neighbor in closed:
            continue

        new_move_to_neighbor_cost = cost[current_position][0] + calc_cost(
            current_position, neighbor
        )

        # if (neighbor not in open) or new_cost < (cost[neighbor][0] + cost[neighbor][1]):
        if (neighbor not in open) or new_move_to_neighbor_cost < cost[neighbor][0]:
            neighbor_Gcost = new_move_to_neighbor_cost
            neighbor_Hcost = calc_cost(neighbor, maze_parameters.TARGET_CELL)
            cost[neighbor] = [neighbor_Gcost, neighbor_Hcost]
            parent[neighbor] = current_position

            if neighbor not in open:
                open.append(neighbor)

            if neighbour == maze_parameters.TARGET_CELL:
                break

    return open, parent, cost


""" get_back_path_A_star  NOT USED anymore, but kept in code :).
# @brief Creates a path from current robot position to its current destination.
# First path from target to start is created. Then path from current position to start is created.
# If path to current target wasn't found yet, both paths are combined to create path.
#
# @param maze_map: dictionary graph with current known maze configuration
# @param robot_position: variable with current robot position in maze
# @param target: variable with position to which robot want's to go
# @param parent: list with parent nodes used to create path
#
# @retval path: list with path to target
"""


def get_back_path_A_star(maze_map, target, robot_position, parent):
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


""" check_fork_DFS  
# @brief Detect's fork, assign number to it and 
# monitor possible unused routes from each one which is used for DFS algorithm operation.
# Also detect's dead-ends.
#
# @param connections: list with cells available from current position
# @param robot_position: variable with current robot position in maze
# @param fork: dictionary with paths to each fork from current position
# @param fork_number: variable with number of last used fork
# @param fork_count: dictionary with number of unused routes for each fork
#
# @retval fork: updated fork dictionary
# @retval fork_number: updated fork number
# @retval fork_count: updated fork count
# @retval dead_end: bool variable with info if current cell is dead-end
"""


def check_fork_DFS(connections, robot_position, fork, fork_number, fork_count):

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


""" get_back_path_BFS  
# @brief Creates a path from current robot position to its current destination.
# It is used when current destination is not in adjacent cell.
#
# @param graph: dictionary graph with visited cells
# @param start: variable with current robot position in maze
# @param target: variable with position to which robot want's to go
#
# @retval path: list with path to target
"""


def get_path_BFS(graph, start, target):

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
                    if n == target:
                        while n != start:  # or parent[n] != n
                            path.append(n)
                            n = parent[n]
                        search_end = True
                        break
    return path


""" change_orientation
# @brief Change robot orientation basing on last orientation and last turn.
#
# @param robot_orientation: variable with actual robot orientation in global directions
# @param action: variable with information where robot turns
#
# @retval robot_orientation: variable with updated robot orientation
"""


def change_orientation(robot_orientation, action):
    match action:
        case moves.right:  # turn right
            if robot_orientation == direction.WEST:
                robot_orientation = direction.NORTH
            else:
                robot_orientation //= 2
        case moves.left:  # turn left
            if robot_orientation == direction.NORTH:
                robot_orientation = direction.WEST
            else:
                robot_orientation *= 2
        case moves.back:  # turn back
            if (
                robot_orientation == direction.NORTH
                or robot_orientation == direction.EAST
            ):
                robot_orientation //= 4
            else:
                robot_orientation *= 4

    x = direction.index(robot_orientation)

    if mode_params.TESTING:
        print("orientacja:", direction._fields[x])

    return robot_orientation


""" change_position
# @brief Update position of the robot basing on current orientation of the robot.
#
# @param robot_position: variable with robot position
# @param robot_orientation: variable with actual robot orientation in global directions
#
# @retval robot_position: variable with updated robot position
"""


def change_position(robot_position, robot_orientation):

    if robot_orientation == direction.NORTH:
        robot_position = robot_position + maze_parameters.COLUMNS

    elif robot_orientation == direction.EAST:
        robot_position = robot_position + 1
    elif robot_orientation == direction.SOUTH:
        robot_position = robot_position - maze_parameters.COLUMNS

    elif robot_orientation == direction.WEST:
        robot_position = robot_position - 1

    return robot_position


""" change_target
# @brief Marks every visited cell, after reaching targeted cell, change cell to first unvisited cell.
# When reaching final target, saves distance map to file.
#
# @param maze_map: list with actual maze map with walls
# @param robot_position: variable with actual robot position in maze
# @param distance: list with actual distances values/path
# @param target: variable with field number to which robot tries to get
#
# @retval target: variable with updated targeted field
"""


def change_target(maze_map, robot_position, distance, target):

    match mode_params.WHOLE_SEARCH:
        case False:
            distance = init_distance_map(distance, target)  # reset path
            distance = floodfill(maze_map, distance)  # path

            # fill unvisited cells with 4 walls to verify if the shortest path was find
            shortest_path = check_distance(distance, maze_map, target)

            if robot_position == 136:

                maze_map = mark_center(maze_map)

                # distance = init_distance_map(distance, target) #reset path
                # distance = floodfill(maze_map, distance) #path

                # #fill unvisited cells with 4 walls to verify if the shortest path was find
                # shortest_path = check_distance(distance, maze_map, target)

                if shortest_path:
                    print("This is the shortest/ one of the shortest paths")
                    var.searching_end = True
                else:
                    print("There might be a shorter path, keep going")
                    target = 0

            elif robot_position == 0:
                # shortest_path = check_distance(distance, maze_map, target)

                if shortest_path:
                    print("This is the shortest/ one of the shortest paths")
                    var.searching_end = True
                else:
                    print("There might be a shorter path, keep going")

                target = 136
        case True:
            search = True
            i = 0
            while search:  # search to find unvisited cell, otherwise end

                if not (maze_map[i] & maze_parameters.VISITED):

                    target = i
                    search = False
                    if mode_params.TESTING:
                        print("target =", target)
                else:
                    i += 1

                if i == 256:
                    target = 136
                    search = False
                    var.searching_end = True
                    if mode_params.TESTING:
                        print("target =", target)

    return target, maze_map


""" mark_center
# @brief Adds walls to unvisited cells in center
#
# @param maze_map: list with actual maze map with walls
#
# @retval maze_map: list with updated maze map
"""


def mark_center(maze_map):

    center = [119, 120, 135]

    for center_cell in center:
        if (maze_map[center_cell] & maze_parameters.VISITED) != maze_parameters.VISITED:
            match center_cell:
                case 119:
                    maze_map[center_cell] = 3
                    maze_map[center_cell - 1] |= direction.EAST
                    maze_map[center_cell - 16] |= direction.NORTH
                case 120:
                    maze_map[center_cell] = 6
                    maze_map[center_cell + 1] |= direction.WEST
                    maze_map[center_cell - 16] |= direction.NORTH
                case 135:
                    maze_map[center_cell] = 9
                    maze_map[center_cell - 1] |= direction.EAST
                    maze_map[center_cell + 16] |= direction.SOUTH

    return maze_map


""" mark_center_graph 
# @brief Adds walls to unvisited cells in center.
# Path is used to determine which cells weren't visited.
#
# @param maze_map: list with actual maze map with walls
# @param path: list with path
#
# @retval maze_map: list with updated maze map
"""


def mark_center_graph(maze_map, path):

    center = [119, 120, 135]
    rows = maze_parameters.ROWS

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


""" check_distance
# @brief Fills unvisited cells with 4 walls to verify if the shortest path was find.
#
# @param distance: list with actual distances values/path
# @param maze_map: list with actual maze map with walls
# @param target: variable with field number to which robot tries to get
#
# @retval shortest_path: bool variable which informs if shortest path was found
"""


def check_distance(distance, maze_map, target):

    distance_check = distance.copy()
    maze_map_check = maze_map.copy()

    for i in range(0, maze_parameters.MAZE_SIZE):
        if (maze_map_check[i] & maze_parameters.VISITED) != maze_parameters.VISITED:
            maze_map_check[i] = maze_map_check[i] | 15 | maze_parameters.VISITED

    distance_check = init_distance_map(distance_check, target)  # reset path
    distance_check = floodfill(maze_map_check, distance_check)  # path
    if target == 136:
        shortest_path = distance[0] >= distance_check[0]  # could be just equal'
    elif target == 0:
        shortest_path = distance[136] >= distance_check[136]  # could be just equal'

    return shortest_path


""" calc_cost  
# @brief Calculates Manhatan's distance which is used
# as cost in A* algorithm.
#
# @param start: variable with first position
# @param target: variable with second position
#
# @retval distance: Manhatan's distance/cost
"""


def calc_cost(start, target):

    # index to matrix/grid
    point1 = [start % 16, start // 16]
    point2 = [target % 16, target // 16]

    distance = 0
    for x1, x2 in zip(point1, point2):
        difference = x2 - x1
        absolute_difference = abs(difference)
        distance += absolute_difference

    return distance


""" read_file
# @brief Read file. Pickle module used.
#
# @param file_name: variable with a file name
#
# @retval list_temp: list with a content of a file
"""


def read_file(file_name):

    with open(file_name, "rb") as file:
        readed = pickle.load(file)

    if file == None:
        print("ERROR")
        exit(1)

    return readed


""" write_file
# @brief Write file. Pickle module used.
#
# @param file_name: variable with a file name
# @param: values: any type of object with a content to write file
#
# @retval None
"""


def write_file(file_name, values):

    with open(file_name, "wb") as file:
        pickle.dump(values, file)

    if file == None:
        print("ERROR")
        exit(1)


""" choose_file_path
# @brief Choose apropiete directory and file name to save results
#
# @param None
#
# @retval path_file, maze_file: Appropriate files directory and names
"""


def choose_file_path():

    match mode_params.MAZE_LAYOUT:
        case maze.FORBOT:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Forbot/floodfill_path.pkl"
                    maze_file = "Results/Forbot/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Forbot/DFS_path.pkl"
                    maze_file = "Results/Forbot/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Forbot/BFS_path.pkl"
                    maze_file = "Results/Forbot/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Forbot/A_star_path.pkl"
                    maze_file = "Results/Forbot/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Forbot/A_star_mod_path.pkl"
                    maze_file = "Results/Forbot/A_star_mod_maze.pkl"
        case maze.TAIWAN_2015:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Taiwan2015/floodfill_path.pkl"
                    maze_file = "Results/Taiwan2015/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Taiwan2015/DFS_path.pkl"
                    maze_file = "Results/Taiwan2015/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Taiwan2015/BFS_path.pkl"
                    maze_file = "Results/Taiwan2015/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Taiwan2015/A_star_path.pkl"
                    maze_file = "Results/Taiwan2015/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Taiwan2015/A_star_mod_path.pkl"
                    maze_file = "Results/Taiwan2015/A_star_mod_maze.pkl"
        case maze.APEC_2010:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Apec2010/floodfill_path.pkl"
                    maze_file = "Results/Apec2010/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Apec2010/DFS_path.pkl"
                    maze_file = "Results/Apec2010/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Apec2010/BFS_path.pkl"
                    maze_file = "Results/Apec2010/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Apec2010/A_star_path.pkl"
                    maze_file = "Results/Apec2010/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Apec2010/A_star_mod_path.pkl"
                    maze_file = "Results/Apec2010/A_star_mod_maze.pkl"
        case maze.UK_2016:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/UK2016/floodfill_path.pkl"
                    maze_file = "Results/UK2016/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/UK2016/DFS_path.pkl"
                    maze_file = "Results/UK2016/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/UK2016/BFS_path.pkl"
                    maze_file = "Results/UK2016/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/UK2016/A_star_path.pkl"
                    maze_file = "Results/UK2016/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/UK2016/A_star_mod_path.pkl"
                    maze_file = "Results/UK2016/A_star_mod_maze.pkl"
        case maze.HIGASHI_2017:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Higashi2017_mod/floodfill_path.pkl"
                    maze_file = "Results/Higashi2017_mod/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Higashi2017_mod/DFS_path.pkl"
                    maze_file = "Results/Higashi2017_mod/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Higashi2017_mod/BFS_path.pkl"
                    maze_file = "Results/Higashi2017_mod/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Higashi2017_mod/A_star_path.pkl"
                    maze_file = "Results/Higashi2017_mod/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Higashi2017_mod/A_star_mod_path.pkl"
                    maze_file = "Results/Higashi2017_mod/A_star_mod_maze.pkl"
        case maze.JAPAN_2013EQ:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Japan2013eq/floodfill_path.pkl"
                    maze_file = "Results/Japan2013eq/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Japan2013eq/DFS_path.pkl"
                    maze_file = "Results/Japan2013eq/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Japan2013eq/BFS_path.pkl"
                    maze_file = "Results/Japan2013eq/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Japan2013eq/A_star_path.pkl"
                    maze_file = "Results/Japan2013eq/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Japan2013eq/A_star_mod_path.pkl"
                    maze_file = "Results/Japan2013eq/A_star_mod_maze.pkl"
        case maze.KANKOU_2003:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Kankou2003/floodfill_path.pkl"
                    maze_file = "Results/Kankou2003/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Kankou2003/DFS_path.pkl"
                    maze_file = "Results/Kankou2003/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Kankou2003/BFS_path.pkl"
                    maze_file = "Results/Kankou2003/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Kankou2003/A_star_path.pkl"
                    maze_file = "Results/Kankou2003/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Kankou2003/A_star_mod_path.pkl"
                    maze_file = "Results/Kankou2003/A_star_mod_maze.pkl"
        case maze.JAPAN_2011:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Japan2011/floodfill_path.pkl"
                    maze_file = "Results/Japan2011/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Japan2011/DFS_path.pkl"
                    maze_file = "Results/Japan2011/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Japan2011/BFS_path.pkl"
                    maze_file = "Results/Japan2011/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Japan2011/A_star_path.pkl"
                    maze_file = "Results/Japan2011/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Japan2011/A_star_mod_path.pkl"
                    maze_file = "Results/Japan2011/A_star_mod_maze.pkl"
        case maze.JAPAN_1987:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Japan1987/floodfill_path.pkl"
                    maze_file = "Results/Japan1987/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Japan1987/DFS_path.pkl"
                    maze_file = "Results/Japan1987/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Japan1987/BFS_path.pkl"
                    maze_file = "Results/Japan1987/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Japan1987/A_star_path.pkl"
                    maze_file = "Results/Japan1987/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Japan1987/A_star_mod_path.pkl"
                    maze_file = "Results/Japan1987/A_star_mod_maze.pkl"
        case maze.KOR_88:
            match mode_params.ALGORITHM:
                case algorithms.FLOODFILL:
                    path_file = "Results/Kor88/floodfill_path.pkl"
                    maze_file = "Results/Kor88/floodfill_maze.pkl"
                case algorithms.DFS:
                    path_file = "Results/Kor88/DFS_path.pkl"
                    maze_file = "Results/Kor88/DFS_maze.pkl"
                case algorithms.BFS:
                    path_file = "Results/Kor88/BFS_path.pkl"
                    maze_file = "Results/Kor88/BFS_maze.pkl"
                case algorithms.A_STAR:
                    path_file = "Results/Kor88/A_star_path.pkl"
                    maze_file = "Results/Kor88/A_star_maze.pkl"
                case algorithms.A_STAR_MOD:
                    path_file = "Results/Kor88/A_star_mod_path.pkl"
                    maze_file = "Results/Kor88/A_star_mod_maze.pkl"

    return path_file, maze_file
