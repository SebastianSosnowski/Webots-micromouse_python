from controller import Keyboard, Robot
from collections import namedtuple, deque
from threading import Thread

# my modules
from Constants import *
import map_functions as map_f
import move_functions as move_f
import algorithm_functions as algorithm_f
import draw_maze
import var

from utils.my_robot import MyRobot
from config.enums import Move


""" floodfill_main
# @brief Main program for floodfill algorithm controller. 
# 2 approaches are available:
# 1. Without searching whole maze - every cycle robot calculates shortest path to target
# and tries to go to it. When target is found it checks if it was the shortest path
# by comparing paths for 2 mazes: actually discovered and discovered but cells
# which weren't visited are assumed with 4 walls. If path from actually discovered
# maze has same length as 2nd one - the shortest path was founded. If not robot makes
# second run - from target to start cell to search some of unvisited part of maze.
# Process repeat's until shortest path is found.
# 2. With searching whole maze - WORKS only for mazes, where ALL cells are accessible.
# Unrecommended to use.
#
# @param robot: object with robot instance
#
# @retval None
"""


def floodfill_main(robot: MyRobot):

    # left_motor, right_motor, ps_left, ps_right, ps, tof = init_devices(robot)
    target, robot_position, start, robot_orientation = init_parameters()

    maze_map = [0] * maze_parameters.MAZE_SIZE
    distance = [255] * maze_parameters.MAZE_SIZE

    maze_map = map_f.init_maze_map(maze_map)

    var.maze_map_global = maze_map

    path_file, maze_file = algorithm_f.choose_file_path()

    while robot.step(TIME_STEP) != -1:

        if mode_params.TESTING:
            print("sensor tof %.2f" % tof.getValue())

        if mode_params.TESTING:
            print("sensor ps6 left %.2f" % ps[6].getValue())

        if mode_params.TESTING:
            print("sensor ps1 right %.2f" % ps[1].getValue())

        match mode_params.MODE:
            case mode_params.SEARCH:  # search

                if start:
                    # run in another thread to make it possible to look on it during robot run
                    Maze_thread = Thread(
                        target=draw_maze.draw_maze,
                        args=(var.maze_map_global, var.distance_global),
                        daemon=True,
                    )
                    Maze_thread.start()

                    start = False
                if mode_params.TESTING:
                    timer = robot.getTime()

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(
                    robot, ps, tof, 5
                )

                if left_wall:
                    maze_map = map_f.add_wall(
                        maze_map, robot_position, robot_orientation, direction.WEST
                    )

                if front_wall:
                    maze_map = map_f.add_wall(
                        maze_map, robot_position, robot_orientation, direction.NORTH
                    )

                if right_wall:
                    maze_map = map_f.add_wall(
                        maze_map, robot_position, robot_orientation, direction.EAST
                    )

                if back_wall:
                    maze_map = map_f.add_wall(
                        maze_map, robot_position, robot_orientation, direction.SOUTH
                    )

                distance = map_f.init_distance_map(distance, target)  # reset path

                distance = algorithm_f.floodfill(maze_map, distance)  # path

                var.robot_pos = robot_position

                var.maze_map_global = maze_map

                if var.distance_global != distance:
                    var.distance_global = distance
                    var.distance_update = True

                var.target_global = target
                var.drawing_event.set()

                if var.searching_end:
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    algorithm_f.write_file(maze_file, maze_map)
                    for i in range(
                        0, maze_parameters.MAZE_SIZE
                    ):  # to make sure path will use only visited cells
                        if (
                            maze_map[i] & maze_parameters.VISITED
                        ) != maze_parameters.VISITED:
                            maze_map[i] |= 15
                    target = 136
                    distance = map_f.init_distance_map(distance, target)  # reset path
                    distance = algorithm_f.floodfill(maze_map, distance)  # path
                    algorithm_f.write_file(path_file, distance)
                    input("press any key to end")
                    exit(0)

                robot_position, robot_orientation = move_f.move_one_position(
                    maze_map[robot_position],
                    distance,
                    robot_position,
                    robot_orientation,
                    robot,
                    ps,
                    tof,
                    left_motor,
                    right_motor,
                    ps_left,
                    ps_right,
                )

                if mode_params.TESTING:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

                maze_map[robot_position] |= maze_parameters.VISITED  # mark visited tile

                if robot_position == target:
                    target, maze_map = algorithm_f.change_target(
                        maze_map, robot_position, distance, target
                    )

                var.main_event.wait()
                var.main_event.clear()

            case mode_params.SPEEDRUN:  # speedrun

                if start:
                    distance = algorithm_f.read_file(path_file)
                    maze_map = algorithm_f.read_file(maze_file)

                    # run in another thread to make it possible to look on it during robot run
                    Maze_thread = Thread(
                        target=draw_maze.draw_maze,
                        args=(maze_map, distance),
                        daemon=True,
                    )
                    Maze_thread.start()

                    start = 0

                robot_position, robot_orientation = move_f.move_one_position(
                    maze_map[robot_position],
                    distance,
                    robot_position,
                    robot_orientation,
                    robot,
                    ps,
                    tof,
                    left_motor,
                    right_motor,
                    ps_left,
                    ps_right,
                )

                var.robot_pos = robot_position

                var.drawing_event.set()

                var.main_event.wait()
                var.main_event.clear()

                if robot_position == target:
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


""" DFS_main
# @brief Main program for Depth first search algorithm controller.
# Doesn't guarantee the shortest path but usually finds path very fast (micromouse mazes).
#
# @param robot: object with robot instance
#
# @retval None
"""


def DFS_main(robot):

    left_motor, right_motor, ps_left, ps_right, ps, tof = init_devices(robot)
    target, robot_position, start, robot_orientation = init_parameters()

    maze_map = map_f.init_maze_map_graph()

    var.maze_map_global = maze_map

    path_file, maze_file = algorithm_f.choose_file_path()

    # dfs vars
    fork_number = -1
    unused_routes = {}
    fork = {}
    visited = []
    stack = []
    path = []
    visited.append(robot_position)
    stack.append(robot_position)

    match mode_params.MODE:
        case mode_params.SEARCH:  # search

            if start:
                # run in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze,
                    args=(var.maze_map_global, []),
                    daemon=True,
                )
                Maze_thread.start()

                start = False

            if mode_params.TESTING:
                timer = robot.getTime()

            while stack:

                if robot.step(TIME_STEP) == -1:
                    break

                stack.pop()

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(
                    robot, ps, tof, 5
                )
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot_position, robot_orientation, walls
                )

                path.append(robot_position)

                var.robot_pos = robot_position
                var.maze_map_global = maze_map

                var.drawing_event.set()

                if robot_position == target:

                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    var.main_event.wait()
                    var.main_event.clear()
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_file, path)
                    algorithm_f.write_file(maze_file, maze_map)
                    input("press any key to end")
                    exit(0)

                fork, fork_number, unused_routes, dead_end = algorithm_f.check_fork_DFS(
                    maze_map[robot_position],
                    robot_position,
                    fork,
                    fork_number,
                    unused_routes,
                )

                if dead_end:
                    (
                        fork,
                        fork_number,
                        unused_routes,
                        path,
                        robot_orientation,
                        robot_position,
                    ) = move_f.move_back_DFS(
                        stack[-1],
                        maze_map,
                        robot_position,
                        fork,
                        fork_number,
                        unused_routes,
                        robot_orientation,
                        robot,
                        ps,
                        tof,
                        left_motor,
                        right_motor,
                        ps_left,
                        ps_right,
                        path,
                    )
                    fork[fork_number].append(path[-1])

                current_destination, visited, stack = (
                    algorithm_f.check_possible_routes_DFS(
                        maze_map[robot_position], visited, stack, target
                    )
                )

                if current_destination not in maze_map[robot_position]:
                    fork[fork_number].pop()
                    (
                        fork,
                        fork_number,
                        unused_routes,
                        path,
                        robot_orientation,
                        robot_position,
                    ) = move_f.move_back_DFS(
                        current_destination,
                        maze_map,
                        robot_position,
                        fork,
                        fork_number,
                        unused_routes,
                        robot_orientation,
                        robot,
                        ps,
                        tof,
                        left_motor,
                        right_motor,
                        ps_left,
                        ps_right,
                        path,
                    )
                    fork[fork_number].append(robot_position)

                robot_position, robot_orientation = move_f.move_one_position_graph(
                    current_destination,
                    robot_position,
                    robot_orientation,
                    robot,
                    ps,
                    tof,
                    left_motor,
                    right_motor,
                    ps_left,
                    ps_right,
                )

                if mode_params.TESTING:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

                var.main_event.wait()
                var.main_event.clear()

        case mode_params.SPEEDRUN:

            if start:
                path = algorithm_f.read_file(path_file)
                path.reverse()
                print(len(path))
                path.pop()  # remove start cell

                maze_map = algorithm_f.read_file(maze_file)
                var.maze_map_global = maze_map

                # run in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze, args=(maze_map, []), daemon=True
                )
                Maze_thread.start()

                start = 0

            while path:

                if robot.step(TIME_STEP) == -1:
                    break

                current_destination = path.pop()

                robot_position, robot_orientation = move_f.move_one_position_graph(
                    current_destination,
                    robot_position,
                    robot_orientation,
                    robot,
                    ps,
                    tof,
                    left_motor,
                    right_motor,
                    ps_left,
                    ps_right,
                )

                var.robot_pos = robot_position

                var.drawing_event.set()

                var.main_event.wait()
                var.main_event.clear()

                if robot_position == target:
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


""" BFS_main
# @brief Main program for Breadth first search algorithm controller.
# It was adjusted for robot movement. BFS is a horizontal searching through graph
# by going by each 'level' of nodes. To avoid unnecessary back-tracking,
# only forks are treated as 'levels',  which means that robot will go back
# only when it moves to new fork or dead-end. Because of that it doesn't guarantees shortest path.
#
# @param robot: object with robot instance
#
# @retval None
"""


def BFS_main(robot):

    left_motor, right_motor, ps_left, ps_right, ps, tof = init_devices(robot)
    target, robot_position, start, robot_orientation = init_parameters()

    maze_map = map_f.init_maze_map_graph()

    var.maze_map_global = maze_map

    maze_map_searched = {}
    for i in range(256):
        maze_map_searched[i] = []

    path_file, maze_file = algorithm_f.choose_file_path()

    # bfs vars
    visited = []
    path = []
    searching_end = False
    move_back = False
    fork = False
    queue = deque()

    visited.append(robot_position)
    queue.append(robot_position)

    match mode_params.MODE:
        case mode_params.SEARCH:  # search

            if start:
                # run in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze,
                    args=(var.maze_map_global, []),
                    daemon=True,
                )
                Maze_thread.start()

                start = False
            if mode_params.TESTING:
                timer = robot.getTime()

            while queue:

                if robot.step(TIME_STEP) == -1:
                    break

                if searching_end:
                    robot_position = queue.pop()
                elif move_back or fork:
                    robot_position = queue.popleft()
                    move_back = False
                else:
                    robot_position = queue.pop()

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(
                    robot, ps, tof, 5
                )
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot_position, robot_orientation, walls
                )
                maze_map_searched = map_f.add_walls_graph(
                    maze_map_searched, robot_position, robot_orientation, walls
                )

                var.robot_pos = robot_position

                var.maze_map_global = maze_map
                var.drawing_event.set()

                if robot_position == target:
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    path = algorithm_f.get_path_BFS(
                        maze_map_searched, maze_parameters.START_CELL, target
                    )
                    var.main_event.wait()
                    var.main_event.clear()
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_file, path)
                    algorithm_f.write_file(maze_file, maze_map)
                    input("press any key to end")
                    exit(0)

                routes = len(maze_map[robot_position])
                fork = routes >= 3

                current_destination, visited, queue, move_back, searching_end = (
                    algorithm_f.check_possible_routes_BFS(
                        maze_map[robot_position], visited, queue, fork, target
                    )
                )

                if (
                    current_destination not in maze_map[robot_position]
                ):  # not adjacent cell e.g. we move back farther than 1 cell
                    # temp_graph = {}
                    # for cell in visited: # create sub-graph only from visited cells
                    #     temp_graph[cell] = maze_map[cell]

                    back_path = algorithm_f.get_path_BFS(
                        maze_map_searched, robot_position, current_destination
                    )

                    while back_path:
                        Move_to = back_path.pop()
                        robot_position, robot_orientation = (
                            move_f.move_one_position_graph(
                                Move_to,
                                robot_position,
                                robot_orientation,
                                robot,
                                ps,
                                tof,
                                left_motor,
                                right_motor,
                                ps_left,
                                ps_right,
                            )
                        )
                else:
                    robot_position, robot_orientation = move_f.move_one_position_graph(
                        current_destination,
                        robot_position,
                        robot_orientation,
                        robot,
                        ps,
                        tof,
                        left_motor,
                        right_motor,
                        ps_left,
                        ps_right,
                    )

                if mode_params.TESTING:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

                var.main_event.wait()
                var.main_event.clear()

        case mode_params.SPEEDRUN:

            if start:
                path = algorithm_f.read_file(path_file)
                print(len(path))
                maze_map = algorithm_f.read_file(maze_file)
                var.maze_map_global = maze_map

                # run in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze, args=(maze_map, []), daemon=True
                )
                Maze_thread.start()

                start = 0

            while path:

                if robot.step(TIME_STEP) == -1:
                    break

                current_destination = path.pop()

                robot_position, robot_orientation = move_f.move_one_position_graph(
                    current_destination,
                    robot_position,
                    robot_orientation,
                    robot,
                    ps,
                    tof,
                    left_motor,
                    right_motor,
                    ps_left,
                    ps_right,
                )

                var.robot_pos = robot_position

                var.drawing_event.set()

                var.main_event.wait()
                var.main_event.clear()

                if robot_position == target:
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


""" A_star_main
# @brief Main program for A* algorithm controller. 
# Guarantees shortest path, but very long search time.
#
# @param robot: object with robot instance
#
# @retval None
"""


def A_star_main(robot):

    left_motor, right_motor, ps_left, ps_right, ps, tof = init_devices(robot)
    target, robot_position, start, robot_orientation = init_parameters()

    maze_map = map_f.init_maze_map_graph()

    var.maze_map_global = maze_map

    maze_map_searched = {}
    for i in range(256):
        maze_map_searched[i] = []

    path_file, maze_file = algorithm_f.choose_file_path()

    # A* vars
    open = []  # list of unvisited nodes
    closed = []  # list of visited nodes
    cost = {}
    parent = {}  # probably not needed anymore
    path = []
    time_sum = 0.0
    current_destination = robot_position
    cost[robot_position] = [0, algorithm_f.calc_cost(robot_position, target)]
    parent[robot_position] = robot_position
    open.append(robot_position)

    match mode_params.MODE:
        case mode_params.SEARCH:  # search

            if start:
                # run in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze,
                    args=(var.maze_map_global, []),
                    daemon=True,
                )
                Maze_thread.start()

                start = False
            if mode_params.TESTING:
                timer = robot.getTime()

            while open:

                if robot.step(TIME_STEP) == -1:
                    break

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(
                    robot, ps, tof, 5
                )
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot_position, robot_orientation, walls
                )
                maze_map_searched = map_f.add_walls_graph(
                    maze_map_searched, robot_position, robot_orientation, walls
                )

                open.remove(robot_position)
                closed.append(robot_position)

                open, parent, cost = algorithm_f.update_neighbors_costs(
                    maze_map[robot_position], open, closed, parent, cost, robot_position
                )

                var.robot_pos = robot_position
                var.maze_map_global = maze_map
                var.cost_global = cost

                var.drawing_event.set()

                if robot_position == target:
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, maze_parameters.START_CELL, target
                    )
                    var.main_event.wait()
                    var.main_event.clear()
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_file, path)
                    algorithm_f.write_file(maze_file, maze_map)
                    input("press any key to end")
                    exit(0)

                current_destination = algorithm_f.check_possible_routes_A_star(
                    open, cost
                )

                if (
                    current_destination not in maze_map[robot_position]
                ):  # not neighbor cell e.g. we move back farther than 1 cell

                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, robot_position, current_destination
                    )
                    while path:
                        Move_to = path.pop(0)
                        robot_position, robot_orientation = (
                            move_f.move_one_position_graph(
                                Move_to,
                                robot_position,
                                robot_orientation,
                                robot,
                                ps,
                                tof,
                                left_motor,
                                right_motor,
                                ps_left,
                                ps_right,
                            )
                        )

                else:
                    robot_position, robot_orientation = move_f.move_one_position_graph(
                        current_destination,
                        robot_position,
                        robot_orientation,
                        robot,
                        ps,
                        tof,
                        left_motor,
                        right_motor,
                        ps_left,
                        ps_right,
                    )

                if mode_params.TESTING:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

                var.main_event.wait()
                var.main_event.clear()

        case mode_params.SPEEDRUN:  # speedrun

            if start:
                path = algorithm_f.read_file(path_file)
                print(len(path))
                maze_map = algorithm_f.read_file(maze_file)
                var.maze_map_global = maze_map

                # run in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze, args=(maze_map, []), daemon=True
                )
                Maze_thread.start()

                start = 0

            while path:

                if robot.step(TIME_STEP) == -1:
                    break

                current_destination = path.pop(0)

                robot_position, robot_orientation = move_f.move_one_position_graph(
                    current_destination,
                    robot_position,
                    robot_orientation,
                    robot,
                    ps,
                    tof,
                    left_motor,
                    right_motor,
                    ps_left,
                    ps_right,
                )

                var.robot_pos = robot_position

                var.drawing_event.set()

                var.main_event.wait()
                var.main_event.clear()

                if robot_position == target:
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


""" A_star_main_modified
# @brief Main program for A* modified algorithm controller.
# Modification is that robot chooses where to go in 2 ways:
# 1. If current position is fork or dead-end - choose cell with lowest
# Fcost and/or Hcost (just like in normal A*).
# 2. If current cell is corridor - keep going until it's fork or dead-end.
# This approach makes searching much faster than normal A*, because robot
# doesn't need to keep moving across whole maze to just check one cell.
# The only drawback is that this approach might not guarantee the shortest path,
# although in micromouse mazes it usually should find it.
#
# @param robot: object with robot instance
#
# @retval None
"""


def A_star_main_modified(robot):

    left_motor, right_motor, ps_left, ps_right, ps, tof = init_devices(robot)
    target, robot_position, start, robot_orientation = init_parameters()

    maze_map = map_f.init_maze_map_graph()
    var.maze_map_global = maze_map

    maze_map_searched = {}
    for i in range(256):
        maze_map_searched[i] = []

    path_file, maze_file = algorithm_f.choose_file_path()

    # A* vars
    open = []  # list of unvisited nodes
    closed = []  # list of visited nodes
    cost = {}
    parent = {}  # probably not needed anymore
    path = []

    time_sum = 0.0

    current_destination = robot_position
    cost[robot_position] = [0, algorithm_f.calc_cost(robot_position, target)]
    parent[robot_position] = robot_position
    open.append(robot_position)

    match mode_params.MODE:
        case mode_params.SEARCH:  # search

            if start:
                # run maze drawing in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze,
                    args=(var.maze_map_global, []),
                    daemon=True,
                )
                Maze_thread.start()

                start = False
            if mode_params.TESTING:
                timer = robot.getTime()

            while open:

                if robot.step(TIME_STEP) == -1:
                    break

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(
                    robot, ps, tof, 5
                )
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot_position, robot_orientation, walls
                )
                maze_map_searched = map_f.add_walls_graph(
                    maze_map_searched, robot_position, robot_orientation, walls
                )

                open.remove(robot_position)
                closed.append(robot_position)

                open, parent, cost = algorithm_f.update_neighbors_costs(
                    maze_map[robot_position], open, closed, parent, cost, robot_position
                )

                var.robot_pos = robot_position
                var.maze_map_global = maze_map
                var.cost_global = cost

                var.drawing_event.set()

                if robot_position == target:
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, maze_parameters.START_CELL, target
                    )
                    var.main_event.wait()
                    var.main_event.clear()
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_file, path)
                    algorithm_f.write_file(maze_file, maze_map)
                    input("press any key to end")
                    exit(0)

                routes = len(maze_map[robot_position])
                corridor = routes == 2

                if corridor:
                    current_destination = open[-1]
                else:
                    current_destination = algorithm_f.check_possible_routes_A_star(
                        open, cost
                    )

                if (
                    current_destination not in maze_map[robot_position]
                ):  # not adjacent cell e.g. we move back farther than 1 cell

                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, robot_position, current_destination
                    )

                    while path:
                        Move_to = path.pop(0)
                        robot_position, robot_orientation = (
                            move_f.move_one_position_graph(
                                Move_to,
                                robot_position,
                                robot_orientation,
                                robot,
                                ps,
                                tof,
                                left_motor,
                                right_motor,
                                ps_left,
                                ps_right,
                            )
                        )

                else:
                    robot_position, robot_orientation = move_f.move_one_position_graph(
                        current_destination,
                        robot_position,
                        robot_orientation,
                        robot,
                        ps,
                        tof,
                        left_motor,
                        right_motor,
                        ps_left,
                        ps_right,
                    )

                if mode_params.TESTING:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

                var.main_event.wait()
                var.main_event.clear()

        case mode_params.SPEEDRUN:  # speedrun

            if start:
                path = algorithm_f.read_file(path_file)
                print(len(path))
                maze_map = algorithm_f.read_file(maze_file)
                var.maze_map_global = maze_map

                # run in another thread to make it possible to look on it during robot run
                Maze_thread = Thread(
                    target=draw_maze.draw_maze, args=(maze_map, []), daemon=True
                )
                Maze_thread.start()

                start = 0

            while path:

                if robot.step(TIME_STEP) == -1:
                    break

                current_destination = path.pop(0)

                robot_position, robot_orientation = move_f.move_one_position_graph(
                    current_destination,
                    robot_position,
                    robot_orientation,
                    robot,
                    ps,
                    tof,
                    left_motor,
                    right_motor,
                    ps_left,
                    ps_right,
                )

                var.robot_pos = robot_position

                var.drawing_event.set()

                var.main_event.wait()
                var.main_event.clear()

                if robot_position == target:
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


""" keyboard_main
# @brief Main program for manual/with arrows controller.
# Made for testing purposes to move robot with WASD.
#
# @param robot: object with robot instance
#
# @retval None
"""


def keyboard_main(robot: MyRobot):

    # left_motor, right_motor, ps_left, ps_right, ps, tof = init_devices(robot)

    keyboard = Keyboard()
    keyboard.enable(robot.time_step)
    max_tof = 0
    while robot.step(robot.time_step) != -1:

        if mode_params.TESTING:
            avg_front_sensor = 0
            for i in range(0, 3):  # more scans for better accuracy

                avg_front_sensor += robot.tof.getValue()

                robot.step(robot.time_step)  # simulation update

            avg_front_sensor = avg_front_sensor / 3

            print("sensor tof %.2f" % avg_front_sensor)
            if avg_front_sensor > max_tof:
                max_tof = avg_front_sensor
            print("max tof %.2f" % max_tof)

        if mode_params.TESTING:
            print("sensor ps6 left %.2f" % robot.ps[6].getValue())

        if mode_params.TESTING:
            print("sensor ps1 right %.2f" % robot.ps[1].getValue())

        key = keyboard.get_key()
        match key:
            case Move.FORWARD:
                print(key)
                move_f.move_1_tile(robot)
            case Move.RIGHT | Move.LEFT | Move.BACK:
                print(key)
                move_f.turn(robot, key)


""" init_devices
# @brief Init robot peripherals.
#
# @param robot: object with robot instance
#
# @retval left_motor: object with left motor instance
# @retval right_motor: object with right motor instance
# @retval ps_left: object with left position sensor instance
# @retval ps_right: object with right position sensor instance
# @retval ps: list of distance IR sensors objects
# @retval tof: object with distance Tof sensor
"""


def init_devices(robot: MyRobot):
    left_motor = robot.getDevice("left wheel motor")
    right_motor = robot.getDevice("right wheel motor")
    left_motor.setVelocity(robot_parameters.SPEED)
    right_motor.setVelocity(robot_parameters.SPEED)

    ps_left = robot.getDevice("left wheel sensor")
    ps_left.enable(TIME_STEP)
    ps_right = robot.getDevice("right wheel sensor")
    ps_right.enable(TIME_STEP)

    ps = [""] * 8
    ps_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
    for i in range(len(ps_names)):
        ps[i] = robot.getDevice(ps_names[i])
        ps[i].enable(TIME_STEP)

    tof = robot.getDevice("tof")
    tof.enable(TIME_STEP)

    return left_motor, right_motor, ps_left, ps_right, ps, tof


""" init_parameters
# @brief Init robot peripherals.
#
# @param None
#
# @retval target: variable with target cell
# @retval robot_position: variable with starting robot position
# @retval start: bool variable with start indication
# @retval robot_orientation: variable with starting robot orientation
"""


def init_parameters():

    target = maze_parameters.TARGET_CELL  # robot start target
    robot_position = maze_parameters.START_CELL  # robot start position
    start = True  # to open file 1 time
    robot_orientation = direction.NORTH  # robot start orientation

    return target, robot_position, start, robot_orientation
