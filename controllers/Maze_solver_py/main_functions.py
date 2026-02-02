from collections import deque
from queue import Queue

from controller import Keyboard

import algorithm_functions as algorithm_f
import map_functions as map_f
import move_functions as move_f

from config.enums import Direction, Mode, Move
from config.world import world
from draw.maze_drawer import MazeDrawer

from utils.my_robot import MyRobot

# from robot.robot_base import MyRobot
from maze_solver import MazeSolver

from utils.params import DrawState
from read_files.storage import read_results, save_results


def interface_main(mz: MazeSolver):
    mz.algorithm.init()

    path, maze_map, position_values = mz.init_drawer_values()

    draw_queue = Queue(maxsize=1)
    drawer = MazeDrawer(mz._cfg, maze_map, position_values, draw_queue)
    drawer.start_drawing()
    match mz._cfg.simulation.mode:
        case Mode.SEARCH:
            while mz.robot.robot.step(mz._cfg.simulation.time_step) != -1:
                detected = mz.robot.read_sensors()
                targets = mz.algorithm.update(detected, mz.robot.state)
                while targets:
                    draw_queue.put(
                        DrawState(
                            mz.robot.state.pos, mz.algorithm.maze_map, mz.algorithm.position_values
                        )
                    )
                    mz.robot.move(targets.pop(0))
                if mz.algorithm.finish():
                    draw_queue.put(
                        DrawState(
                            mz.robot.state.pos, mz.algorithm.maze_map, mz.algorithm.position_values
                        )
                    )
                    draw_queue.put(None)
                    print("Target reached")
                    print("Searching time: %.2f" % mz.robot.robot.getTime(), "s")
                    path, maze_map, position_values = mz.algorithm.prepare_results()
                    save_results(
                        path,
                        maze_map,
                        position_values,
                        mz._cfg.simulation.maze_layout,
                        mz._cfg.simulation.algorithm,
                    )
                    break
        case Mode.SPEEDRUN:
            while mz.robot.robot.step(mz._cfg.simulation.time_step) != -1:
                if not path:
                    break
                draw_queue.put(
                    DrawState(mz.robot.state.pos, mz.algorithm.maze_map, position_values)
                )
                mz.robot.move(path.pop(0))
            draw_queue.put(DrawState(mz.robot.state.pos, mz.algorithm.maze_map, position_values))
            draw_queue.put(None)
            print("Target reached")
            print("Speedrun time: %.2f" % mz.robot.robot.getTime(), "s")

    input("press any key to end")
    exit(0)


def floodfill_main(robot: MyRobot):
    """Main program for floodfill algorithm controller.
    Every cycle robot calculates shortest path to target and tries to go to it.
    When target is found, it checks if it was the shortest path by comparing paths
    for 2 mazes: actually discovered and discovered, but cells which weren't visited
    are assumed with 4 walls. If path from actually discovered maze has same length
    as 2nd one - the shortest path was founded. If not, robot makes next run - from
    target to start cell to search some of unvisited part of maze. Process is repeated
    until shortest path is found.

    Args:
        robot: Object with robot instance.

    Returns:
        None
    """
    maze_map = [0] * world.maze.size
    distance = [255] * world.maze.size

    maze_map = map_f.init_maze_map(maze_map)

    shortest_path = False

    path_dir, maze_dir = algorithm_f.create_files_directories(
        world.sim.maze_layout, world.sim.algorithm
    )
    draw_queue = Queue(maxsize=1)

    while robot.step(world.sim.time_step) != -1:

        if world.sim.testing:
            print("sensor tof %.2f" % robot.tof.getValue())

        if world.sim.testing:
            print("sensor ps6 left %.2f" % robot.ps[6].getValue())

        if world.sim.testing:
            print("sensor ps1 right %.2f" % robot.ps[1].getValue())

        match world.sim.mode:
            case Mode.SEARCH:

                if robot.state.start:
                    # run in another thread to make it possible to look on it during robot run
                    drawer = MazeDrawer(maze_map, distance, draw_queue)
                    drawer.start_drawing()
                    robot.state.start = False
                if world.sim.testing:
                    timer = robot.getTime()

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(robot, 5)

                if left_wall:
                    maze_map = map_f.add_wall(robot, maze_map, Direction.WEST)

                if front_wall:
                    maze_map = map_f.add_wall(robot, maze_map, Direction.NORTH)

                if right_wall:
                    maze_map = map_f.add_wall(robot, maze_map, Direction.EAST)

                if back_wall:
                    maze_map = map_f.add_wall(robot, maze_map, Direction.SOUTH)

                distance = map_f.init_distance_map(
                    distance, robot.state.current_target
                )  # reset path

                distance = algorithm_f.floodfill(maze_map, distance)  # path

                draw_queue.put(DrawState(robot.state.pos, maze_map, distance, {}))

                if shortest_path:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    algorithm_f.write_file(maze_dir, maze_map)

                    # to make sure path will use only visited cells
                    for i in range(0, world.maze.size):
                        if (maze_map[i] & world.maze.visited) != world.maze.visited:
                            maze_map[i] |= 15
                    robot.state.current_target = world.maze.target_cell
                    distance = map_f.init_distance_map(
                        distance, robot.state.current_target
                    )  # reset path
                    distance = algorithm_f.floodfill(maze_map, distance)  # path
                    algorithm_f.write_file(path_dir, distance)
                    input("press any key to end")
                    exit(0)

                move_f.move_one_position(robot, maze_map[robot.state.pos], distance)

                if world.sim.testing:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

                maze_map[robot.state.pos] |= world.maze.visited  # mark visited tile

                if robot.state.pos == robot.state.current_target:
                    maze_map, shortest_path = algorithm_f.change_target(robot, maze_map, distance)

            case Mode.SPEEDRUN:

                if robot.state.start:
                    distance = algorithm_f.read_file(path_dir)
                    maze_map = algorithm_f.read_file(maze_dir)
                    drawer = MazeDrawer(maze_map, distance, draw_queue)
                    drawer.start_drawing()

                    robot.state.start = False
                move_f.move_one_position(robot, maze_map[robot.state.pos], distance)
                draw_queue.put(DrawState(robot.state.pos, maze_map, distance, {}))

                if robot.state.pos == robot.state.current_target:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


def DFS_main(robot: MyRobot):
    """Main program for Depth first search algorithm controller.
    Doesn't guarantee the shortest path but usually finds path very fast (micromouse mazes).

    Args:
        robot: Object with robot instance.

    Returns:
        None
    """
    maze_map = map_f.init_maze_map_graph()

    # path_dir, maze_dir = algorithm_f.create_files_directories(
    #     world.sim.maze_layout, world.sim.algorithm
    # )

    # dfs vars
    fork_number = -1
    unused_routes = {}
    fork = {}
    visited = []
    stack = []
    path = []
    visited.append(robot.state.pos)
    stack.append(robot.state.pos)

    draw_queue = Queue(maxsize=1)

    match world.sim.mode:
        case Mode.SEARCH:

            if robot.state.start:
                # run in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False
            if world.sim.testing:
                timer = robot.getTime()

            while stack:

                if robot.step(world.sim.time_step) == -1:
                    break

                stack.pop()

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(robot, 5)
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot.state.pos, robot.state.orientation, walls
                )

                path.append(robot.state.pos)

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], {}))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_dir, path)
                    algorithm_f.write_file(maze_dir, maze_map)
                    input("press any key to end")
                    exit(0)

                fork, fork_number, unused_routes, dead_end = algorithm_f.check_fork_DFS(
                    maze_map[robot.state.pos],
                    robot.state.pos,
                    fork,
                    fork_number,
                    unused_routes,
                )

                if dead_end:
                    (fork, fork_number, unused_routes, path) = move_f.move_back_DFS(
                        robot, stack[-1], maze_map, fork, fork_number, unused_routes, path
                    )
                    fork[fork_number].append(path[-1])

                robot.state.current_target, visited, stack = algorithm_f.check_possible_routes_DFS(
                    maze_map[robot.state.pos], visited, stack
                )

                if robot.state.current_target not in maze_map[robot.state.pos]:
                    fork[fork_number].pop()
                    (fork, fork_number, unused_routes, path) = move_f.move_back_DFS(
                        robot,
                        robot.state.current_target,
                        maze_map,
                        fork,
                        fork_number,
                        unused_routes,
                        path,
                    )
                    fork[fork_number].append(robot.state.pos)

                move_f.move_one_position_graph(robot, robot.state.current_target)

                if world.sim.testing:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

                # var.main_event.wait()
                # var.main_event.clear()

        case Mode.SPEEDRUN:

            if robot.state.start:
                path = algorithm_f.read_file(path_dir)
                path.reverse()
                print(len(path))
                path.pop()  # remove start cell

                maze_map = algorithm_f.read_file(maze_dir)
                # var.maze_map_global = maze_map

                # run in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False

            while path:

                if robot.step(world.sim.time_step) == -1:
                    break

                robot.state.current_target = path.pop()

                move_f.move_one_position_graph(robot, robot.state.current_target)

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], {}))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


def BFS_main(robot: MyRobot):
    """Main program for Breadth first search algorithm controller.
    It was adjusted for robot movement. BFS is a horizontal searching through graph
    by going by each 'level' of nodes. To avoid unnecessary back-tracking,
    only forks are treated as 'levels',  which means that robot will go back
    only when it moves to new fork or dead-end. Because of that it doesn't guarantees shortest path.

    Args:
        robot: Object with robot instance.

    Returns:
        None
    """
    maze_map = map_f.init_maze_map_graph()

    maze_map_searched = {}
    for i in range(256):
        maze_map_searched[i] = []

    path_dir, maze_dir = algorithm_f.create_files_directories(
        world.sim.maze_layout, world.sim.algorithm
    )
    # bfs vars
    visited = []
    path = []
    searching_end = False
    move_back = False
    fork = False
    queue = deque()

    visited.append(robot.state.pos)
    queue.append(robot.state.pos)

    draw_queue = Queue(maxsize=1)

    match world.sim.mode:
        case Mode.SEARCH:

            if robot.state.start:
                # run in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False
            if world.sim.testing:
                timer = robot.getTime()

            while queue:

                if robot.step(world.sim.time_step) == -1:
                    break

                if searching_end:
                    robot.state.pos = queue.pop()
                elif move_back or fork:
                    robot.state.pos = queue.popleft()
                    move_back = False
                else:
                    robot.state.pos = queue.pop()

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(robot, 5)
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot.state.pos, robot.state.orientation, walls
                )
                maze_map_searched = map_f.add_walls_graph(
                    maze_map_searched, robot.state.pos, robot.state.orientation, walls
                )

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], {}))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    path = algorithm_f.get_path_BFS(
                        maze_map_searched, world.maze.start_cell, world.maze.target_cell
                    )
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_dir, path)
                    algorithm_f.write_file(maze_dir, maze_map)
                    input("press any key to end")
                    exit(0)

                routes = len(maze_map[robot.state.pos])
                fork = routes >= 3

                robot.state.current_target, visited, queue, move_back, searching_end = (
                    algorithm_f.check_possible_routes_BFS(
                        maze_map[robot.state.pos], visited, queue, fork
                    )
                )

                # not adjacent cell e.g. we move back farther than 1 cell
                if robot.state.current_target not in maze_map[robot.state.pos]:

                    back_path = algorithm_f.get_path_BFS(
                        maze_map_searched, robot.state.pos, robot.state.current_target
                    )

                    while back_path:
                        Move_to = back_path.pop()
                        move_f.move_one_position_graph(robot, Move_to)
                else:
                    move_f.move_one_position_graph(robot, robot.state.current_target)

                if world.sim.testing:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

        case Mode.SPEEDRUN:

            if robot.state.start:
                path = algorithm_f.read_file(path_dir)
                print(len(path))
                maze_map = algorithm_f.read_file(maze_dir)

                # run in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False

            while path:

                if robot.step(world.sim.time_step) == -1:
                    break

                robot.state.current_target = path.pop()

                move_f.move_one_position_graph(robot, robot.state.current_target)

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], {}))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


def A_star_main(robot: MyRobot):
    """Main program for A* algorithm controller.
    Guarantees shortest path, but very long search time.

    Args:
        robot: Object with robot instance.

    Returns:
        None
    """

    maze_map = map_f.init_maze_map_graph()

    maze_map_searched = {}
    for i in range(256):
        maze_map_searched[i] = []

    path_dir, maze_dir = algorithm_f.create_files_directories(
        world.sim.maze_layout, world.sim.algorithm
    )
    # A* vars
    open = []  # list of unvisited nodes
    closed = []  # list of visited nodes
    cost = {}
    parent = {}  # probably not needed anymore
    path = []
    time_sum = 0.0
    robot.state.current_target = robot.state.pos
    cost[robot.state.pos] = [
        0,
        algorithm_f.calc_cost(robot.state.pos, robot.state.current_target),
    ]
    parent[robot.state.pos] = robot.state.pos
    open.append(robot.state.pos)
    draw_queue = Queue(maxsize=1)

    match world.sim.mode:
        case Mode.SEARCH:

            if robot.state.start:
                # run in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False
            if world.sim.testing:
                timer = robot.getTime()

            while open:

                if robot.step(world.sim.time_step) == -1:
                    break

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(robot, 5)
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot.state.pos, robot.state.orientation, walls
                )
                maze_map_searched = map_f.add_walls_graph(
                    maze_map_searched, robot.state.pos, robot.state.orientation, walls
                )

                open.remove(robot.state.pos)
                closed.append(robot.state.pos)

                open, parent, cost = algorithm_f.update_neighbors_costs(
                    maze_map[robot.state.pos], open, closed, parent, cost, robot.state.pos
                )

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], cost))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, world.maze.start_cell, world.maze.target_cell
                    )
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_dir, path)
                    algorithm_f.write_file(maze_dir, maze_map)
                    input("press any key to end")
                    exit(0)

                robot.state.current_target = algorithm_f.check_possible_routes_A_star(open, cost)

                # not neighbor cell e.g. we move back farther than 1 cell
                if robot.state.current_target not in maze_map[robot.state.pos]:

                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, robot.state.pos, robot.state.current_target
                    )
                    while path:
                        Move_to = path.pop(0)
                        move_f.move_one_position_graph(robot, Move_to)

                else:
                    move_f.move_one_position_graph(robot, robot.state.current_target)

                if world.sim.testing:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

        case Mode.SPEEDRUN:

            if robot.state.start:
                path = algorithm_f.read_file(path_dir)
                print(len(path))
                maze_map = algorithm_f.read_file(maze_dir)

                # run in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False

            while path:

                if robot.step(world.sim.time_step) == -1:
                    break

                robot.state.current_target = path.pop(0)

                move_f.move_one_position_graph(robot, robot.state.current_target)

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], {}))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


def A_star_main_modified(robot: MyRobot):
    """Main program for A* modified algorithm controller.
    Modification is that robot chooses where to go in 2 ways:
    1. If current position is fork or dead-end - choose cell with lowest
    Fcost and/or Hcost (just like in normal A*).
    2. If current cell is corridor - keep going until it's fork or dead-end.
    This approach makes searching much faster than normal A*, because robot
    doesn't need to keep moving across whole maze to just check one cell.
    The only drawback is that this approach might not guarantee the shortest path,
    although in micromouse mazes it usually should find it.

    Args:
        robot: Object with robot instance.

    Returns:
        None
    """

    maze_map = map_f.init_maze_map_graph()

    maze_map_searched = {}
    for i in range(256):
        maze_map_searched[i] = []

    path_dir, maze_dir = algorithm_f.create_files_directories(
        world.sim.maze_layout, world.sim.algorithm
    )
    # A* vars
    open = []  # list of unvisited nodes
    closed = []  # list of visited nodes
    cost = {}
    parent = {}  # probably not needed anymore
    path = []

    time_sum = 0.0

    robot.state.current_target = robot.state.pos
    cost[robot.state.pos] = [
        0,
        algorithm_f.calc_cost(robot.state.pos, robot.state.current_target),
    ]
    parent[robot.state.pos] = robot.state.pos
    open.append(robot.state.pos)

    draw_queue = Queue(maxsize=1)

    match world.sim.mode:
        case Mode.SEARCH:

            if robot.state.start:
                # run maze drawing in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False

            if world.sim.testing:
                timer = robot.getTime()

            while open:

                if robot.step(world.sim.time_step) == -1:
                    break

                left_wall, front_wall, right_wall, back_wall = map_f.detect_walls(robot, 5)
                walls = {
                    "front wall": front_wall,
                    "right wall": right_wall,
                    "back wall": back_wall,
                    "left wall": left_wall,
                }

                maze_map = map_f.add_walls_graph(
                    maze_map, robot.state.pos, robot.state.orientation, walls
                )
                maze_map_searched = map_f.add_walls_graph(
                    maze_map_searched, robot.state.pos, robot.state.orientation, walls
                )

                open.remove(robot.state.pos)
                closed.append(robot.state.pos)

                open, parent, cost = algorithm_f.update_neighbors_costs(
                    maze_map[robot.state.pos], open, closed, parent, cost, robot.state.pos
                )

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], cost))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Searching time: %.2f" % robot.getTime(), "s")
                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, world.maze.start_cell, world.maze.target_cell
                    )
                    maze_map = algorithm_f.mark_center_graph(maze_map, path)
                    algorithm_f.write_file(path_dir, path)
                    algorithm_f.write_file(maze_dir, maze_map)
                    input("press any key to end")
                    exit(0)

                routes = len(maze_map[robot.state.pos])
                corridor = routes == 2

                if corridor:
                    robot.state.current_target = open[-1]
                else:
                    robot.state.current_target = algorithm_f.check_possible_routes_A_star(
                        open, cost
                    )

                # not adjacent cell e.g. we move back farther than 1 cell
                if robot.state.current_target not in maze_map[robot.state.pos]:
                    path = algorithm_f.get_path_A_star(
                        maze_map_searched, robot.state.pos, robot.state.current_target
                    )

                    while path:
                        Move_to = path.pop(0)
                        move_f.move_one_position_graph(robot, Move_to)

                else:
                    move_f.move_one_position_graph(robot, robot.state.current_target)

                if world.sim.testing:
                    timer = robot.getTime() - timer
                    print("Move time: %.2f" % timer, "s")

        case Mode.SPEEDRUN:

            if robot.state.start:
                path = algorithm_f.read_file(path_dir)
                print(len(path))
                maze_map = algorithm_f.read_file(maze_dir)

                # run in another thread to make it possible to look on it during robot run
                drawer = MazeDrawer(maze_map, [], draw_queue)
                drawer.start_drawing()
                robot.state.start = False

            while path:

                if robot.step(world.sim.time_step) == -1:
                    break

                robot.state.current_target = path.pop(0)

                move_f.move_one_position_graph(robot, robot.state.current_target)

                draw_queue.put(DrawState(robot.state.pos, maze_map, [], {}))

                if robot.state.pos == world.maze.target_cell:
                    draw_queue.put(None)
                    print("Target reached")
                    print("Speedrun time: %.2f" % robot.getTime(), "s")
                    input("press any key to end")
                    exit(0)


def keyboard_main(robot: MyRobot):
    """Main program for manual controller.
    Made for testing purposes to move robot with WASD.

    Args:
        robot: Object with robot instance.

    Returns:
        None
    """

    keyboard = Keyboard()
    keyboard.enable(world.sim.time_step)
    max_tof = 0
    while robot.step(world.sim.time_step) != -1:

        if world.sim.testing:
            avg_front_sensor = 0
            for i in range(0, 3):  # more scans for better accuracy

                avg_front_sensor += robot.tof.getValue()

                robot.step(world.sim.time_step)  # simulation update

            avg_front_sensor = avg_front_sensor / 3

            print("sensor tof %.2f" % avg_front_sensor)
            if avg_front_sensor > max_tof:
                max_tof = avg_front_sensor
            print("max tof %.2f" % max_tof)

        if world.sim.testing:
            print("sensor ps6 left %.2f" % robot.ps[6].getValue())

        if world.sim.testing:
            print("sensor ps1 right %.2f" % robot.ps[1].getValue())

        key = keyboard.get_key()
        match key:
            case Move.FORWARD:
                print(key)
                move_f.move_1_tile(robot)
            case Move.RIGHT | Move.LEFT | Move.BACK:
                print(key)
                move_f.turn(robot, key)
