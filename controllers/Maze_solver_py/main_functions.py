from collections import deque
from queue import Queue
from copy import deepcopy

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
from read_files.storage import save_results


def interface_main(mz: MazeSolver):
    mz.algorithm.init()

    path, maze_map, position_values = mz.init_drawer_values()

    draw_queue = Queue(maxsize=1)
    drawer = MazeDrawer(mz._cfg, deepcopy(maze_map), deepcopy(position_values), draw_queue)
    drawer.start_drawing()
    match mz._cfg.simulation.mode:
        case Mode.SEARCH:
            while mz.robot.robot.step(mz._cfg.simulation.time_step) != -1:
                detected = mz.robot.read_sensors()
                targets = mz.algorithm.update(detected, mz.robot.state)
                while targets:
                    draw_queue.put(
                        DrawState(
                            mz.robot.state.pos,
                            deepcopy(mz.algorithm.maze_map),
                            deepcopy(mz.algorithm.position_values),
                        )
                    )
                    mz.robot.move(targets.pop(0))
                if mz.algorithm.finish():
                    draw_queue.put(
                        DrawState(
                            mz.robot.state.pos,
                            deepcopy(mz.algorithm.maze_map),
                            deepcopy(mz.algorithm.position_values),
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
                draw_queue.put(DrawState(mz.robot.state.pos, maze_map, position_values))
                mz.robot.move(path.pop(0))
            draw_queue.put(DrawState(mz.robot.state.pos, maze_map, position_values))
            draw_queue.put(None)
            print("Target reached")
            print("Speedrun time: %.2f" % mz.robot.robot.getTime(), "s")

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
