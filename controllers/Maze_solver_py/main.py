"""Maze Solver controller."""

from config.logging_config import setup_logging

setup_logging()
import logging

logger = logging.getLogger(__name__)

from pathlib import Path
from queue import Queue
from copy import deepcopy

from config.loader import load_config
from config.enums import Mode
from maze_solver import MazeSolver
from draw.maze_drawer import MazeDrawer
from utils.params import DrawState
from read_files.storage import save_results


def run_robot(mz: MazeSolver):
    """Run the robot based on the configured algorithm.

    Args:
        robot: The robot instance to control.
    """
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
                    logger.info("Target reached")
                    logger.info("Searching time: %.2f s", mz.robot.robot.getTime())
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
            logger.info("Target reached")
            logger.info("Speedrun time: %.2f s", mz.robot.robot.getTime())

    input("press any key to end")
    exit(0)


if __name__ == "__main__":
    config_path = Path("config.yaml")

    config = load_config(config_path)
    maze_solver = MazeSolver(config)

    run_robot(maze_solver)
