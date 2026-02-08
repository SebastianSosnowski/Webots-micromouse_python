from PySide6.QtCore import QThread, Signal
from copy import deepcopy
import logging

from config.enums import Mode
from maze_solver import MazeSolver
from utils.types import DrawState
from read_files.storage import save_results

logger = logging.getLogger(__name__)


class SolverWorker(QThread):
    draw_state = Signal(object)  # DrawState
    finished = Signal()

    def __init__(self, maze_solver: MazeSolver):
        super().__init__()
        self.mz = maze_solver

    def run(self):
        self.mz.algorithm.init()

        path, maze_map, position_values = self.mz.init_drawer_values()

        match self.mz._cfg.simulation.mode:
            case Mode.SEARCH:
                while self.mz.robot.robot.step(self.mz._cfg.simulation.time_step) != -1:

                    detected = self.mz.robot.read_sensors()
                    targets = self.mz.algorithm.update(detected, self.mz.robot.state)

                    while targets:
                        self.mz.visited.add(self.mz.robot.state.pos)
                        self.draw_state.emit(
                            DrawState(
                                self.mz.robot.state.pos,
                                self.mz.prev_pos,
                                deepcopy(self.mz.algorithm.maze_map),
                                deepcopy(self.mz.algorithm.position_values),
                                set(self.mz.visited),
                            )
                        )
                        self.mz.robot.move(targets.pop(0))

                    if self.mz.algorithm.finish():
                        self.draw_state.emit(
                            DrawState(
                                self.mz.robot.state.pos,
                                self.mz.prev_pos,
                                deepcopy(self.mz.algorithm.maze_map),
                                deepcopy(self.mz.algorithm.position_values),
                                set(self.mz.visited),
                            )
                        )

                        logger.info("Target reached")
                        logger.info(
                            "Searching time: %.2f s",
                            self.mz.robot.robot.getTime(),
                        )

                        path, maze_map, position_values = self.mz.algorithm.prepare_results()

                        save_results(
                            path,
                            maze_map,
                            position_values,
                            self.mz._cfg.simulation.maze_layout,
                            self.mz._cfg.simulation.algorithm,
                        )
                        logger.info("Press any key to end")
                        self.finished.emit()
                        return

            case Mode.SPEEDRUN:
                while self.mz.robot.robot.step(self.mz._cfg.simulation.time_step) != -1:
                    if not path:
                        break
                    self.mz.prev_pos = self.mz.robot.state.pos
                    self.mz.robot.move(path.pop(0))
                    self.draw_state.emit(
                        DrawState(
                            self.mz.robot.state.pos,
                            self.mz.prev_pos,
                            maze_map,
                            position_values,
                            set(self.mz.visited),
                        )
                    )

                logger.info("Target reached")
                logger.info(
                    "Speedrun time: %.2f s",
                    self.mz.robot.robot.getTime(),
                )
                logger.info("Press any key to end")

                self.finished.emit()
