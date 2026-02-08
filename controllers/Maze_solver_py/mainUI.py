import sys
from pathlib import Path
from config.logging_config import setup_logging

setup_logging()
import logging

from PySide6.QtWidgets import QApplication, QGraphicsView, QMainWindow
from PySide6.QtCore import QObject, Qt

from config.loader import load_config
from maze_solver import MazeSolver
from draw.solver_worker import SolverWorker
from draw.qt_maze_drawer import MazeScene
from utils.types import DrawState

logger = logging.getLogger(__name__)


class DrawStatePrinter(QObject):
    """Tymczasowy receiver – tylko printuje DrawState."""

    def on_draw_state(self, state: DrawState):
        logger.info("=== DrawState received ===")
        logger.info("Position: %s", state.robot_pos)
        logger.info("Maze map: %s", state.maze_map)
        logger.info("Position values: %s", state.position_values)
        logger.info("==========================\n")


def main():
    app = QApplication(sys.argv)

    config_path = Path("config.yaml")
    config = load_config(config_path)
    maze_solver = MazeSolver(config)

    worker = SolverWorker(maze_solver)

    # --- scene + view ---
    scene = MazeScene(
        config=config,
        maze_map=maze_solver.algorithm.maze_map,
        position_values=maze_solver.algorithm.position_values,
        cell_size=60,
    )

    # --- view ---
    view = QGraphicsView(scene)

    view.setAlignment(Qt.AlignmentFlag.AlignCenter)
    view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
    view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

    # --- main window ---
    window = QMainWindow()
    window.setWindowTitle("Micromouse Maze")
    window.setCentralWidget(view)

    scene_rect = scene.sceneRect()
    EXTRA_MARGIN = 8

    window.setFixedSize(
        int(scene_rect.width()) + EXTRA_MARGIN,
        int(scene_rect.height()) + EXTRA_MARGIN,
    )

    window.show()

    # --- worker ---
    worker = SolverWorker(maze_solver)
    worker.draw_state.connect(scene.update_from_state)
    worker.finished.connect(app.quit)

    worker.start()

    sys.exit(app.exec())
    # # --- receiver ---
    # printer = DrawStatePrinter()

    # worker.draw_state.connect(printer.on_draw_state)
    # worker.finished.connect(app.quit)

    # worker.start()

    # sys.exit(app.exec())


if __name__ == "__main__":
    main()
