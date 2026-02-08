import sys
from pathlib import Path
from config.logging_config import setup_logging

setup_logging()
import logging

from PySide6.QtWidgets import QApplication, QGraphicsView
from PySide6.QtCore import Qt, QSettings

from config.loader import load_config
from maze_solver import MazeSolver
from draw.solver_worker import SolverWorker
from draw.qt_maze_drawer import MazeScene, MainWindow

logger = logging.getLogger(__name__)


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
    window = MainWindow()
    window.setWindowTitle("Micromouse Maze")
    window.setCentralWidget(view)

    scene_rect = scene.sceneRect()
    EXTRA_MARGIN = 8

    window.setMinimumSize(
        int(scene_rect.width()) + EXTRA_MARGIN, int(scene_rect.height()) + EXTRA_MARGIN
    )

    window.show()

    # --- worker ---
    worker = SolverWorker(maze_solver)
    worker.draw_state.connect(scene.update_from_state)

    worker.start()

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
