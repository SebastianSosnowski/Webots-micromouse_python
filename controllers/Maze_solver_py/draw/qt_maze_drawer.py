from PySide6.QtWidgets import QGraphicsScene, QGraphicsItem, QGraphicsTextItem
from PySide6.QtGui import QPainter, QPen
from PySide6.QtCore import QRectF, Qt

from draw.common import walls_from_bitmask, walls_from_graph, format_position_values
from utils.types import DrawState
from config.enums import Algorithms, Direction
from config.models import AppConfig


class CellItem(QGraphicsItem):
    def __init__(self, row: int, col: int, size: int):
        super().__init__()
        self._row = row
        self._col = col
        self._size = size

        self._walls: dict[Direction, bool] = {
            Direction.NORTH: False,
            Direction.SOUTH: False,
            Direction.EAST: False,
            Direction.WEST: False,
        }

    def boundingRect(self) -> QRectF:
        return QRectF(0, 0, self._size, self._size)

    def set_walls(self, walls: dict[Direction, bool]):
        self._walls = walls
        self.update()

    def paint(self, painter: QPainter, option, widget=None):
        pen = QPen(Qt.GlobalColor.black, 3)
        painter.setPen(pen)

        if self._walls[Direction.NORTH]:
            painter.drawLine(0, 0, self._size, 0)
        if self._walls[Direction.SOUTH]:
            painter.drawLine(0, self._size, self._size, self._size)
        if self._walls[Direction.EAST]:
            painter.drawLine(self._size, 0, self._size, self._size)
        if self._walls[Direction.WEST]:
            painter.drawLine(0, 0, 0, self._size)


class MazeScene(QGraphicsScene):
    Z_GRID = 0
    Z_WALLS = 1
    Z_TEXT = 2
    Z_PATH = 3
    Z_ROBOT = 4

    def __init__(
        self, config: AppConfig, maze_map: list | dict, position_values: list | dict, cell_size=60
    ):
        super().__init__()
        self._cfg = config
        self._maze_map = maze_map
        self._position_values = position_values
        self._cell_size = cell_size

        self._rows = self._cfg.maze.rows
        self._cols = self._cfg.maze.columns
        self._algorithm = self._cfg.simulation.algorithm

        self._grid_items = []
        self._cells = []
        self._text_items = []

        self._init_scene()
        self._init_grid_layer()
        self._init_cell_layer()

    def _init_scene(self):
        width = self._cols * self._cell_size
        height = self._rows * self._cell_size
        self.setSceneRect(0, 0, width, height)

    def _init_grid_layer(self):
        pen = QPen(Qt.GlobalColor.black, 1, Qt.PenStyle.DashDotDotLine)

        for row in range(self._rows):
            for col in range(self._cols):
                rect = self.addRect(
                    col * self._cell_size,
                    row * self._cell_size,
                    self._cell_size,
                    self._cell_size,
                    pen,
                )
                rect.setZValue(self.Z_GRID)
                self._grid_items.append(rect)

    def _init_cell_layer(self):
        for row in range(self._rows):
            for col in range(self._cols):
                cell = CellItem(row, col, self._cell_size)
                qt_row = self._sim_row_to_qt_row(row)

                cell.setPos(col * self._cell_size, qt_row * self._cell_size)
                cell.setZValue(self.Z_WALLS)
                self.addItem(cell)
                self._cells.append(cell)

    def _update_text_layer(self, values: dict[int, list[str]]):
        total_cells = self._rows * self._cols

        for i in range(total_cells):
            # ensure text item exists
            if i >= len(self._text_items):
                text_item = QGraphicsTextItem()
                text_item.setZValue(self.Z_TEXT)
                self.addItem(text_item)
                self._text_items.append(text_item)

            text_item = self._text_items[i]

            # no values for this cell → clear text
            if i not in values:
                text_item.setPlainText("")
                continue

            # set multiline text
            text_item.setPlainText("\n".join(values[i]))

            # position (simulation → Qt coords)
            row_sim = i // self._cols
            col = i % self._cols
            qt_row = self._sim_row_to_qt_row(row_sim)

            text_item.setPos(
                col * self._cell_size + self._cell_size * 0.15,
                qt_row * self._cell_size + self._cell_size * 0.1,
            )

    def update_from_state(self, state: DrawState):
        """
        Update maze visualization from DrawState.
        Called from Qt main thread via signal from SolverWorker.
        """
        # --- update walls ---
        for i, cell in enumerate(self._cells):
            if self._algorithm == Algorithms.FLOODFILL:
                walls = walls_from_bitmask(state.maze_map[i])
            else:
                walls = walls_from_graph(
                    i,
                    state.maze_map[i],
                    self._rows,
                    self._cols,
                )

            cell.set_walls(walls)

        # --- update text / values ---
        if state.position_values is not None:
            formatted_values = format_position_values(state.position_values, self._algorithm)
            self._update_text_layer(formatted_values)

    def _sim_row_to_qt_row(self, row: int) -> int:
        return (self._rows - 1) - row
