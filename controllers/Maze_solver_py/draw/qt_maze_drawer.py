from PySide6.QtWidgets import QGraphicsScene, QGraphicsItem, QGraphicsTextItem, QGraphicsRectItem
from PySide6.QtGui import QPainter, QPen, QFont, QBrush, QColor
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
    Z_VISITED = 0
    Z_GRID = 1
    Z_WALLS = 2
    Z_TEXT = 3
    Z_PATH = 4
    Z_ROBOT = 5

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
        self._visited_items: list[QGraphicsRectItem] = []

        self._grid_items = []
        self._cells = []
        self._cell_text_items: list[dict[str, QGraphicsTextItem]] = []
        self._font_f = QFont("Arial", 16, QFont.Weight.DemiBold)
        self._font_gh = QFont("Arial", 12)
        self._font_floodfill = QFont("Arial", 16, QFont.Weight.Medium)

        self._init_scene()
        self._init_visited_layer()
        self._init_grid_layer()
        self._init_cell_layer()

    def update_from_state(self, state: DrawState):
        """
        Update maze visualization from DrawState.
        Called from Qt main thread via signal from SolverWorker.
        """
        self._update_visited_layer(state.visited)
        self._update_walls_layer(state.maze_map)
        if state.position_values is not None:
            formatted_values = format_position_values(state.position_values, self._algorithm)
            self._update_text_layer(formatted_values)

    def _init_visited_layer(self):
        for row in range(self._rows):
            for col in range(self._cols):
                qt_row = self._sim_row_to_qt_row(row)

                rect = QGraphicsRectItem(
                    col * self._cell_size,
                    qt_row * self._cell_size,
                    self._cell_size,
                    self._cell_size,
                )

                rect.setBrush(QBrush(Qt.GlobalColor.transparent))
                rect.setPen(Qt.PenStyle.NoPen)
                rect.setZValue(self.Z_VISITED)

                self.addItem(rect)
                self._visited_items.append(rect)

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

    def _update_walls_layer(self, maze_map: list | dict):
        for i, cell in enumerate(self._cells):
            if self._algorithm == Algorithms.FLOODFILL:
                walls = walls_from_bitmask(maze_map[i])
            else:
                walls = walls_from_graph(i, maze_map[i], self._rows, self._cols)

            cell.set_walls(walls)

    def _update_visited_layer(self, visited: set[int]):
        visited_brush = QBrush(QColor(180, 255, 180))

        for idx in visited:
            if 0 <= idx < len(self._visited_items):
                self._visited_items[idx].setBrush(visited_brush)

    def _update_text_layer(self, values: dict[int, list[str]]):
        total_cells = self._rows * self._cols

        for i in range(total_cells):
            # --- ensure dict for this cell ---
            if i >= len(self._cell_text_items):
                self._cell_text_items.append({})

            texts = self._cell_text_items[i]

            # --- clear previous texts ---
            for item in texts.values():
                item.setPlainText("")

            if i not in values:
                continue
            # --- compute cell position (simulation -> Qt) ---
            row_sim = i // self._cols
            col = i % self._cols
            qt_row = self._sim_row_to_qt_row(row_sim)

            cell_x = col * self._cell_size
            cell_y = qt_row * self._cell_size

            lines = values[i]

            if self._algorithm == Algorithms.FLOODFILL:
                self._update_text_floodfill(cell_x, cell_y, lines, texts)
            elif self._algorithm == Algorithms.A_STAR or self._algorithm == Algorithms.A_STAR_MOD:
                self._update_text_a_star(cell_x, cell_y, lines, texts)

    def _update_text_floodfill(
        self, cell_x: int, cell_y: int, text_line: list[str], texts: dict[str, QGraphicsTextItem]
    ):
        text = texts.get("d")
        if text is None:
            text = QGraphicsTextItem()
            text.setFont(self._font_floodfill)
            text.setZValue(self.Z_TEXT)
            self.addItem(text)
            texts["d"] = text

        text.setPlainText(text_line[0])
        rect = text.boundingRect()

        x = cell_x + (self._cell_size - rect.width()) / 2
        y = cell_y + (self._cell_size - rect.height()) / 2
        text.setPos(x, y)

    def _update_text_a_star(
        self, cell_x: int, cell_y: int, text_lines: list[str], texts: dict[str, QGraphicsTextItem]
    ):
        f_text, g_text, h_text = text_lines
        padding = 2

        # ---------- F (center, big) ----------
        f_item = texts.get("f")
        if f_item is None:
            f_item = QGraphicsTextItem()
            f_item.setFont(self._font_f)
            f_item.setZValue(self.Z_TEXT)
            self.addItem(f_item)
            texts["f"] = f_item

        f_item.setPlainText(f_text)
        f_rect = f_item.boundingRect()

        fx = cell_x + (self._cell_size - f_rect.width()) / 2
        fy = cell_y + (self._cell_size - f_rect.height()) / 2 + self._cell_size * 0.15
        f_item.setPos(fx, fy)

        # ---------- G (top-left, small) ----------
        g_item = texts.get("g")
        if g_item is None:
            g_item = QGraphicsTextItem()
            g_item.setFont(self._font_gh)
            g_item.setZValue(self.Z_TEXT)
            self.addItem(g_item)
            texts["g"] = g_item

        g_item.setPlainText(g_text)
        g_rect = g_item.boundingRect()

        gx = cell_x + padding
        gy = cell_y + padding
        g_item.setPos(gx, gy)

        # ---------- H (top-right, small) ----------
        h_item = texts.get("h")
        if h_item is None:
            h_item = QGraphicsTextItem()
            h_item.setFont(self._font_gh)
            h_item.setZValue(self.Z_TEXT)
            self.addItem(h_item)
            texts["h"] = h_item

        h_item.setPlainText(h_text)
        h_rect = h_item.boundingRect()

        hx = cell_x + self._cell_size - h_rect.width() - padding
        hy = cell_y + padding
        h_item.setPos(hx, hy)

    def _ensure_text_items_for_cell(self, i: int):
        while i >= len(self._cell_text_items):
            self._cell_text_items.append({})

        return self._cell_text_items[i]

    def _sim_row_to_qt_row(self, row: int) -> int:
        return (self._rows - 1) - row
