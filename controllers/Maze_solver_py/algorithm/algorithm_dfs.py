from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls
from config.world import world


class DFS(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        self._maze_map = {}
        self._fork_number = -1
        self._fork = {}
        self._unused_routes = {}
        self._visited = []
        self._stack = []
        self._path = []
        self._pos = world.maze.start_cell
        self._current_target = world.maze.target_cell

    def init(self) -> None:
        self._visited = self._pos
        self._stack = self._pos

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        return []

    def finish(self):
        pass

    def prepare_results(self) -> tuple[list | dict, list[int]]:
        return ([], [])

    @property
    def maze_map(self) -> list[int] | dict[int, list[int]]:
        return []

    @property
    def distance(self) -> list:
        return []

    @property
    def pos(self) -> int:
        return 0

    def _init_maze_map_graph(self):
        """Initialize maze map with external walls as graph.

        Border positions are initialized with respective walls.
        Inside positions are initialized without any walls i.e. 4 connections.
        """

        rows = world.maze.rows
        cols = world.maze.columns
        size = rows * cols
        left_down_corner = 0
        right_down_corner = rows - 1
        left_up_corner = rows * (cols - 1)
        right_up_corner = size - 1

        # corners
        self._maze_map[left_down_corner] = [rows, 1]
        self._maze_map[right_down_corner] = [right_down_corner + rows, right_down_corner - 1]
        self._maze_map[left_up_corner] = [left_up_corner + 1, left_up_corner - rows]
        self._maze_map[right_up_corner] = [right_up_corner - rows, right_up_corner - 1]

        # down wall cells
        for position in range(left_down_corner + 1, right_down_corner):
            self._maze_map[position] = [position + rows, position + 1, position - 1]
        # up wall cells
        for position in range(left_up_corner + 1, right_up_corner):
            self._maze_map[position] = [position + 1, position - rows, position - 1]
        # left wall cells
        for position in range(left_down_corner + rows, left_up_corner, 16):
            self._maze_map[position] = [position + rows, position + 1, position - rows]
        # right wall cells
        for position in range(right_down_corner + rows, right_up_corner, 16):
            self._maze_map[position] = [position + rows, position - rows, position - 1]

        position = 17
        # inside cells
        while True:

            self._maze_map[position] = [
                position + rows,
                position + 1,
                position - rows,
                position - 1,
            ]

            end = position == (right_up_corner - 1 - rows)
            if end:
                break

            row_end = (position % rows) == 14  # without border cells

            if row_end:  # next row
                position += 3
            else:  # next column
                position += 1
