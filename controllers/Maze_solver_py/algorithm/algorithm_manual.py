from controller import Keyboard

from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls
from config.models import AppConfig
from algorithm.common import init_maze_map_graph, add_walls_graph, reconstruct_full_path
from config.enums import Direction, Move, RelativeDir


class Manual(AlgorithmInterface):
    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._maze_map: dict[int, list[int]] = {}
        self._pos = cfg.maze.start_position
        self._parent: dict[int, int | None] = {}
        self._visited: set[int] = set()
        self._keyboard = Keyboard()

    def init(self):
        self._maze_map = init_maze_map_graph(self._cfg.maze.rows, self._cfg.maze.columns)
        self._parent[self._pos] = None
        self._visited.add(self._pos)
        self._keyboard.enable(self._cfg.simulation.time_step)

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        add_walls_graph(self._maze_map, self._cfg.maze.rows, detected, state)
        key = self._keyboard.get_key()
        if key is None:
            return []

        target = self._select_next_position(key, state)
        if target is None:
            return []

        self._pos = target
        return [target]

    def finish(self):
        return self._pos == self._cfg.maze.target_position

    def prepare_results(self) -> tuple[list[int], list | dict, list | dict]:
        path = reconstruct_full_path(self._parent, self._cfg.maze.target_position)
        return path, self._maze_map, []

    @property
    def maze_map(self) -> dict[int, list[int]]:
        return self._maze_map

    @property
    def position_values(self) -> list:
        return []

    @property
    def pos(self) -> int:
        return self._pos

    def _select_next_position(self, key: str, state: RobotState) -> int | None:
        try:
            move = Move(key)
        except ValueError:
            return None

        rel_dir = {
            Move.FORWARD: RelativeDir.FRONT,
            Move.LEFT: RelativeDir.LEFT,
            Move.RIGHT: RelativeDir.RIGHT,
            Move.BACK: RelativeDir.BACK,
        }.get(move)

        if rel_dir is None:
            return None

        global_dir = state.relative_to_global(rel_dir)

        rows = self._cfg.maze.rows
        neighbors = {
            Direction.NORTH: state.pos + rows,
            Direction.SOUTH: state.pos - rows,
            Direction.EAST: state.pos + 1,
            Direction.WEST: state.pos - 1,
        }

        target = neighbors.get(global_dir)

        for neighbor in reversed(self._maze_map[state.pos]):
            if neighbor not in self._visited:
                self._visited.add(neighbor)
                self._parent[neighbor] = state.pos
        self._parent[self._pos] = target
        if target in self._maze_map[state.pos]:
            return target
        return None
