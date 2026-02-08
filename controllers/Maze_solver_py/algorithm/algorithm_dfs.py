from algorithm import AlgorithmInterface
from utils.types import RobotState, DetectedWalls
from algorithm.common import (
    init_maze_map_graph,
    add_walls_graph,
    build_path_to_next_target,
    reconstruct_full_path,
)
from config.models import AppConfig


class DFS(AlgorithmInterface):
    """Deep first search algorithm implementation."""

    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._maze_map: dict[int, list[int]] = {}
        self._visited: set[int] = set()
        self._parent: dict[int, int | None] = {}
        self._stack: list[int] = []
        self._pos = cfg.maze.start_position
        self._current_target = cfg.maze.target_position

    def init(self) -> None:
        self._maze_map = init_maze_map_graph(self._cfg.maze.rows, self._cfg.maze.columns)
        self._visited.add(self._pos)
        self._stack.append(self._pos)
        self._parent[self._pos] = None

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        add_walls_graph(self._maze_map, self._cfg.maze.rows, detected, state)

        self._current_target = self._select_next_position()
        path = build_path_to_next_target(
            self._maze_map, self._pos, self._parent, self._current_target
        )
        self._pos = self._current_target
        return path

    def finish(self) -> bool:
        return self._pos == self._cfg.maze.target_position

    def prepare_results(self) -> tuple[list[int], dict, list]:
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

    def _select_next_position(self) -> int:
        """
        Select next position to visit.

        The order of visiting neighbors is related to how neighbors
        positions were initialized in init_maze_map_graph.

        Returns:
            Next position to visit.

        Raises:
            RuntimeError: If stack is empty.
        """
        if not self._stack:
            raise RuntimeError("Stack should not be empty before reaching target!")
        current = self._stack.pop()
        for neighbor in reversed(self._maze_map[current]):
            if neighbor not in self._visited:
                self._visited.add(neighbor)
                self._parent[neighbor] = current
                self._stack.append(neighbor)
        if not self._stack:
            raise RuntimeError("Stack should not be empty before reaching target!")

        return self._stack[-1]
