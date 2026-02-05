from collections import deque

from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls
from config.models import AppConfig
from algorithm.common import (
    init_maze_map_graph,
    add_walls_graph,
    build_path_to_next_target,
    reconstruct_full_path,
)


class BFS(AlgorithmInterface):
    """Breadth first search algorithm implementation.

    To avoid unnecessary backtracking, only forks are treated as levels,
    meaning the robot only backtracks when reaching a new fork or dead-end.
    As a result, it does not guarantee the shortest path like the pure BFS algorithm,
    but is much faster to search the maze.
    """

    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._maze_map: dict[int, list[int]] = {}
        self._visited: set[int] = set()
        self._parent: dict[int, int | None] = {}
        self._queue: deque[int] = deque()
        self._dead_end = False
        self._fork = False
        self._pos = cfg.maze.start_position
        self._current_target = cfg.maze.target_position

    def init(self):
        self._maze_map = init_maze_map_graph(self._cfg.maze.rows, self._cfg.maze.columns)
        self._visited.add(self._pos)
        self._queue.append(self._pos)
        self._parent[self._pos] = None

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        add_walls_graph(self._maze_map, self._cfg.maze.rows, detected, state)

        self._current_target = self._select_next_position()
        path = build_path_to_next_target(
            self._maze_map, self._pos, self._parent, self._current_target
        )
        self._pos = self._current_target
        return path

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

    def _select_next_position(self) -> int:
        if self._fork or self._dead_end:
            curr = self._queue.popleft()
        else:
            curr = self._queue.pop()
        self._dead_end = True
        self._fork = self._is_junction(curr)

        for neighbor in self._maze_map[curr]:
            if neighbor not in self._visited:
                self._visited.add(neighbor)
                self._queue.append(neighbor)
                self._parent[neighbor] = curr
                self._dead_end = False
                if neighbor == self._cfg.maze.target_position:
                    break

        if self._fork or self._dead_end:
            return self._queue[0]
        return self._queue[-1]

    def _is_junction(self, pos: int) -> bool:
        unvisited = [n for n in self._maze_map[pos] if n not in self._visited]
        return len(unvisited) >= 2
