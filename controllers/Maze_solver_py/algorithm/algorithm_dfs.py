from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls, Fork
from algorithm.common import init_maze_map_graph, add_walls_graph
from config.models import AppConfig


class DFS(AlgorithmInterface):
    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._maze_map: dict[int, list[int]] = {}
        self._fork: list[Fork] = []
        self._visited: list[int] = []
        self._stack: list[int] = []
        self._current_path: list[int] = []
        self._full_path: list[int] = []
        self._pos = cfg.maze.start_position
        self._current_target = cfg.maze.target_position

    def init(self) -> None:
        self._maze_map = init_maze_map_graph(self._cfg.maze.rows, self._cfg.maze.columns)
        self._visited.append(self._pos)
        self._current_path.append(self._pos)

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        targets = []

        add_walls_graph(self._maze_map, self._cfg.maze.rows, detected, state)

        dead_end = self._check_fork(self._maze_map[self._pos], self._pos, self._visited)
        if dead_end:
            self._move_to_last_fork(targets)

        self._current_target = self._check_possible_routes(
            self._maze_map[state.pos], self._visited, self._stack
        )

        targets.append(self._current_target)
        self._pos = self._current_target
        self._visited.append(self._pos)
        self._current_path.append(self._pos)
        return targets

    def finish(self) -> bool:
        return self._pos == self._cfg.maze.target_position

    def prepare_results(self) -> tuple[list[int], dict, list]:
        self._current_path.pop(0)
        return self._current_path, self._maze_map, []

    @property
    def maze_map(self) -> dict[int, list[int]]:
        return self._maze_map

    @property
    def position_values(self) -> list:
        return []

    @property
    def pos(self) -> int:
        return self._pos

    def _move_to_last_fork(self, targets: list[int]):
        """
        Move to previous valid fork and extend targets path.

        Use current_path to determine path to fork and trim it afterwards.

        Args:
            targets: Path to target position for robot.
        """
        fork = self._fork[-1]
        fork_pos = fork.position

        idx = self._current_path.index(fork_pos)

        path_back = list(reversed(self._current_path[idx:-1]))
        targets.extend(path_back)
        self._current_path = self._current_path[: idx + 1]
        self._pos = fork_pos

        fork.unused_routes -= 1
        if fork.unused_routes == 0:
            self._fork.pop()

    def _check_fork(self, connections: list[int], robot_position: int, visited: list[int]) -> bool:
        """
        Check whether position is corridor, fork or dead-end.

        Add new Fork if found.

        Args:
            connections: List with positions available from current position.
            robot_position: Variable with current robot position in maze.

        Returns:
            True if current cell is dead-end. Otherwise False
        """
        unvisited = [c for c in connections if c not in visited]

        if not unvisited:
            return True

        if len(unvisited) >= 2:
            self._fork.append(Fork(robot_position, len(unvisited) - 1))

        return False

    def _check_possible_routes(
        self, adjacent_positions: list[int], visited: list[int], stack: list[int]
    ) -> int:
        """Add possible adjacent positions to stack and then decides to which position move next.

        Args:
            adjacent_positions: Positions accessible from current robot position.
            visited: Positions already added to stack.
            stack: Positions which will be visited.

        Returns:
            current_destination: Position to which move next.
        """
        for pos in reversed(adjacent_positions):
            if pos not in visited and pos not in stack:
                stack.append(pos)
                if pos == self._cfg.maze.target_position:
                    break
        return stack.pop()
