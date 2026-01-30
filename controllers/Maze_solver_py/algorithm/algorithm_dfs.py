from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls, Fork
from config.world import world
from algorithm.common import init_maze_map_graph, add_walls_graph


class DFS(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        self._maze_map = {}
        self._fork: list[Fork] = []
        self._visited: list[int] = []
        self._stack: list[int] = []
        self._full_path: list[int] = []
        self._pos = world.maze.start_cell
        self._current_target = world.maze.target_cell

    def init(self) -> None:
        self._maze_map = init_maze_map_graph()
        self._visited.append(self._pos)
        self._stack.append(self._pos)

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        targets = []
        self._stack.pop()
        add_walls_graph(self._maze_map, detected, state)
        dead_end = self._check_fork(self._maze_map[state.pos], state.pos)
        if dead_end:
            self._move_to_previous_fork(targets)
        self._current_target = self._check_possible_routes(
            self._maze_map[self._pos], self._visited, self._stack
        )
        if self._current_target not in self._maze_map[self._pos]:
            self._remove_fork_and_move_back_further(targets)
        targets.append(self._current_target)
        self._pos = targets[-1]
        return targets

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

    def _move_to_previous_fork(self, targets: list[int]):
        """
        Move to previous valid fork and extend targets path.

        Args:
            targets: Path to target position for robot.
        """
        path_back = self._move_back_DFS()
        path_back.reverse()
        targets.extend(path_back)
        self._pos = targets[-1]

    def _remove_fork_and_move_back_further(self, targets: list[int]):
        """
        Remove last fork and extend targets path.

        Args:
            targets: Path to target position for robot.
        """
        self._fork.pop()
        path_back = self._move_back_DFS()
        path_back.reverse()
        targets.extend(path_back)

    def _check_fork(self, connections: list[int], robot_position: int) -> bool:
        """Detect's fork, assign number to it and monitor possible
        unused routes from each one which is used for DFS algorithm operation.

        Also detect's dead-ends.

        Args:
            connections: List with positions available from current position.
            robot_position: Variable with current robot position in maze.

        Returns:
            dead_end: True if current cell is dead-end. Otherwise False
        """
        dead_end = False

        routes = len(connections)
        if routes >= 3:
            self._fork.append(Fork([robot_position], routes - 2))
        elif routes == 2:
            if self._fork:
                self._fork[-1].path.append(robot_position)
        else:
            dead_end = True

        return dead_end

    def _move_back_DFS(self) -> list[int]:
        """Moves back to previous valid fork."""
        return self._fork[-1].path

    def _check_possible_routes(
        self, adjacent_positions: list[int], visited: list[int], stack: list[int]
    ):
        """Add possible adjacent cells to stack and then decides to which cell move next.

        Last item in stack is chosen.

        Args:
            adjacent_positions: Positions accessible from current robot position.
            visited: Positions already added to stack.
            stack: Positions which will be visited.

        Returns:
            current_destination: Position to which move next.
        """
        for cell in reversed(adjacent_positions):
            if cell not in visited:
                visited.append(cell)
                stack.append(cell)
                if cell == world.maze.target_cell:
                    break
        current_destination = stack[-1]

        return current_destination

    def _create_full_path(self):
        """Create full path from forks paths."""
        for fork in self._fork:
            self._full_path.extend(fork.path)
