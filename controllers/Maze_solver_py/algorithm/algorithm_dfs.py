from algorithm import AlgorithmInterface
from utils.params import RobotState, DetectedWalls
from config.world import world
from algorithm.common import init_maze_map_graph, add_walls_graph


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
        self._maze_map = init_maze_map_graph()
        self._visited = self._pos
        self._stack = self._pos

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        add_walls_graph(self._maze_map, detected, state)
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
