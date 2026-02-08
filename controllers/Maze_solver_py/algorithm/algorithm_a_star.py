from algorithm import AlgorithmInterface
from utils.types import RobotState, DetectedWalls, Cost
from config.models import AppConfig
from algorithm.common import (
    init_maze_map_graph,
    add_walls_graph,
    build_path_to_next_target,
    reconstruct_full_path,
)


class AStar(AlgorithmInterface):
    """A* algorithm implementation."""

    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        self._maze_map: dict[int, list[int]] = {}
        # A* vars
        self._open: list[int] = []  # list of unvisited nodes
        self._closed: list[int] = []  # list of visited nodes
        self._cost: dict[int, Cost] = {}
        self._parent = {}  # probably not needed anymore

        self._pos = cfg.maze.start_position
        self._current_target = cfg.maze.target_position

    def init(self):
        self._maze_map = init_maze_map_graph(self._cfg.maze.rows, self._cfg.maze.columns)
        self._open.append(self._pos)
        self._cost[0] = Cost(0, 0)
        self._parent[self._pos] = None

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        add_walls_graph(self._maze_map, self._cfg.maze.rows, detected, state)

        self._open.remove(state.pos)
        self._closed.append(state.pos)
        self._update_neighbors_costs(self._maze_map[state.pos], state.pos)

        self._current_target = self._select_next_position(self._cost)

        path = build_path_to_next_target(
            self._maze_map, self._pos, self._parent, self._current_target
        )
        self._pos = self._current_target
        return path

    def finish(self):
        return self._pos == self._cfg.maze.target_position

    def prepare_results(self) -> tuple[list[int], dict[int, list[int]], dict[int, Cost]]:
        path = reconstruct_full_path(self._parent, self._cfg.maze.target_position)
        return path, self._maze_map, self._cost

    @property
    def maze_map(self) -> dict[int, list[int]]:
        return self._maze_map

    @property
    def position_values(self) -> dict:
        return self._cost

    @property
    def pos(self) -> int:
        return self._pos

    def _select_next_position(self, cost: dict[int, Cost]):
        """Decides to which cell move next. TODO Implement heap to make it much faster.

        Cell with the lowest overall cost (Fcost) is chosen. If more cells have equal
        Fcost, then cell with lowest Hcost (closer to target) is chosen.

        Args:
            open: List with cells to visit.
            cost: Dictionary of costs, which contain Gcost and Hcost.

        Returns:
            int: Variable with a cell to which move next.
        """
        # if there are 2 cells with same Fcost and Hcost, pick last added to open i.e. neighbor

        current_destination = self._open[-1]

        for i in self._open:
            Fcost_i = cost[i].g + cost[i].h
            Fcost_curr = cost[current_destination].g + cost[current_destination].h
            if (Fcost_i < Fcost_curr) or (
                Fcost_i == Fcost_curr and cost[i].h < cost[current_destination].h
            ):
                current_destination = i

        return current_destination

    def _update_neighbors_costs(self, neighbors: list[int], current_position: int):
        """Used in A* algorithm. Assign and/or update costs of neighbor nodes.

        New cost is assigned to node when it's not in open list or new cost is lower than actual.
        In addition parent of the node is assigned, which allows to create path later.
        If target is found, function ends.

        Args:
            neighbors: List with cells adjacent to current position.
            open: List with cells to visit.
            closed: List with cells already visited.
            parent: Dictionary with parent nodes used to create path.
            cost: Dictionary of costs, which contain Gcost and Hcost.
            current_position: Variable with current robot position.

        Returns:
            tuple: (open, parent, cost)
                - open: Updated list with cells to visit.
                - parent: Updated dictionary with parent nodes used to create path.
                - cost: Updated dictionary of costs, which contain Gcost and Hcost.
        """
        for neighbor in neighbors:
            if neighbor in self._closed:
                continue

            new_move_to_neighbor_cost = self._cost[current_position].g + self._calc_cost(
                current_position, neighbor
            )

            # if (neighbor not in open) or new_cost < (cost[neighbor][0] + cost[neighbor][1]):
            if (neighbor not in self._open) or new_move_to_neighbor_cost < self._cost[neighbor].g:
                neighbor_Gcost = new_move_to_neighbor_cost
                neighbor_Hcost = self._calc_cost(neighbor, self._cfg.maze.target_position)
                self._cost[neighbor] = Cost(neighbor_Gcost, neighbor_Hcost)
                self._parent[neighbor] = current_position

                if neighbor not in self._open:
                    self._open.append(neighbor)

                if neighbor == self._cfg.maze.target_position:
                    break

    @staticmethod
    def _calc_cost(start: int, target: int):
        """Calculates Manhattan's distance which is used as cost in A* algorithm.

        Args:
            start: Variable with first position.
            target: Variable with second position.

        Returns:
            int: Manhattan's distance/cost.
        """

        # index to matrix/grid
        point1 = [start % 16, start // 16]
        point2 = [target % 16, target // 16]

        distance = 0
        for x1, x2 in zip(point1, point2):
            difference = x2 - x1
            absolute_difference = abs(difference)
            distance += absolute_difference

        return distance
