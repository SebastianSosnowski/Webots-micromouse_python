from algorithm import AStar
from utils.params import RobotState, DetectedWalls
from config.models import AppConfig
from algorithm.common import (
    init_maze_map_graph,
    add_walls_graph,
    build_path_to_next_target,
    reconstruct_full_path,
)


class AStarMod(AStar):
    def _select_next_position(self, cost: dict[int, list[int]]):
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
        if self._is_corridor(self._pos):
            return self._open[-1]
        current_destination = self._open[-1]

        for i in self._open:
            Fcost_i = cost[i][0] + cost[i][1]
            Fcost_curr = cost[current_destination][0] + cost[current_destination][1]
            if (Fcost_i < Fcost_curr) or (
                Fcost_i == Fcost_curr and cost[i][1] < cost[current_destination][1]
            ):
                current_destination = i

        return current_destination

    def _is_corridor(self, pos: int) -> bool:
        unvisited = [n for n in self._maze_map[pos] if n not in self._open]
        return len(unvisited) == 1
