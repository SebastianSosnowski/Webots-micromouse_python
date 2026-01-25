from algorithm import AlgorithmInterface
from config.world import world
from config.enums import Direction
from utils.params import RobotState


class Floodfill(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        self._maze_map = [0] * world.maze.size
        self._distance = [255] * world.maze.size
        self.shortest_path = False
        self._pos = world.maze.start_cell
        self._current_target = world.maze.target_cell

    def init(self) -> tuple[list | dict, list[int]]:
        self._init_maze_map()
        return self._maze_map, self._distance

    def update(self, maze_map: list[int], state: RobotState):
        self._maze_map = maze_map
        self._init_distance_map(self._current_target)
        self._floodfill()
        target = self._where_to_move(state, self._maze_map[state.pos])
        return [target]

    def finish(self):
        pass

    @property
    def maze_map(self):
        return self._maze_map

    @property
    def distance(self):
        return self._distance

    def _init_maze_map(self):
        """Initialize maze map with external walls.

        Args:
            maze_map: List which contains maze map values.

        Returns:
            list: Initialized maze map list.
        """
        self._maze_map[0] = self._maze_map[0] | world.maze.visited  # mark start as visited

        for i in range(0, 16):
            self._maze_map[i] = self._maze_map[i] | Direction.SOUTH

        for i in range(240, 256):
            self._maze_map[i] = self._maze_map[i] | Direction.NORTH

        for i in range(0, 241, 16):
            self._maze_map[i] = self._maze_map[i] | Direction.WEST

        for i in range(15, 256, 16):
            self._maze_map[i] = self._maze_map[i] | Direction.EAST

        # print_array(self.maze_map, 0)

    def _init_distance_map(self, target: int):
        """Initialize distance map with max values and 0 as target.
        Target is 0 for floodfill algorithm working properly.

        Args:
            distance: List which contains distance values.
            target: Value which contains targeted cell.

        Returns:
            list: Initialized distance list.
        """
        _distance = [world.maze.size - 1] * world.maze.size
        _distance[target] = 0

    def _floodfill(self):
        """Floodfill algorithm which calculates shortest path to actual target based on actual maze map.

        Args:
            maze_map: List with actual maze map with walls.
            distance: List with actual distances values/path.

        Returns:
            list: Updated distance list.
        """

        search = True

        while search:
            search = False

            for i in range(0, world.maze.size):
                if self._distance[i] < 255:

                    if (self._maze_map[i] & Direction.NORTH) != Direction.NORTH:
                        if self._distance[i + world.maze.columns] == 255 or (
                            (self._distance[i] + 1) < self._distance[i + world.maze.columns]
                        ):
                            # update distance value on north tile
                            self._distance[i + world.maze.columns] = self._distance[i] + 1
                            search = True

                    if (self._maze_map[i] & Direction.EAST) != Direction.EAST:
                        if self._distance[i + 1] == 255 or (
                            (self._distance[i] + 1) < self._distance[i + 1]
                        ):
                            self._distance[i + 1] = (
                                self._distance[i] + 1
                            )  # update distance value on EAST tile
                            search = True

                    if (self._maze_map[i] & Direction.WEST) != Direction.WEST:
                        if self._distance[i - 1] == 255 or (
                            (self._distance[i] + 1) < self._distance[i - 1]
                        ):
                            self._distance[i - 1] = (
                                self._distance[i] + 1
                            )  # update distance value on WEST tile
                            search = True
                    # prop unnecessary cuz robot doesn't move backward
                    if (self._maze_map[i] & Direction.SOUTH) != Direction.SOUTH:
                        if self._distance[i - world.maze.columns] == 255 or (
                            (self._distance[i] + 1) < self._distance[i - world.maze.columns]
                        ):
                            self._distance[i - world.maze.columns] = (
                                self._distance[i] + 1
                            )  # update distance value on SOUTH tile
                            search = True

        # print('\n Path ')
        # print_array(distance, 0)
        # print(' Path ')

        # return self._distance

    def _where_to_move(self, state: RobotState, walls: int):
        """
        Decide which neighboring cell to move to based on floodfill distance.
        Preference is given to the cell in front of the robot if distances are equal.

        Returns:
            int: Index of the target cell.
        """
        pos = state.pos
        orientation = state.orientation

        neighbors = {
            Direction.NORTH: pos + world.maze.columns,
            Direction.EAST: pos + 1,
            Direction.SOUTH: pos - world.maze.columns,
            Direction.WEST: pos - 1,
        }

        best_dist = float("inf")
        best_cells: list[Direction] = []

        for direction, neighbor_pos in neighbors.items():
            # skip if wall exists in this direction
            if (walls & direction) == direction:
                continue

            d = self._distance[neighbor_pos]

            if d < best_dist:
                best_dist = d
                best_cells = [direction]
            elif d == best_dist:
                best_cells.append(direction)

        # prefer moving forward if possible
        if orientation in best_cells:
            chosen_dir = orientation
        else:
            chosen_dir = best_cells[0]

        return neighbors[chosen_dir]
