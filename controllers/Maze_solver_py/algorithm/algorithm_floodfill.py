from algorithm import AlgorithmInterface
from config.enums import Direction
from utils.params import RobotState, DetectedWalls
from config.models import AppConfig

import logging

logger = logging.getLogger(__name__)


class Floodfill(AlgorithmInterface):
    """Floodfill algorithm implementation."""

    def __init__(self, cfg: AppConfig):
        self._cfg = cfg
        size = cfg.maze.rows * cfg.maze.columns
        self._maze_map = [0] * size
        self._distance = [255] * size
        self.shortest_path = False
        self._pos = cfg.maze.start_position
        self._current_target = cfg.maze.target_position

    def init(self) -> None:
        self._init_maze_map()

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        self._update_map(detected, state)
        self._distance = self._init_distance_map(self._distance, self._current_target)
        self._floodfill(self._maze_map, self._distance)
        self._pos = self._where_to_move(state, self._maze_map[state.pos])
        return [self._pos]

    def finish(self) -> bool:
        if self._pos == self._current_target:
            return self._change_target()
        return False

    def prepare_results(self) -> tuple[list[int], list, list]:
        maze_map = self._maze_map.copy()
        # to make sure path will use only visited cells
        size = self._cfg.maze.rows * self._cfg.maze.columns
        for i in range(0, size):
            if (maze_map[i] & self._cfg.maze.visited_flag) != self._cfg.maze.visited_flag:
                maze_map[i] |= 15
        self._current_target = self._cfg.maze.target_position
        self._distance = self._init_distance_map(self._distance, self._current_target)  # reset path
        self._floodfill(maze_map, self._distance)  # path
        path = Floodfill.extract_path(
            self._maze_map,
            self._distance,
            self._cfg.maze.start_position,
            self._cfg.maze.target_position,
            self._cfg.maze.columns,
        )
        return path, self._maze_map, self._distance

    @property
    def maze_map(self) -> list[int]:
        return self._maze_map

    @property
    def position_values(self) -> list:
        return self._distance

    @property
    def pos(self) -> int:
        return self._pos

    def _init_maze_map(self):
        """Initialize maze map with external walls."""
        rows = self._cfg.maze.rows
        cols = self._cfg.maze.columns

        # mark start as visited
        self._maze_map[0] |= self._cfg.maze.visited_flag

        for cell in range(rows * cols):
            row = cell // cols
            col = cell % cols

            if row == 0:
                self._maze_map[cell] |= Direction.SOUTH
            if row == rows - 1:
                self._maze_map[cell] |= Direction.NORTH
            if col == 0:
                self._maze_map[cell] |= Direction.WEST
            if col == cols - 1:
                self._maze_map[cell] |= Direction.EAST

    def _init_distance_map(self, distance: list[int], target: int):
        """Initialize distance map with max values and 0 as target.
        Target is 0 for floodfill algorithm working properly.

        Args:
            distance: List which contains distance values.
            target: Value which contains target position to find.

        Returns:
            list: Initialized distance list.
        """
        distance = [self._cfg.maze.rows * self._cfg.maze.columns - 1] * (
            self._cfg.maze.rows * self._cfg.maze.columns
        )
        distance[target] = 0
        return distance

    def _floodfill(self, maze_map: list[int], distance: list[int]):
        """Floodfill algorithm, which calculates shortest path to
        actual target based on actual maze map.
        The result is updated distance.

        Args:
            maze_map: List with actual maze map with walls.
            distance: List with actual distances values/path.
        """

        search = True

        while search:
            search = False

            for i in range(len(distance)):
                if distance[i] >= 255:
                    continue
                if (maze_map[i] & Direction.NORTH) != Direction.NORTH:
                    if distance[i + self._cfg.maze.columns] == 255 or (
                        (distance[i] + 1) < distance[i + self._cfg.maze.columns]
                    ):
                        # update distance value on north tile
                        distance[i + self._cfg.maze.columns] = distance[i] + 1
                        search = True

                if (maze_map[i] & Direction.EAST) != Direction.EAST:
                    if distance[i + 1] == 255 or ((distance[i] + 1) < distance[i + 1]):
                        distance[i + 1] = distance[i] + 1  # update distance value on EAST tile
                        search = True

                if (maze_map[i] & Direction.WEST) != Direction.WEST:
                    if distance[i - 1] == 255 or ((distance[i] + 1) < distance[i - 1]):
                        distance[i - 1] = distance[i] + 1  # update distance value on WEST tile
                        search = True
                # prop unnecessary cuz robot doesn't move backward
                if (maze_map[i] & Direction.SOUTH) != Direction.SOUTH:
                    if distance[i - self._cfg.maze.columns] == 255 or (
                        (distance[i] + 1) < distance[i - self._cfg.maze.columns]
                    ):
                        # update distance value on SOUTH tile
                        distance[i - self._cfg.maze.columns] = distance[i] + 1
                        search = True

    def _where_to_move(self, state: RobotState, walls: int):
        """
        Decide which neighboring position to move to based on floodfill distance values.
        Preference is given to the position in front of the robot if distances are equal.

        Args:
            state: Dataclass representing the current robot state (position and orientation).
            walls: Bitmask representing walls present at the robot's current position.
                Each bit corresponds to a wall in a global direction:

                - NORTH = 1  (0b0001)
                - EAST  = 2  (0b0010)
                - SOUTH = 4  (0b0100)
                - WEST  = 8  (0b1000)

                Multiple walls are represented by setting multiple bits.
                For example:
                    - walls = 3  (0b0011) → wall to the NORTH and EAST
                    - walls = 12 (0b1100) → wall to the SOUTH and WEST
                    - walls = 15 (0b1111) → walls in all directions

                Wall presence can be checked using bitwise AND, e.g.:
                    (walls & Direction.NORTH) == Direction.NORTH

        Returns:
            int: Index of the target neighboring position to move to.
        """
        pos = state.pos
        orientation = state.orientation

        neighbors = {
            Direction.NORTH: pos + self._cfg.maze.columns,
            Direction.EAST: pos + 1,
            Direction.SOUTH: pos - self._cfg.maze.columns,
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

    def _change_target(self) -> bool:
        """
        Determine whether shortest path was found after reaching current target position.

        Uses currently discovered part of maze.
        Also marks center walls if reached global target position.

        Returns:
            bool: True if found the shortest/ one of the shortest paths. Otherwise False.
        """
        self._distance = self._init_distance_map(self._distance, self._current_target)  # reset path
        self._floodfill(self._maze_map, self._distance)  # path

        shortest_path, possible_path, actual_path = self._is_shortest()

        if self._pos == self._cfg.maze.target_position:
            self._mark_center(self._maze_map)

            if shortest_path:
                logger.debug("This is the shortest/ one of the shortest paths")
            else:
                logger.debug(
                    "There might be a shorter path (%d vs %d), keep going",
                    possible_path,
                    actual_path,
                )
                self._current_target = self._cfg.maze.start_position

        elif self._pos == self._cfg.maze.start_position:
            if shortest_path:
                logger.debug("This is the shortest/ one of the shortest paths")
            else:
                logger.debug(
                    "There might be a shorter path (%d vs %d), keep going",
                    possible_path,
                    actual_path,
                )

            self._current_target = self._cfg.maze.target_position

        return shortest_path

    def _is_shortest(self) -> tuple[bool, int, int]:
        """Determine whether shortest path was found. on currently discovered maze map.

        Use currently discovered part of maze and
        fill all unvisited positions with 4 walls to make them unreachable.
        Then calculate distance and compare it with actual value.

        Returns:
            bool: True if found the shortest/ one of the shortest paths. Otherwise False.

        Raises:
            RuntimeError: If current target is invalid.
        """

        distance_check = self._distance.copy()
        maze_map_check = self._maze_map.copy()

        for i in range(len(maze_map_check)):
            if (maze_map_check[i] & self._cfg.maze.visited_flag) != self._cfg.maze.visited_flag:
                maze_map_check[i] |= 15 | self._cfg.maze.visited_flag

        distance_check = self._init_distance_map(distance_check, self._current_target)  # reset path
        self._floodfill(maze_map_check, distance_check)  # path
        if self._current_target == self._cfg.maze.target_position:
            path_length = self._distance[self._cfg.maze.start_position]
            path_length_check = distance_check[self._cfg.maze.start_position]
        elif self._current_target == self._cfg.maze.start_position:
            path_length = self._distance[self._cfg.maze.target_position]
            path_length_check = distance_check[self._cfg.maze.target_position]
        else:
            raise RuntimeError("Invalid current target")

        is_shortest = path_length >= path_length_check

        return (is_shortest, path_length, path_length_check)

    def _mark_center(self, maze_map: list[int]):
        """Adds walls to unvisited positions in center in maze map."""
        center = [119, 120, 135]

        for center_cell in center:
            if (maze_map[center_cell] & self._cfg.maze.visited_flag) != self._cfg.maze.visited_flag:
                match center_cell:
                    case 119:
                        maze_map[center_cell] = 3
                        maze_map[center_cell - 1] |= Direction.EAST
                        maze_map[center_cell - 16] |= Direction.NORTH
                    case 120:
                        maze_map[center_cell] = 6
                        maze_map[center_cell + 1] |= Direction.WEST
                        maze_map[center_cell - 16] |= Direction.NORTH
                    case 135:
                        maze_map[center_cell] = 9
                        maze_map[center_cell - 1] |= Direction.EAST
                        maze_map[center_cell + 16] |= Direction.SOUTH

    def _update_map(self, detected: DetectedWalls, state: RobotState):
        """
        Update maze map with discovered walls by robot.

        Args:
            detected: A dataclass with information about which walls were detected by robot in current position.
            state: A dataclass with information about current robot state.
        """
        self._maze_map[state.pos] |= self._cfg.maze.visited_flag
        if detected.left_wall:
            self._add_wall(state, self._maze_map, Direction.WEST)

        if detected.front_wall:
            self._add_wall(state, self._maze_map, Direction.NORTH)

        if detected.right_wall:
            self._add_wall(state, self._maze_map, Direction.EAST)

        if detected.back_wall:
            self._add_wall(state, self._maze_map, Direction.SOUTH)

    def _add_wall(self, state: RobotState, maze_map: list[int], detected_wall: int):
        """
        Add wall to maze map on correct positions.
        Depending on robot orientation, value in variable detected_wall
        is shifted so it matches global directions. Then wall is added
        to maze map on robot position and respective neighboring position.

        Args:
            state: A dataclass with information about current robot state.
            maze_map: Actual maze map with walls.
            detected_wall: Value which indicates on which side of robot wall was detected.
        """
        robot_position = state.pos
        orientation = state.orientation

        # shift wall value
        if orientation == Direction.EAST:
            if detected_wall != Direction.WEST:
                detected_wall //= 2
            else:
                detected_wall = Direction.NORTH
        elif orientation == Direction.SOUTH:
            if detected_wall == Direction.WEST or detected_wall == Direction.SOUTH:
                detected_wall *= 4
            else:
                detected_wall //= 4
        elif orientation == Direction.WEST:
            if detected_wall != Direction.NORTH:
                detected_wall *= 2
            else:
                detected_wall = Direction.WEST

        maze_map[robot_position] |= detected_wall  # add sensed wall

        # add wall in neighbor field
        if detected_wall == Direction.NORTH:

            robot_position = robot_position + self._cfg.maze.columns  # upper field
            check = robot_position in range(len(maze_map))

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.SOUTH

        if detected_wall == Direction.EAST:

            robot_position = robot_position + 1  # left field
            check = robot_position in range(len(maze_map))

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.WEST

        if detected_wall == Direction.SOUTH:

            robot_position = robot_position - self._cfg.maze.columns  # lower field
            check = robot_position in range(len(maze_map))

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.NORTH

        if detected_wall == Direction.WEST:

            robot_position = robot_position - 1  # right field
            check = robot_position in range(len(maze_map))

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.EAST

    @staticmethod
    def extract_path(
        maze_map: list[int], distance: list[int], start: int, target: int, columns: int
    ) -> list[int]:
        """
        Extract shortest path from distance map.

        Args:
            maze_map: maze map with walls.
            distance: Distance values calculated by algorithm.
            start: Path start position.
            target: Path target position.

        Returns:
            List of positions creating a path from start to target.
        """
        path = []
        current = start

        while current != target:
            d = distance[current]

            for next_cell in Floodfill._neighbors(current, maze_map, columns):
                if distance[next_cell] == d - 1:
                    path.append(next_cell)
                    current = next_cell
                    break
            else:
                raise RuntimeError("Path reconstruction failed")

        return path

    @staticmethod
    def _neighbors(i: int, maze_map: list[int], columns: int) -> list[int]:
        """
        Return neighboring positions, which are not blocked by wall.

        Args:
            i: Maze position
            maze_map: Maze map with walls.
            columns: Number of columns in maze.

        Returns:
            List of reachable positions.
        """
        n = []

        if (maze_map[i] & Direction.NORTH) != Direction.NORTH:
            n.append(i + columns)
        if (maze_map[i] & Direction.EAST) != Direction.EAST:
            n.append(i + 1)
        if (maze_map[i] & Direction.WEST) != Direction.WEST:
            n.append(i - 1)
        if (maze_map[i] & Direction.SOUTH) != Direction.SOUTH:
            n.append(i - columns)
        return n
