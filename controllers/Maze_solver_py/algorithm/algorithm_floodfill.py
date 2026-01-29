from algorithm import AlgorithmInterface
from config.world import world
from config.enums import Direction
from utils.params import RobotState, DetectedWalls


class Floodfill(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        self._maze_map = [0] * world.maze.size
        self._distance = [255] * world.maze.size
        self.shortest_path = False
        self._pos = world.maze.start_cell
        self._current_target = world.maze.target_cell

    def init(self):
        self._init_maze_map()

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        self._update_map(detected, state)
        self._distance = self._init_distance_map(self._distance, self._current_target)
        self._floodfill(self._maze_map, self._distance)
        self._pos = self._where_to_move(state, self._maze_map[state.pos])
        return [self._pos]

    def finish(self):
        if self._pos == self._current_target:
            return self._change_target()
        return False

    def prepare_results(self) -> tuple[list[int], list | dict, list | dict]:
        maze_map = self._maze_map.copy()
        # to make sure path will use only visited cells
        for i in range(0, world.maze.size):
            if (maze_map[i] & world.maze.visited) != world.maze.visited:
                maze_map[i] |= 15
        self._current_target = world.maze.target_cell
        self._distance = self._init_distance_map(self._distance, self._current_target)  # reset path
        self._floodfill(maze_map, self._distance)  # path
        path = Floodfill.extract_path(
            self._maze_map, self._distance, world.maze.start_cell, world.maze.target_cell
        )
        return path, self._maze_map, self._distance

    @property
    def maze_map(self) -> list[int] | dict[int, list[int]]:
        return self._maze_map

    @property
    def distance(self) -> list:
        return self._distance

    @property
    def pos(self) -> int:
        return self._pos

    def _init_maze_map(self):
        """Initialize maze map with external walls."""
        self._maze_map[0] = self._maze_map[0] | world.maze.visited  # mark start as visited

        for i in range(0, 16):
            self._maze_map[i] = self._maze_map[i] | Direction.SOUTH

        for i in range(240, 256):
            self._maze_map[i] = self._maze_map[i] | Direction.NORTH

        for i in range(0, 241, 16):
            self._maze_map[i] = self._maze_map[i] | Direction.WEST

        for i in range(15, 256, 16):
            self._maze_map[i] = self._maze_map[i] | Direction.EAST

    def _init_distance_map(self, distance: list[int], target: int):
        """Initialize distance map with max values and 0 as target.
        Target is 0 for floodfill algorithm working properly.

        Args:
            distance: List which contains distance values.
            target: Value which contains target position to find.

        Returns:
            list: Initialized distance list.
        """
        distance = [world.maze.size - 1] * world.maze.size
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

            for i in range(0, world.maze.size):
                if distance[i] >= 255:
                    continue
                if (maze_map[i] & Direction.NORTH) != Direction.NORTH:
                    if distance[i + world.maze.columns] == 255 or (
                        (distance[i] + 1) < distance[i + world.maze.columns]
                    ):
                        # update distance value on north tile
                        distance[i + world.maze.columns] = distance[i] + 1
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
                    if distance[i - world.maze.columns] == 255 or (
                        (distance[i] + 1) < distance[i - world.maze.columns]
                    ):
                        # update distance value on SOUTH tile
                        distance[i - world.maze.columns] = distance[i] + 1
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

        shortest_path = self._is_shortest()

        if self._pos == world.maze.target_cell:
            self._mark_center(self._maze_map)

            if shortest_path:
                print("This is the shortest/ one of the shortest paths")
            else:
                print("There might be a shorter path, keep going")
                self._current_target = world.maze.start_cell

        elif self._pos == world.maze.start_cell:
            if shortest_path:
                print("This is the shortest/ one of the shortest paths")
            else:
                print("There might be a shorter path, keep going")

            self._current_target = world.maze.target_cell

        return shortest_path

    def _is_shortest(self) -> bool:
        """Determine whether shortest path was found. on currently discovered maze map.

        Use currently discovered part of maze and
        fill all unvisited positions with 4 walls to make them unreachable.
        Then calculate distance and compare it with actual value.

        Returns:
            bool: True if found the shortest/ one of the shortest paths. Otherwise False.
        """

        distance_check = self._distance.copy()
        maze_map_check = self._maze_map.copy()

        for i in range(0, world.maze.size):
            if (maze_map_check[i] & world.maze.visited) != world.maze.visited:
                maze_map_check[i] |= 15 | world.maze.visited

        distance_check = self._init_distance_map(distance_check, self._current_target)  # reset path
        self._floodfill(maze_map_check, distance_check)  # path
        if self._current_target == world.maze.target_cell:
            shortest_path = self._distance[0] >= distance_check[0]  # could be just equal'
        elif self._current_target == world.maze.start_cell:
            shortest_path = self._distance[136] >= distance_check[136]  # could be just equal'

        return shortest_path

    def _mark_center(self, maze_map: list[int]):
        """Adds walls to unvisited positions in center in maze map."""
        center = [119, 120, 135]

        for center_cell in center:
            if (maze_map[center_cell] & world.maze.visited) != world.maze.visited:
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
        self._maze_map[state.pos] |= world.maze.visited
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

            robot_position = robot_position + world.maze.columns  # upper field
            check = robot_position in range(0, world.maze.size)

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.SOUTH

        if detected_wall == Direction.EAST:

            robot_position = robot_position + 1  # left field
            check = robot_position in range(0, world.maze.size)

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.WEST

        if detected_wall == Direction.SOUTH:

            robot_position = robot_position - world.maze.columns  # lower field
            check = robot_position in range(0, world.maze.size)

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.NORTH

        if detected_wall == Direction.WEST:

            robot_position = robot_position - 1  # right field
            check = robot_position in range(0, world.maze.size)

            if check:
                maze_map[robot_position] = maze_map[robot_position] | Direction.EAST

    @staticmethod
    def extract_path(
        maze_map: list[int], distance: list[int], start: int, target: int
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

            for next_cell in Floodfill._neighbors(current, maze_map):
                if distance[next_cell] == d - 1:
                    path.append(next_cell)
                    current = next_cell
                    break
            else:
                raise RuntimeError("Path reconstruction failed")

        return path

    @staticmethod
    def _neighbors(i: int, maze_map: list[int]) -> list[int]:
        """
        Return neighboring positions, which are not blocked by wall.

        Args:
            i: Maze position
            maze_map: Maze map with walls.

        Returns:
            List of reachable positions.
        """
        n = []

        if (maze_map[i] & Direction.NORTH) != Direction.NORTH:
            n.append(i + world.maze.columns)
        if (maze_map[i] & Direction.EAST) != Direction.EAST:
            n.append(i + 1)
        if (maze_map[i] & Direction.WEST) != Direction.WEST:
            n.append(i - 1)
        if (maze_map[i] & Direction.SOUTH) != Direction.SOUTH:
            n.append(i - world.maze.columns)
        return n
