from algorithm import AlgorithmInterface
from config.world import world
from config.enums import Direction


class Floodfill(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        self.maze_map = [0] * world.maze.size
        self.distance = [255] * world.maze.size
        self.shortest_path = False
        self.pos = world.maze.start_cell

    def init(self) -> tuple[list | dict, list[int]]:
        self._init_maze_map()
        return self.maze_map, self.distance

    def update(self, maze_map: list[int], distance: list[int]):
        self.maze_map = maze_map
        self.distance = distance
        
        pass

    def finish(self):
        pass

    def _init_maze_map(self):
        """Initialize maze map with external walls.

        Args:
            maze_map: List which contains maze map values.

        Returns:
            list: Initialized maze map list.
        """
        self.maze_map[0] = self.maze_map[0] | world.maze.visited  # mark start as visited

        for i in range(0, 16):
            self.maze_map[i] = self.maze_map[i] | Direction.SOUTH

        for i in range(240, 256):
            self.maze_map[i] = self.maze_map[i] | Direction.NORTH

        for i in range(0, 241, 16):
            self.maze_map[i] = self.maze_map[i] | Direction.WEST

        for i in range(15, 256, 16):
            self.maze_map[i] = self.maze_map[i] | Direction.EAST

        # print_array(self.maze_map, 0)
