from utils.params import SimulationParams, MazeParams


class World:
    def init(self, sim_cfg, maze_cfg):
        """Initialize the world with simulation and maze parameters.

        Args:
            sim_cfg (dict): Simulation configuration dictionary.
            maze_cfg (dict): Maze configuration dictionary.
        """
        self.sim = SimulationParams(
            sim_cfg["mode"],
            sim_cfg["algorithm"],
            sim_cfg["maze_layout"],
            sim_cfg["testing"],
            sim_cfg["time_step"],
        )
        self.maze = MazeParams(
            maze_cfg["rows"],
            maze_cfg["columns"],
            maze_cfg["rows"] * maze_cfg["columns"],
            maze_cfg["start_cell"],
            maze_cfg["target_cell"],
            maze_cfg["tile_length"],
            maze_cfg["visited_flag"],
        )


world = World()
