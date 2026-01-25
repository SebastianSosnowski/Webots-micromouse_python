from algorithm import AlgorithmInterface, Floodfill, Keyboard, DFS, BFS, AStar, AStarMod
from utils.params import RobotState


class AlgorithmV2(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        if sim_cfg["algorithm"] == "FLOODFILL":
            self.impl = Floodfill(sim_cfg)
        elif sim_cfg["algorithm"] == "KEYBOARD":
            self.impl = Keyboard(sim_cfg)
        elif sim_cfg["algorithm"] == "DFS":
            self.impl = DFS(sim_cfg)
        elif sim_cfg["algorithm"] == "BFS":
            self.impl = BFS(sim_cfg)
        elif sim_cfg["algorithm"] == "A_STAR":
            self.impl = AStar(sim_cfg)
        elif sim_cfg["algorithm"] == "A_STAR_MOD":
            self.impl = AStarMod(sim_cfg)
        else:
            raise ValueError(f"Unknown algorithm: {sim_cfg['algorithm']}")

    def init(self) -> tuple[list | dict, list[int]]:
        return self.impl.init()

    def update(self, maze_map, state: RobotState) -> list[int]:
        return self.impl.update(maze_map, state)

    def finish(self):
        self.impl.finish()
