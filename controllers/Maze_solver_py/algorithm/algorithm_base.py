from algorithm import AlgorithmInterface, Floodfill, Keyboard, DFS, BFS, AStar, AStarMod
from utils.params import RobotState, DetectedWalls
from config.enums import Algorithms


class AlgorithmV2(AlgorithmInterface):
    def __init__(self, sim_cfg: dict):
        if sim_cfg["algorithm"] == Algorithms.FLOODFILL:
            self.impl = Floodfill(sim_cfg)
        elif sim_cfg["algorithm"] == Algorithms.KEYBOARD:
            self.impl = Keyboard(sim_cfg)
        elif sim_cfg["algorithm"] == Algorithms.DFS:
            self.impl = DFS(sim_cfg)
        elif sim_cfg["algorithm"] == Algorithms.BFS:
            self.impl = BFS(sim_cfg)
        elif sim_cfg["algorithm"] == Algorithms.A_STAR:
            self.impl = AStar(sim_cfg)
        elif sim_cfg["algorithm"] == Algorithms.A_STAR_MOD:
            self.impl = AStarMod(sim_cfg)
        else:
            raise ValueError(f"Unknown algorithm: {sim_cfg['algorithm']}")

    def init(self) -> None:
        return self.impl.init()

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        return self.impl.update(detected, state)

    def finish(self):
        return self.impl.finish()

    def prepare_results(self) -> tuple[list[int], list | dict, list | dict]:
        return self.impl.prepare_results()

    @property
    def maze_map(self) -> list[int] | dict[int, list[int]]:
        return self.impl.maze_map

    @property
    def distance(self) -> list:
        return self.impl.distance

    @property
    def pos(self) -> int:
        return self.impl.pos
