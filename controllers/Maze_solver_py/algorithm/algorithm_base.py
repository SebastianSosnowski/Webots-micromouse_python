from algorithm import AlgorithmInterface, Floodfill, Keyboard, DFS, BFS, AStar, AStarMod
from utils.params import RobotState, DetectedWalls
from config.enums import Algorithms
from config.models import AppConfig


class AlgorithmV2(AlgorithmInterface):
    def __init__(self, cfg: AppConfig):
        if cfg.simulation.algorithm == Algorithms.FLOODFILL:
            self.impl = Floodfill(cfg)
        elif cfg.simulation.algorithm == Algorithms.KEYBOARD:
            self.impl = Keyboard(cfg)
        elif cfg.simulation.algorithm == Algorithms.DFS:
            self.impl = DFS(cfg)
        elif cfg.simulation.algorithm == Algorithms.BFS:
            self.impl = BFS(cfg)
        elif cfg.simulation.algorithm == Algorithms.A_STAR:
            self.impl = AStar(cfg)
        elif cfg.simulation.algorithm == Algorithms.A_STAR_MOD:
            self.impl = AStarMod(cfg)
        else:
            raise ValueError(f"Unknown algorithm: {cfg.simulation.algorithm}")

    def init(self) -> None:
        return self.impl.init()

    def update(self, detected: DetectedWalls, state: RobotState) -> list[int]:
        return self.impl.update(detected, state)

    def finish(self) -> bool:
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
