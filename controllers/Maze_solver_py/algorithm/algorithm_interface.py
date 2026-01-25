from abc import ABC, abstractmethod


class AlgorithmInterface(ABC):
    @abstractmethod
    def init(self) -> tuple[list | dict, list[int]]:
        pass

    @abstractmethod
    def update(self, maze_map, distance):
        pass

    @abstractmethod
    def finish(self):
        pass
