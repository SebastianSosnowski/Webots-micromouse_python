from abc import ABC, abstractmethod


class MazeSolverBase(ABC):
    def __init__(self):
        super().__init__()

    @abstractmethod
    def init_algorithm(self):
        pass

    def detect_walls(self):
        pass

    @abstractmethod
    def update_map(self):
        pass

    @abstractmethod
    def update_path(self):
        pass

    def move_robot(self):
        pass

    @abstractmethod
    def finished(self):
        pass

    def run(self):
        self.init_algorithm()
        while not self.finished():
            self.detect_walls()
            self.update_map()
            self.update_path()
            self.move_robot()


    