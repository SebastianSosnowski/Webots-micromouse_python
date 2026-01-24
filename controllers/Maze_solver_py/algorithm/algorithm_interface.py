from abc import ABC, abstractmethod


class AlgorithmInterface(ABC):
    @abstractmethod
    def init(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def finish(self):
        pass
