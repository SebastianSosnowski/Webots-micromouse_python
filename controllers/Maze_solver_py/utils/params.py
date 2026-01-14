from dataclasses import dataclass
from config.enums import Algorithm, MazeLayout, Mode, Direction


@dataclass
class RobotParams:
    axle: float
    wheel: float
    speed: float = 4.0


@dataclass
class SimulationParams:
    mode: Mode  # Python enum
    algorithm: Algorithm  # Python enum
    maze_layout: MazeLayout  # Python enum
    testing: bool
    whole_search: bool
    time_step: int = 64


@dataclass
class MazeParams:
    rows: int
    columns: int
    start_cell: int
    target_cell: int
    tile_length: float
    visited_flag: int


@dataclass
class RobotState:
    pos: int
    start: bool = True
    orientation: Direction = Direction.NORTH
