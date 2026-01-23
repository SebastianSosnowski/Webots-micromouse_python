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
    time_step: int = 64


@dataclass
class MazeParams:
    rows: int
    columns: int
    size: int
    start_cell: int
    target_cell: int
    tile_length: float
    visited: int


@dataclass
class RobotState:
    pos: int
    current_target: int
    start: bool = True
    orientation: Direction = Direction.NORTH


@dataclass(frozen=True)
class DrawState:
    robot_pos: int
    robot_orientation: int
    maze_map: list[int] | None
    distance: list[int] | None
    cost_global: dict | None
