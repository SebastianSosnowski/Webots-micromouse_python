from dataclasses import dataclass
from config.enums import Algorithms, MazeLayout, Mode, Direction, RelativeDir


@dataclass
class RobotParams:
    axle: float
    wheel: float
    speed: float = 4.0


@dataclass
class SimulationParams:
    mode: Mode  # Python enum
    algorithm: Algorithms  # Python enum
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

    _REL_TO_GLOBAL = {
        Direction.NORTH: {
            RelativeDir.FRONT: Direction.NORTH,
            RelativeDir.RIGHT: Direction.EAST,
            RelativeDir.BACK: Direction.SOUTH,
            RelativeDir.LEFT: Direction.WEST,
        },
        Direction.EAST: {
            RelativeDir.FRONT: Direction.EAST,
            RelativeDir.RIGHT: Direction.SOUTH,
            RelativeDir.BACK: Direction.WEST,
            RelativeDir.LEFT: Direction.NORTH,
        },
        Direction.SOUTH: {
            RelativeDir.FRONT: Direction.SOUTH,
            RelativeDir.RIGHT: Direction.WEST,
            RelativeDir.BACK: Direction.NORTH,
            RelativeDir.LEFT: Direction.EAST,
        },
        Direction.WEST: {
            RelativeDir.FRONT: Direction.WEST,
            RelativeDir.RIGHT: Direction.NORTH,
            RelativeDir.BACK: Direction.EAST,
            RelativeDir.LEFT: Direction.SOUTH,
        },
    }

    def relative_to_global(self, rel_dir: RelativeDir) -> Direction:
        return self._REL_TO_GLOBAL[self.orientation][rel_dir]


@dataclass(frozen=True)
class DrawState:
    robot_pos: int
    maze_map: list | dict
    distance: list | dict
    cost: dict


@dataclass
class SensorSnapshot:
    ps: list[float]  # 8 IR
    tof: float  # TOF


@dataclass
class DetectedWalls:
    left_wall: bool
    front_wall: bool
    right_wall: bool
    back_wall: bool

    def items(self):
        return {
            RelativeDir.FRONT: self.front_wall,
            RelativeDir.LEFT: self.left_wall,
            RelativeDir.RIGHT: self.right_wall,
            RelativeDir.BACK: self.back_wall,
        }.items()


@dataclass
class Fork:
    position: int  # Path to the fork between until next fork
    unused_routes: int
