"""Custom data classes used across the project."""

from dataclasses import dataclass
from config.enums import Direction, RelativeDir


@dataclass
class RobotParams:
    axle: float
    wheel: float
    speed: float = 4.0


@dataclass
class RobotState:
    pos: int
    current_target: int
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
    position_values: list | dict
    visited: set[int]


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
class Cost:
    g: int
    h: int
