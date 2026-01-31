from enum import Enum, IntEnum, StrEnum, auto


class Mode(IntEnum):
    """Run mode."""

    SEARCH = 1
    SPEEDRUN = 2


class Algorithms(IntEnum):
    """Algorithm to solve maze."""

    KEYBOARD = 1
    FLOODFILL = 2
    DFS = 3
    BFS = 4
    A_STAR = 5
    A_STAR_MOD = 6


class MazeLayout(IntEnum):
    """Maze layout corresponding to simulation world."""

    FORBOT = 1
    TAIWAN_2015 = 2
    APEC_2010 = 3
    UK_2016 = 4
    HIGASHI_2017 = 5
    JAPAN_2013EQ = 6
    KANKOU_2003 = 7
    JAPAN_2011 = 8
    JAPAN_1987 = 9
    KOR_88 = 10


class Direction(IntEnum):
    """Wall value according to its global direction."""

    WEST = 1  #  00000001
    SOUTH = 2  #  00000010
    EAST = 4  #  00000100
    NORTH = 8  #  00001000
    # NORTH = 1  #  00000001
    # EAST = 2  #  00000010
    # SOUTH = 4  #  00000100
    # WEST = 8  #  00001000


class RelativeDir(Enum):
    FRONT = auto()
    LEFT = auto()
    RIGHT = auto()
    BACK = auto()


class Move(StrEnum):
    FORWARD = "W"
    LEFT = "A"
    BACK = "S"
    RIGHT = "D"
