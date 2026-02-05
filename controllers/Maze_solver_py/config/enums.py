"""Enum classes."""

from enum import Enum, IntEnum, StrEnum, auto


class Mode(str, Enum):
    """Run mode."""

    SEARCH = "SEARCH"
    SPEEDRUN = "SPEEDRUN"


class Algorithms(str, Enum):
    """Algorithm to solve maze."""

    MANUAL = "MANUAL"
    FLOODFILL = "FLOODFILL"
    DFS = "DFS"
    BFS = "BFS"
    A_STAR = "A_STAR"
    A_STAR_MOD = "A_STAR_MOD"


class MazeLayout(str, Enum):
    """Maze layout corresponding to simulation world."""

    FORBOT = "FORBOT"
    TAIWAN_2015 = "TAIWAN_2015"
    APEC_2010 = "APEC_2010"
    UK_2016 = "UK_2016"
    HIGASHI_2017 = "HIGASHI_2017"
    JAPAN_2013EQ = "JAPAN_2013EQ"
    KANKOU_2003 = "KANKOU_2003"
    JAPAN_2011 = "JAPAN_2011"
    JAPAN_1987 = "JAPAN_1987"
    KOR_88 = "KOR_88"


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
