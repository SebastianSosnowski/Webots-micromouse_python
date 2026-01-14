from collections import namedtuple

# START Values to change to run program in different modes etc. START

# 1- search, 2 - speedrun
MODE = 1

# 1 - keyboard/manual, 2 - floodfill, 3 - depth first search (DFS),
# 4 - breadth first search (BFS), 5 - A*, 6 - A* modified
ALGORITHM = 2

# 1 - Forbot, 2 - Taiwan2015, 3 - Apec2010, 4 - UK2016, 5 - Higashi2017_mod,
# 6 - Japan2013eq, 7 - Kankou2003, 8 - Japan2011, 9 - Japan1987, 10 - Kor88
MAZE_LAYOUT = 8

# 1- print sensors values etc. for testing, 0 - no prints
TESTING = 0

# only for floodfill WON'T WORK WITH MAZES WHERE THERE ARE UNAVAILABLE CELLS
# True - discover whole maze (guaranteed shortest path, but long searching time),
# False - make 2 runs: start -> target and target -> start (shorter searching time, but shortest path not guaranteed)
WHOLE_SEARCH = False

# END Values to change to run program in different modes etc. END

# Robot parameters
TIME_STEP = 64
TILE_LENGTH = 0.12  # tile length
AXLE = 0.0568  # axle length 0.0568
WHEEL = 0.02002  # wheel radius 0.02002
SPEED = 4

# Modes
SEARCH = 1
SPEEDRUN = 2

# Algorithms
KEYBOARD = 1
FLOODFILL = 2
DFS = 3
BFS = 4
A_STAR = 5
A_STAR_MOD = 6

# Maze Layout
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

# Walls values according to direction (NORTH is not always forward etc.)
WEST = 1  #  00000001
SOUTH = 2  #  00000010
EAST = 4  #  00000100
NORTH = 8  #  00001000
# NORTH =    1    #  00000001
# EAST =     2    #  00000010
# SOUTH =    4    #  00000100
# WEST =     8    #  00001000
# Maze params
ROWS = 16
COLUMNS = 16
MAZE_SIZE = ROWS * COLUMNS
START_CELL = 0
VISITED = 64

# Sets correct target for used mode
if MODE == 2:
    TARGET_CELL = 136
elif MODE == 1:
    if WHOLE_SEARCH:
        TARGET_CELL = 1
    else:
        TARGET_CELL = 136

# Named tuples for more readable code
# Direction = namedtuple("direction", " WEST SOUTH EAST NORTH")
# Direction = Direction(WEST, SOUTH, EAST, NORTH)

Robot_parameters = namedtuple("robot_parameters", "AXLE WHEEL SPEED")
robot_parameters = Robot_parameters(AXLE, WHEEL, SPEED)

Mode_params = namedtuple(
    "mode_params", "SEARCH SPEEDRUN MODE TESTING WHOLE_SEARCH ALGORITHM MAZE_LAYOUT"
)
mode_params = Mode_params(SEARCH, SPEEDRUN, MODE, TESTING, WHOLE_SEARCH, ALGORITHM, MAZE_LAYOUT)

Maze = namedtuple(
    "maze",
    "FORBOT TAIWAN_2015 APEC_2010 UK_2016 HIGASHI_2017 JAPAN_2013EQ KANKOU_2003 JAPAN_2011 JAPAN_1987 KOR_88",
)
maze = Maze(
    FORBOT,
    TAIWAN_2015,
    APEC_2010,
    UK_2016,
    HIGASHI_2017,
    JAPAN_2013EQ,
    KANKOU_2003,
    JAPAN_2011,
    JAPAN_1987,
    KOR_88,
)

Algorithms = namedtuple("algorithms", "KEYBOARD FLOODFILL DFS BFS A_STAR A_STAR_MOD")
algorithms = Algorithms(KEYBOARD, FLOODFILL, DFS, BFS, A_STAR, A_STAR_MOD)

Maze_parameters = namedtuple(
    "maze_parameters", "ROWS COLUMNS MAZE_SIZE VISITED TARGET_CELL TILE_LENGTH START_CELL"
)
maze_parameters = Maze_parameters(
    ROWS, COLUMNS, MAZE_SIZE, VISITED, TARGET_CELL, TILE_LENGTH, START_CELL
)

# Moves = namedtuple("moves", "forward left back right")
# Move = Moves("W", "A", "S", "D")
