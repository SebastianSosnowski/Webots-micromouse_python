import pytest
from algorithm.common import init_maze_map_graph, add_walls_graph
from algorithm.algorithm_dfs import DFS
from utils.params import RobotState, DetectedWalls, Direction, Fork, Mode, Algorithms, MazeLayout
from config.models import AppConfig, MazeConfig, SimulationConfig, RobotConfig


@pytest.fixture
def maze_cfg_4x4() -> AppConfig:
    sim_cfg = SimulationConfig(
        mode=Mode.SEARCH,
        algorithm=Algorithms.DFS,
        maze_layout=MazeLayout.APEC_2010,
        testing=True,
        time_step=64,
    )
    robot_cfg = RobotConfig(model="", axle=0, wheel=0, speed=0)
    maze_cfg = MazeConfig(
        rows=4,
        columns=4,
        start_position=0,
        target_position=15,
        tile_length=0.12,
        visited_flag=64,
    )
    return AppConfig(simulation=sim_cfg, robot=robot_cfg, maze=maze_cfg)


@pytest.fixture
def dfs_4x4_maze(maze_cfg_4x4: AppConfig) -> DFS:
    """
    Fixture with DFS instance.

    Example maze layout 4x4 is created, which covers all crucial combinations:
    o---o---o---o---o
    | 12| 13  14  15|
    o   o---o   o---o
    | 8   9   10  11|
    o   o---o   o---o
    | 4 | 5   6 | 7 |
    o   o   o---o   o
    | 0 | 1   2   3 |
    o---o---o---o---o
    """
    dfs = DFS(maze_cfg_4x4)

    dfs._maze_map = init_maze_map_graph(4, 4)

    walls_by_cell = {
        0: DetectedWalls(left_wall=True, front_wall=False, right_wall=True, back_wall=True),
        1: DetectedWalls(left_wall=True, front_wall=False, right_wall=False, back_wall=True),
        2: DetectedWalls(left_wall=False, front_wall=True, right_wall=False, back_wall=True),
        3: DetectedWalls(left_wall=False, front_wall=False, right_wall=True, back_wall=True),
        4: DetectedWalls(left_wall=True, front_wall=False, right_wall=True, back_wall=False),
        5: DetectedWalls(left_wall=True, front_wall=True, right_wall=False, back_wall=False),
        6: DetectedWalls(left_wall=False, front_wall=False, right_wall=True, back_wall=True),
        7: DetectedWalls(left_wall=True, front_wall=True, right_wall=True, back_wall=False),
        8: DetectedWalls(left_wall=True, front_wall=False, right_wall=False, back_wall=False),
        9: DetectedWalls(left_wall=False, front_wall=True, right_wall=False, back_wall=True),
        10: DetectedWalls(left_wall=False, front_wall=False, right_wall=False, back_wall=False),
        11: DetectedWalls(left_wall=False, front_wall=True, right_wall=True, back_wall=True),
        12: DetectedWalls(left_wall=True, front_wall=True, right_wall=True, back_wall=False),
        13: DetectedWalls(left_wall=True, front_wall=True, right_wall=False, back_wall=True),
        14: DetectedWalls(left_wall=False, front_wall=False, right_wall=False, back_wall=False),
        15: DetectedWalls(left_wall=False, front_wall=True, right_wall=True, back_wall=True),
    }

    for pos, walls in walls_by_cell.items():
        state = RobotState(pos=pos, orientation=Direction.NORTH, current_target=15)
        add_walls_graph(dfs._maze_map, rows=4, detected=walls, state=state)

    return dfs


def test_check_fork_detects_dead_end(dfs_4x4_maze: DFS):
    pos = 12
    dfs_4x4_maze._current_path = [0, 4, 8, 12]
    dfs_4x4_maze._visited = [0, 4, 8, 12]
    connections = [8]

    dead_end = dfs_4x4_maze._check_fork(connections, pos, dfs_4x4_maze._visited)

    assert dead_end is True


def test_check_fork_add_new_fork(dfs_4x4_maze: DFS):
    pos = 8
    dfs_4x4_maze._visited = [0, 4, 8]
    connections = [4, 9, 12]

    dead_end = dfs_4x4_maze._check_fork(connections, pos, dfs_4x4_maze._visited)

    assert dead_end is False
    assert len(dfs_4x4_maze._fork) == 1
    fork = dfs_4x4_maze._fork[0]
    assert fork.position == pos
    assert fork.unused_routes == len(connections) - 2


def test_check_fork_dont_add_new_fork(dfs_4x4_maze: DFS):
    pos = 0
    connections = [4]
    dfs_4x4_maze._visited = [0]

    dead_end = dfs_4x4_maze._check_fork(connections, pos, dfs_4x4_maze._visited)

    assert dead_end is False
    assert len(dfs_4x4_maze._fork) == 0


def test_move_to_last_fork(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4->8->12
    """
    dfs_4x4_maze._current_path = [0, 4, 8, 12]
    dfs_4x4_maze._visited = [0, 4, 8, 12]
    fork_pos = 8
    dfs_4x4_maze._fork.append(Fork(fork_pos, 1))
    targets = []
    dfs_4x4_maze._move_to_last_fork(targets)

    assert targets == [fork_pos]


def test_check_possible_routes_fork(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4->8->12->9->10
    """
    dfs_4x4_maze._visited = [0, 4, 8, 12, 9, 10]
    dfs_4x4_maze._stack = []
    pos = 10

    target = dfs_4x4_maze._check_possible_routes(
        dfs_4x4_maze._maze_map[pos], dfs_4x4_maze._visited, dfs_4x4_maze._stack
    )
    assert target == 14


def test_check_possible_routes_dead_end(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4->8->12->9->10->14->15->13.
    """
    pos = 13
    dfs_4x4_maze._visited = [0, 4, 8, 12, 9, 10, 14, 15, 13]
    dfs_4x4_maze._stack = [6, 11]

    target = dfs_4x4_maze._check_possible_routes(
        dfs_4x4_maze._maze_map[pos], dfs_4x4_maze._visited, dfs_4x4_maze._stack
    )
    assert target == 11


def test_check_possible_routes_found_target(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4->8->12->9->10->14.

    Target should be 13 instead of 15.
    """
    pos = 14
    global_target = dfs_4x4_maze._cfg.maze.target_position = 13
    dfs_4x4_maze._visited = [0, 4, 8, 12, 9, 10, 14]
    dfs_4x4_maze._stack = [6, 11]

    target = dfs_4x4_maze._check_possible_routes(
        dfs_4x4_maze._maze_map[pos], dfs_4x4_maze._visited, dfs_4x4_maze._stack
    )
    assert target == global_target


def test_update_start(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm start at position 0.
    """
    dfs_4x4_maze._visited = [0]
    dfs_4x4_maze._stack = []
    dfs_4x4_maze._current_path = [0]
    state = RobotState(pos=0, orientation=Direction.NORTH, current_target=15)
    detected = DetectedWalls(True, False, True, True)

    targets = dfs_4x4_maze.update(detected, state)

    assert targets == [4]


def test_update_corridor(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4.
    """
    dfs_4x4_maze._visited = [0, 4]
    dfs_4x4_maze._stack = []
    dfs_4x4_maze._current_path = [0, 4]
    dfs_4x4_maze._pos = 4
    state = RobotState(pos=4, orientation=Direction.NORTH, current_target=15)
    detected = DetectedWalls(True, False, True, False)
    targets = dfs_4x4_maze.update(detected, state)

    assert targets == [8]


def test_update_dead_end(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4->8->12->9->10->14->15->13.
    """
    pos = 13
    dfs_4x4_maze._visited = [0, 4, 8, 12, 9, 10, 14, 15, 13]
    dfs_4x4_maze._stack = [6, 11]
    dfs_4x4_maze._current_path = [0, 4, 8, 9, 10, 14, 13]
    dfs_4x4_maze._pos = pos
    dfs_4x4_maze._fork.append(Fork(10, 2))
    state = RobotState(pos=pos, orientation=Direction.NORTH, current_target=15)
    detected = DetectedWalls(True, False, True, False)

    targets = dfs_4x4_maze.update(detected, state)

    assert targets == [14, 10, 11]
