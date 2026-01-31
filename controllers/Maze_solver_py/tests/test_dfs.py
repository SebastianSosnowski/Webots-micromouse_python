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
    connections = [8]
    dead_end = dfs_4x4_maze._check_fork(connections, pos)

    assert dead_end is True
    assert dfs_4x4_maze._fork == []


def test_check_fork_add_new_fork(dfs_4x4_maze: DFS):
    pos = 8
    connections = [4, 9, 12]
    dead_end = dfs_4x4_maze._check_fork(connections, pos)

    assert dead_end is False
    assert len(dfs_4x4_maze._fork) == 1
    fork = dfs_4x4_maze._fork[0]
    assert fork.path == [pos]
    assert fork.unused_routes == len(connections) - 2


def test_check_fork_extends_last_fork_path(dfs_4x4_maze: DFS):
    pos = 9
    connections = [8, 10]
    dfs_4x4_maze._fork.append(Fork([8], 1))

    dead_end = dfs_4x4_maze._check_fork(connections, pos)

    assert dead_end is False
    assert len(dfs_4x4_maze._fork) == 1
    fork = dfs_4x4_maze._fork[-1]
    assert fork.path == [8, pos]


def test_move_to_last_fork(dfs_4x4_maze: DFS):
    pos = 15
    connections = [14]
    targets = [9]
    path_to_last_fork = [10, 14]
    dfs_4x4_maze._fork.append(Fork(path_to_last_fork, 1))

    dfs_4x4_maze._check_fork(connections, pos)
    dfs_4x4_maze._move_to_last_fork(targets)

    assert dfs_4x4_maze._pos == targets[-1]
    assert targets == [14, 10, 9]


def test_check_possible_routes_fork(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4->8->9->10
    """
    dfs_4x4_maze._visited = [0, 4, 8, 12, 9, 10]
    dfs_4x4_maze._stack = [12]
    pos = 10

    target = dfs_4x4_maze._check_possible_routes(
        dfs_4x4_maze._maze_map[pos], dfs_4x4_maze._visited, dfs_4x4_maze._stack
    )
    assert target == 14


def test_check_possible_routes_dead_end(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4->8->9->10->14->15->13.
    """
    pos = 13
    dfs_4x4_maze._visited = [0, 4, 8, 12, 9, 10, 14, 11, 6, 13, 15]
    dfs_4x4_maze._stack = [12, 6, 11]

    target = dfs_4x4_maze._check_possible_routes(
        dfs_4x4_maze._maze_map[pos], dfs_4x4_maze._visited, dfs_4x4_maze._stack
    )
    assert target == 11
