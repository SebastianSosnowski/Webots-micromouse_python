"""Tests for algorithm_dfs.py."""

import pytest
from algorithm.common import init_maze_map_graph, add_walls_graph
from algorithm.algorithm_dfs import DFS
from utils.params import RobotState, DetectedWalls, Direction, Mode, Algorithms, MazeLayout
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


def test_select_next_position_corridor(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4.
    """
    pos = 4
    dfs_4x4_maze._visited.update([0, pos])
    dfs_4x4_maze._stack = [pos]
    dfs_4x4_maze._parent = {0: None, pos: 1}
    dfs_4x4_maze._pos = pos
    state = RobotState(pos=pos, orientation=Direction.NORTH, current_target=15)
    walls = DetectedWalls(True, False, True, False)
    add_walls_graph(dfs_4x4_maze._maze_map, 4, walls, state)
    next = dfs_4x4_maze._select_next_position()
    assert next == 8


def test_update_start(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm start at position 0.
    """
    dfs_4x4_maze._visited.add(0)
    dfs_4x4_maze._stack = [0]
    dfs_4x4_maze._parent = {0: None}
    state = RobotState(pos=0, orientation=Direction.NORTH, current_target=15)
    detected = DetectedWalls(True, False, True, True)

    targets = dfs_4x4_maze.update(detected, state)

    assert targets == [4]


def test_update_corridor(dfs_4x4_maze: DFS):
    """
    Initial state as would algorithm traversed through positions
    0->4.
    """
    dfs_4x4_maze._visited.update([0, 4])
    dfs_4x4_maze._stack = [4]
    dfs_4x4_maze._parent = {0: None, 4: 1}
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
    dfs_4x4_maze._visited.update([0, 4, 8, 12, 9, 10, 6, 11, 14, 15, 13])
    dfs_4x4_maze._stack = [11, 6, 13]
    dfs_4x4_maze._parent = {
        0: None,
        4: 1,
        8: 4,
        12: 8,
        9: 8,
        10: 9,
        11: 10,
        6: 10,
        14: 10,
        15: 14,
        13: 14,
    }
    dfs_4x4_maze._pos = pos
    state = RobotState(pos=pos, orientation=Direction.WEST, current_target=15)
    detected = DetectedWalls(True, True, True, False)

    targets = dfs_4x4_maze.update(detected, state)

    assert targets == [14, 10, 6]
