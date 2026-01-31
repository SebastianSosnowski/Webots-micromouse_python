from algorithm.common import init_maze_map_graph, add_walls_graph
from utils.params import RobotState, DetectedWalls, Direction


def test_init_maze_map_graph_2x2():
    """Test if external walls are initialized correctly"""
    maze_map = init_maze_map_graph(2, 2)

    assert maze_map[0] == [2, 1]
    assert maze_map[1] == [3, 0]
    assert maze_map[2] == [0, 3]
    assert maze_map[3] == [1, 2]


def test_init_maze_map_graph_internal_position():
    """Test if internal position is correctly initialized with all connections"""
    maze_map = init_maze_map_graph(3, 3)

    assert maze_map[4] == [7, 1, 5, 3]


def test_add_walls_graph_orientation():
    """Test if walls are added correctly for different robot orientation"""
    rows = cols = 3

    maze_map = init_maze_map_graph(rows, cols)
    internal_pos = 4
    no_front_wall = DetectedWalls(True, False, True, True)
    orientation_to_connected_pos = {
        Direction.NORTH: 7,
        Direction.EAST: 5,
        Direction.SOUTH: 1,
        Direction.WEST: 3,
    }
    for orientation, connected_pos in orientation_to_connected_pos.items():
        state = RobotState(pos=internal_pos, orientation=orientation, current_target=0)
        add_walls_graph(maze_map, rows, no_front_wall, state)
        assert maze_map[state.pos] == [connected_pos], f"Orientation {orientation.name} failed"
