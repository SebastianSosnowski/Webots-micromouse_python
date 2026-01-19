from threading import Event
from config.world import world

distance_update = False
map_update = False
searching_end = False

robot_pos = 0  # world.maze.start_cell
maze_map_global = [0] * 256  # world.maze.size
distance_global = [255] * 256  # world.maze.size
target_global = 136  # world.maze.target_cell
cost_global = {}

drawing_event = Event()
main_event = Event()
