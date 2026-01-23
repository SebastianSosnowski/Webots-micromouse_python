from queue import Queue
from draw.draw_maze import draw_maze
from utils.params import DrawState


def draw_worker(queue: Queue):
    while True:
        state: DrawState = queue.get()
        if state is None:
            break

        draw_maze(
            maze_map=state.maze_map,
            distance=state.distance,
            robot_pos=state.robot_pos,
        )
