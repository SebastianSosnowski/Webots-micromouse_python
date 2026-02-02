from queue import Queue
from threading import Thread
from turtle import done, setup, tracer, Turtle, update

from config.enums import Algorithms, Direction, Mode
from utils.params import DrawState
from config.models import AppConfig


class MazeDrawer(Thread):
    def __init__(
        self,
        config: AppConfig,
        maze_map: list | dict,
        position_values: list | dict,
        draw_queue: Queue,
    ) -> None:
        """Initialize the MazeDrawer thread.

        Args:
            maze_map: Initial maze map.
            distance: Initial distance map.
            draw_queue (Queue): Queue for drawing updates.
        """
        super().__init__(daemon=True)
        self._cfg = config
        self.draw_queue = draw_queue
        self.maze_map = maze_map
        self.position_values = position_values
        self.size = 60
        self.last_x = 0
        self.last_y = 0
        self.robot_pos = self._cfg.maze.start_position
        self.distance_update = False

    def start_drawing(self):
        """Start the drawing thread."""
        self.start()

    def run(self):
        """Run the drawing loop, processing updates from the queue."""
        self.robot_pos_canvas, self.path_canvas = self._init_canvas()
        self.text_canvas, self.maze_canvas = self._init_maze(self.maze_map, self.position_values)

        while True:
            msg: DrawState = self.draw_queue.get()

            if msg is None:
                break
            self._update_state(msg)

            if self._cfg.simulation.mode == Mode.SEARCH:
                self._draw_search_frame()
            else:
                self._draw_speedrun_frame()
        done()

    def _init_canvas(self):
        """Initialize the drawing canvases for circles and lines."""
        circles = Turtle()
        circles.pencolor("red")
        circles.hideturtle()
        circles.width(4)
        circles.speed(0)

        lines = Turtle()
        lines.pencolor("red")
        lines.hideturtle()
        lines.width(4)
        lines.speed(0)

        return circles, lines

    def _init_maze(self, maze_map, distance):
        """Init maze window with grid and starting walls.

        Args:
            maze_map (list): List with initial maze map with walls.
            distance (list): List with initial distances values/path.

        Returns:
            tuple: Objects with maze text/numbers and maze walls.
        """
        setup(1020, 1020, 1500, 160)  # size and position on screen
        maze = Turtle()
        maze.hideturtle()
        tracer(False)
        maze.color("black")
        maze.width(1)

        grid = Turtle()
        grid.hideturtle()
        grid.color("black")
        grid.width(1)

        text = Turtle()
        text.hideturtle()
        text.width(1)

        # draw grid
        for y in range(-480, 480, self.size):
            for x in range(-480, 480, self.size):
                line(x, y + self.size, x + self.size, y + self.size, grid)
                line(x + self.size, y, x + self.size, y + self.size, grid)
                line(x, y, x + self.size, y, grid)
                line(x, y, x, y + self.size, grid)

        maze.width(5)
        # draw walls
        i = 0
        if self._cfg.simulation.algorithm == Algorithms.FLOODFILL:
            for y in range(-480, 480, self.size):
                for x in range(-480, 480, self.size):
                    write_distance(x, y, distance[i], text)  # write initial distance values
                    if maze_map[i] < 64:  # unvisited cell
                        draw_wall(maze_map[i], x, y, self.size, maze)
                    else:
                        draw_wall(maze_map[i] - 64, x, y, self.size, maze)
                    i += 1
        else:  # graph Algorithm
            for y in range(-480, 480, self.size):
                for x in range(-480, 480, self.size):
                    cell = graph_walls_convert(maze_map[i], i)
                    draw_wall(cell, x, y, self.size, maze)
                    i += 1
        if self._cfg.simulation.mode == Mode.SEARCH:
            self._draw_position(self.robot_pos_canvas)
        return text, maze

    def _update_state(self, msg: DrawState):
        """Update the internal state based on the draw message.

        Args:
            msg (DrawState): The draw state message.
        """
        if msg.position_values != self.position_values:
            self.distance_update = True
            self.position_values = msg.position_values
        self.maze_map = msg.maze_map
        self.robot_pos = msg.robot_pos

    def _draw_search_frame(self):
        """Update maze visualization with visited cells and discovered walls and distance values.
        For floodfill distance values are also drawn.
        For A* costs values are also drawn.

        Returns:
            None
        """
        self._draw_position(self.robot_pos_canvas)

        xx = self.robot_pos % 16
        xx = -480 + xx * self.size
        yy = self.robot_pos // 16
        yy = -480 + yy * self.size

        if self._cfg.simulation.algorithm == Algorithms.FLOODFILL:
            draw_wall(self.maze_map[self.robot_pos] - 64, xx, yy, self.size, self.maze_canvas)
            if self.distance_update:
                i = 0
                self.text_canvas.clear()
                for y in range(-480, 480, self.size):
                    for x in range(-480, 480, self.size):
                        write_distance(x, y, self.position_values[i], self.text_canvas)
                        i += 1
                self.distance_update = False
        else:  # graphs
            cell = graph_walls_convert(self.maze_map[self.robot_pos], self.robot_pos)
            draw_wall(cell, xx, yy, self.size, self.maze_canvas)

            if (
                self._cfg.simulation.algorithm == Algorithms.A_STAR
                or self._cfg.simulation.algorithm == Algorithms.A_STAR_MOD
            ):
                self.text_canvas.clear()
                for key in self.position_values:
                    x = key % 16
                    x = -480 + x * self.size
                    y = key // 16
                    y = -480 + y * self.size
                    write_cost(x, y, self.position_values[key], self.text_canvas)

        if self.robot_pos == 136:
            self._draw_center()

        update()

    def _draw_speedrun_frame(self):
        """Update maze visualization with actual robot position and path.

        Returns:
            None
        """
        self.robot_pos_canvas.clear()
        self._draw_path(self.path_canvas)
        self._draw_position(self.robot_pos_canvas)

        update()

    def _draw_center(self):
        """Update maze visualization with center cells at the end according to visited cells.

        Returns:
            None
        """
        center = [119, 120, 135]
        for center_cell in center:
            if self._cfg.simulation.algorithm == Algorithms.FLOODFILL:
                check = (
                    self.maze_map[center_cell] & self._cfg.maze.visited_flag
                ) != self._cfg.maze.visited_flag
            else:  # graphs Algorithm
                cell = self.maze_map[center_cell]
                if isinstance(cell, list):
                    check = len(cell) == 0  # inside unvisited nodes have 4 edges
                # check = len(self.maze_map[center_cell]) == 0  # inside unvisited nodes have 4 edges
            if check:
                x = center_cell % 16
                x = -480 + x * self.size
                y = center_cell // 16
                y = -480 + y * self.size
                match center_cell:
                    case 119:
                        draw_wall(3, x, y, self.size, self.maze_canvas)
                    case 120:
                        draw_wall(6, x, y, self.size, self.maze_canvas)
                    case 135:
                        draw_wall(9, x, y, self.size, self.maze_canvas)

    def _draw_position(self, t: Turtle):
        """Draw mark in maze cell where robot is right now.

        Args:
            t (Turtle): Corresponding turtle object.

        Returns:
            None
        """
        x = self.robot_pos % self._cfg.maze.columns
        y = int(self.robot_pos / self._cfg.maze.rows)
        t.penup()
        t.goto(-450 + x * self.size, -450 + y * self.size - 6)  # last position
        t.pendown()
        t.fillcolor("red")
        t.begin_fill()
        t.circle(6)
        t.end_fill()

    def _draw_path(self, t: Turtle):
        """Draw robot path.

        Args:
            t (Turtle): Corresponding turtle object.

        Returns:
            None
        """
        next_x = self.robot_pos % 16
        next_y = int(self.robot_pos / 16)
        t.penup()
        t.goto(-450 + self.last_x * self.size, -450 + self.last_y * self.size)  # last position
        t.pendown()
        line(
            -450 + self.last_x * self.size,
            -450 + self.last_y * self.size,
            -450 + next_x * self.size,
            -450 + next_y * self.size,
            t,
        )
        self.last_x, self.last_y = next_x, next_y


def line(start_x: float, start_y: float, end_x: float, end_y: float, t: Turtle):
    """Draw line.

    Args:
        start_x: Variable with line beginning x coordinate.
        start_y: Variable with line beginning y coordinate.
        end_x: Variable with line ending x coordinate.
        end_y: Variable with line ending y coordinate.
        t: Corresponding turtle object.

    Returns:
        None
    """
    t.up()
    t.goto(start_x, start_y)
    t.down()
    t.goto(end_x, end_y)


def write_distance(x: float, y: float, distance: int, t: Turtle):
    """Write distance value in maze cell.

    Args:
        x: Variable with text x coordinate.
        y: Variable with text y coordinate.
        distance: Variable with distance value to target.
        t: Corresponding turtle object.

    Returns:
        None
    """
    t.penup()
    t.goto(x + 8, y + 16)
    t.write("%i" % distance, font=("Verdana", 13, "bold"))


def draw_wall(maze_map: int, x: float, y: float, size: float, t: Turtle):
    """Draw corresponding walls and values in each field.

    Args:
        maze_map: Value with actual maze map walls for this field.
        x: Variable with offset in x direction.
        y: Variable with offset in y direction.
        size: Value with one field size (for easier change when changing window size).
        t: Corresponding turtle object.

    Returns:
        None
    """
    match maze_map:
        case 1:
            line(x, y, x, y + size, t)
        case 2:
            line(x, y, x + size, y, t)
        case 3:
            line(x, y, x, y + size, t)
            line(x, y, x + size, y, t)
        case 4:
            line(x + size, y, x + size, y + size, t)
        case 5:
            line(x, y, x, y + size, t)
            line(x + size, y, x + size, y + size, t)
        case 6:
            line(x + size, y, x + size, y + size, t)
            line(x, y, x + size, y, t)
        case 7:
            line(x + size, y, x + size, y + size, t)
            line(x, y, x + size, y, t)
            line(x, y, x, y + size, t)
        case 8:
            line(x, y + size, x + size, y + size, t)
        case 9:
            line(x, y, x, y + size, t)
            line(x, y + size, x + size, y + size, t)
        case 10:
            line(x, y + size, x + size, y + size, t)
            line(x, y, x + size, y, t)
        case 11:
            line(x, y + size, x + size, y + size, t)
            line(x, y, x + size, y, t)
            line(x, y, x, y + size, t)
        case 12:
            line(x, y + size, x + size, y + size, t)
            line(x + size, y, x + size, y + size, t)
        case 13:
            line(x, y + size, x + size, y + size, t)
            line(x + size, y, x + size, y + size, t)
            line(x, y, x, y + size, t)
        case 14:
            line(x, y + size, x + size, y + size, t)
            line(x + size, y, x + size, y + size, t)
            line(x, y, x + size, y, t)
        case 15:
            line(x, y + size, x + size, y + size, t)
            line(x + size, y, x + size, y + size, t)
            line(x, y, x + size, y, t)
            line(x, y, x, y + size, t)


def graph_walls_convert(maze_field: list[int], position: int):
    """Convert edges in node to value which represents walls configuration.
    Made for compatibility with visualization which was made for floodfill
    which doesn't use graph for a maze map.

    Args:
        maze_field: List with connected fields to position.
        position: Variable with maze position.

    Returns:
        int: Variable with value which represents walls configuration.
    """
    if maze_field is None:
        maze_field = []
    cell_value = 15

    if not maze_field:  # not visited
        cell_value = 0
        return cell_value

    for walls in maze_field:
        x = position - walls
        match x:
            case -16:
                cell_value -= Direction.NORTH
            case -1:
                cell_value -= Direction.EAST
            case 1:
                cell_value -= Direction.WEST
            case 16:
                cell_value -= Direction.SOUTH

    return cell_value


def write_cost(x: float, y: float, cost: list[int], t: Turtle):
    """Write cost values in maze cell. Used in A* algorithm.

    Args:
        x: Variable with text x coordinate.
        y: Variable with text y coordinate.
        cost: List with cell costs values.
        t: Corresponding turtle object.

    Returns:
        None
    """
    Gcost = cost[0]
    Hcost = cost[1]
    Fcost = Gcost + Hcost
    t.penup()
    t.goto(x + 16, y + 8)
    t.write("%i" % Fcost, font=("Verdana", 14, "bold"))

    t.goto(x + 6, y + 32)
    t.write("%i" % Gcost, font=("Verdana", 10, "bold"))

    t.goto(x + 36, y + 32)
    t.write("%i" % Hcost, font=("Verdana", 10, "bold"))
