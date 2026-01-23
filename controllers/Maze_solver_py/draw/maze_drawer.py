from turtle import setup, Turtle, tracer, update, done
import var

from config.enums import Direction, Algorithm, Mode
from config.world import world

from utils.params import DrawState


class MazeDrawer:
    def __init__(self, maze_map, distance) -> None:
        self.size = 60
        self.text_canvas, self.maze_canvas = self._init_maze(maze_map, distance)
        self.robot_pos_canvas, self.path_canvas = self._init_canvas()
        self.last_x = 0
        self.last_y = 0

    def _init_canvas(self):
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

    @staticmethod
    def thread_entry(maze_map, distance):
        drawer = MazeDrawer(maze_map, distance)
        drawer.run()

    def run(self):
        while var.robot_pos != world.maze.target_cell:
            var.drawing_event.wait()
            var.drawing_event.clear()

            if world.sim.mode == Mode.SEARCH:
                self._draw_search_frame()
            else:

                self._draw_speedrun_frame()

            var.main_event.set()
        done()

    def _init_maze(self, maze_map, distance):
        """
        @brief Init maze window with grid and starting walls

        @param maze_map: list with initial maze map with walls
        @param distance: list with initial distances values/path
        @param size: variable with side size of one maze cell

        @retval text: object with maze text/numbers
        @retval maze: object with maze walls
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
        if world.sim.algorithm == Algorithm.FLOODFILL:
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

        return text, maze

    def _draw_search_frame(self):
        """
        @brief Update maze visualization with visited cells and discovered walls and distance values.
        For floodfill distance values are also drawn.
        For A* costs values are also drawn.

        @param size: variable with side size of one maze cell
        @param visited_cell: object with circles that indicates cells visited by robot
        @param text: object with maze text/numbers
        @param maze: object with maze walls

        @retval None
        """
        self._draw_position(self.robot_pos_canvas)

        xx = var.robot_pos % 16
        xx = -480 + xx * self.size
        yy = var.robot_pos // 16
        yy = -480 + yy * self.size

        if world.sim.algorithm == Algorithm.FLOODFILL:
            draw_wall(var.maze_map_global[var.robot_pos] - 64, xx, yy, self.size, self.maze_canvas)
            if var.distance_update:
                i = 0
                self.text_canvas.clear()
                for y in range(-480, 480, self.size):
                    for x in range(-480, 480, self.size):
                        write_distance(x, y, var.distance_global[i], self.text_canvas)
                        i += 1
                var.distance_update = False
        else:  # graphs
            cell = graph_walls_convert(var.maze_map_global[var.robot_pos], var.robot_pos)
            draw_wall(cell, xx, yy, self.size, self.maze_canvas)

            if (
                world.sim.algorithm == Algorithm.A_STAR
                or world.sim.algorithm == Algorithm.A_STAR_MOD
            ):
                self.text_canvas.clear()
                for key in var.cost_global:
                    x = key % 16
                    x = -480 + x * self.size
                    y = key // 16
                    y = -480 + y * self.size
                    write_cost(x, y, var.cost_global[key], self.text_canvas)

        if var.robot_pos == 136:
            self._draw_center()

        update()

    def _draw_speedrun_frame(self):
        """
        @brief Update maze visualization with actual robot position and path.

        @param size: size: variable with side size of one maze cell
        @param lines: object with lines that draw robot path
        @param robot_position: object with circle that indicates actual robot position

        @retval None
        """
        self._draw_path(self.path_canvas)
        self._draw_position(self.robot_pos_canvas)

        self._draw_path(self.path_canvas)
        self.robot_pos_canvas.clear()
        self._draw_position(self.robot_pos_canvas)

        update()

    def _draw_center(self):
        """
        @brief Update maze visualization with center cells at the end according to visited cells.

        @param size: variable with side size of one maze cell
        @param maze: object with maze walls

        @retval None
        """
        center = [119, 120, 135]
        for center_cell in center:
            if world.sim.algorithm == Algorithm.FLOODFILL:
                check = (
                    var.maze_map_global[center_cell] & world.maze.visited
                ) != world.maze.visited
            else:  # graphs Algorithm
                check = (
                    len(var.maze_map_global[center_cell]) == 0
                )  # inside unvisited nodes have 4 edges
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
        """
        @brief Draw mark in maze cell where robot is right now.

        @param size: variable with side size of one maze cell
        @param t: corresponding turtle object

        @retval None
        """
        x = var.robot_pos % world.maze.columns
        y = int(var.robot_pos / world.maze.rows)
        t.penup()
        t.goto(-450 + x * self.size, -450 + y * self.size - 6)  # last position
        t.pendown()
        t.fillcolor("red")
        t.begin_fill()
        t.circle(6)
        t.end_fill()

    def _draw_path(self, t: Turtle):
        """draw_path
        @brief Draw robot path.

        @param last_x: variable with last robot x coordinate position
        @param last_y: variable with last robot y coordinate position
        @param size: variable with side size of one maze cell
        @param t: corresponding turtle object

        @retval next_x: variable with actual robot x coordinate position
        @retval next_y: variable with actual robot y coordinate position
        """
        next_x = var.robot_pos % 16
        next_y = int(var.robot_pos / 16)
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


def line(start_x, start_y, end_x, end_y, t: Turtle):
    """
    @brief Draw line.

    @param start_x: variable with line beginning x coordinate
    @param start_y: variable with line beginning y coordinate
    @param end_x: variable with line ending x coordinate
    @param end_y: variable with line ending y coordinate
    @param t: corresponding turtle object

    @retval None
    """
    t.up()
    t.goto(start_x, start_y)
    t.down()
    t.goto(end_x, end_y)


def write_distance(x, y, distance, t: Turtle):
    """
    @brief Write distance value in maze cell.

    @param x: variable with text x coordinate
    @param y: variable with text y coordinate
    @param distance: variable with distance value to target
    @param t: corresponding turtle object

    @retval None
    """
    t.penup()
    t.goto(x + 8, y + 16)
    t.write("%i" % distance, font=("Verdana", 13, "bold"))


def draw_wall(maze_map, x, y, size, t: Turtle):
    """
    @brief Draw corresponding walls and values in each field.

    @param maze_map: list with actual maze map with walls
    @param x: variable with offset in x direction
    @param y: variable with offset in y direction
    @param size: value with one field size (for easier change when changing window size)
    @param t: corresponding turtle object

    @retval None
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


def graph_walls_convert(maze_field, position):  # list, value
    """
    @brief Convert edges in node to value which represents walls configuration.
    Made for compatibility with visualization which was made for floodfill
    which doesn't use graph for a maze map.

    @param maze_field: list with connected fields to position
    @param position: variable with maze position

    @retval cell_value: variable with value which represents walls configuration.
    """
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


def write_cost(x, y, cost, t: Turtle):
    """write_cost
    @brief Write cost values in maze cell. Used in A* algorithm

    @param x: variable with text x coordinate
    @param y: variable with text y coordinate
    @param cost: list with cell costs values
    @param t: corresponding turtle object

    @retval None
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
