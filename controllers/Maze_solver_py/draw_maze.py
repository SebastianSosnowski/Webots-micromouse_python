from turtle import * 
import var
from Constants import *


''' init_maze
# @brief Init maze window with grid and starting walls
#
# @param maze_map: list with initial maze map with walls
# @param distance: list with initial distances values/path
# @param size: variable with side size of one maze cell
#
# @retval text: object with maze text/numbers
# @retval maze: object with maze walls
'''
def init_maze(maze_map, distance, size):
    setup(1020, 1020, 1500, 160) #size and position on screen
    maze = Turtle()
    maze.hideturtle()
    tracer(False)
    maze.color('black')
    maze.width(1)
    
    grid = Turtle()
    grid.hideturtle()
    grid.color('black')
    grid.width(1)

    text = Turtle()
    text.hideturtle()
    text.width(1)

    #draw grid
    for y in range(-480, 480, size):
        for x in range(-480, 480, size):
            line(x, y + size, x + size, y + size, grid)
            line(x + size, y, x + size, y + size, grid)
            line(x, y, x + size, y, grid)
            line(x, y, x, y + size, grid)

    maze.width(5)
    #draw walls
    i = 0
    if mode_params.ALGORITHM == algorithms.FLOODFILL:
        for y in range(-480, 480, size):
            for x in range(-480, 480, size):
                write_distance(x, y, distance[i], text) # write initial distance values
                if maze_map[i] < 64: #unvisited cell
                    draw_wall(maze_map[i], x, y, size, maze)
                else:
                    draw_wall(maze_map[i] - 64, x, y, size, maze)
                i += 1
    else: #graph algorithms
        for y in range(-480, 480, size):
            for x in range(-480, 480, size):
                cell = graph_walls_convert(maze_map[i], i)
                draw_wall(cell, x, y, size, maze)
                i += 1

    return text, maze


''' draw_maze
# @brief Draw maze with distance values and discovered walls.
# For search run also mark visited cells.
# For speedrun draws robot path and actual position.
#
# @param maze_map: list with actual maze map with walls
# @param distance: list with actual distances values/path
#
# @retval None
'''
def draw_maze(maze_map, distance):
    
    size = 60

    text, maze = init_maze(maze_map, distance, size)

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
    
    if mode_params.MODE == mode_params.SEARCH:
        update_maze_search(size, circles, text, maze)

    update_maze_speedrun(size, lines, circles)

    done()


''' update_maze_speedrun
# @brief Update maze visualization with actual robot position and path.
#
# @param size: size: variable with side size of one maze cell
# @param lines: object with lines that draw robot path
# @param robot_position: object with circle that indicates actual robot position
#
# @retval None
'''
def update_maze_speedrun(size, lines, robot_position):
    last_x, last_y = draw_path(0, 0, size, lines)
    draw_position(size, robot_position)

    while var.robot_pos != maze_parameters.TARGET_CELL:

        var.drawing_event.wait()
        var.drawing_event.clear()

        last_x, last_y = draw_path(last_x, last_y, size, lines)
        robot_position.clear()
        draw_position(size, robot_position)
        
        update()
        var.main_event.set()


''' update_maze_search
# @brief Update maze visualization with visited cells and discovered walls and distance values.
# For floodfill distance values are also drawn.
# For A* costs values are also drawn.
#
# @param size: variable with side size of one maze cell
# @param visited_cell: object with circles that indicates cells visited by robot
# @param text: object with maze text/numbers
# @param maze: object with maze walls
#
# @retval None
'''
def update_maze_search(size, visited_cell, text, maze):
    
    draw_position(size, visited_cell)

    while var.robot_pos != var.target_global:

        var.drawing_event.wait()
        var.drawing_event.clear()
            

        draw_position(size, visited_cell)

        xx = var.robot_pos % 16
        xx = -480 + xx * size 
        yy = var.robot_pos // 16
        yy = -480 + yy * size

        if mode_params.ALGORITHM == algorithms.FLOODFILL:
            draw_wall(var.maze_map_global[var.robot_pos] - 64, xx, yy, size, maze)
            if var.distance_update:
                i = 0
                text.clear()
                for y in range(-480, 480, size):
                    for x in range(-480, 480, size):
                        write_distance(x, y, var.distance_global[i], text)
                        i += 1
                var.distance_update = False
        else: #graphs
            cell = graph_walls_convert(var.maze_map_global[var.robot_pos], var.robot_pos)
            draw_wall(cell, xx, yy, size, maze)
            
            if mode_params.ALGORITHM == algorithms.A_STAR or mode_params.ALGORITHM == algorithms.A_STAR_MOD:
                text.clear()
                for key in var.cost_global:
                    x = key % 16
                    x = -480 + x * size 
                    y = key // 16
                    y = -480 + y * size
                    write_cost(x, y, var.cost_global[key], text)
        
        if var.robot_pos == 136:
            draw_center(size, maze)

        update()
        var.main_event.set()


''' draw_center
# @brief Update maze visualization with center cells at the end according to visited cells.
#
# @param size: variable with side size of one maze cell
# @param maze: object with maze walls
#
# @retval None
'''
def draw_center(size, maze):
        center = [119, 120, 135]
        for center_cell in center:
            if mode_params.ALGORITHM == algorithms.FLOODFILL:
                check = (var.maze_map_global[center_cell] & maze_parameters.VISITED) != maze_parameters.VISITED
            else: #graphs algorithms
                check = len(var.maze_map_global[center_cell]) == 0 #inside unvisited nodes have 4 edges
            if check:
                x = center_cell % 16
                x = -480 + x * size 
                y = center_cell // 16
                y = -480 + y * size
                match center_cell:
                    case 119:
                        draw_wall(3, x, y, size, maze)
                    case 120:
                        draw_wall(6, x, y, size, maze)
                    case 135:
                        draw_wall(9, x, y, size, maze)


''' line
# @brief Draw line.
#
# @param start_x: variable with line beginning x coordinate
# @param start_y: variable with line beginning y coordinate
# @param end_x: variable with line ending x coordinate
# @param end_y: variable with line ending y coordinate
# @param t: corresponding turtle object
#
# @retval None
'''
def line(start_x, start_y, end_x, end_y, t):
    t.up()
    t.goto(start_x, start_y)
    t.down()
    t.goto(end_x, end_y) 


''' draw_path
# @brief Draw robot path.
#
# @param last_x: variable with last robot x coordinate position
# @param last_y: variable with last robot y coordinate position
# @param size: variable with side size of one maze cell
# @param t: corresponding turtle object
#
# @retval next_x: variable with actual robot x coordinate position
# @retval next_y: variable with actual robot y coordinate position
'''
def draw_path(last_x, last_y, size, t):
    next_x = var.robot_pos % 16
    next_y = int(var.robot_pos / 16)
    t.penup()
    t.goto(-450 + last_x * size, -450 + last_y * size) #last position
    t.pendown()
    line(-450 + last_x * size, -450 + last_y * size, -450 + next_x * size, -450 + next_y * size, t)

    return next_x, next_y


''' draw_position
# @brief Draw mark in maze cell where robot is right now.
#
# @param size: variable with side size of one maze cell
# @param t: corresponding turtle object
#
# @retval None
'''
def draw_position(size, t):
    x = var.robot_pos % maze_parameters.COLUMNS
    y = int(var.robot_pos / maze_parameters.ROWS)
    t.penup()
    t.goto(-450 + x * size, -450 + y * size - 6) #last position
    t.pendown()
    t.fillcolor("red")
    t.begin_fill()
    t.circle(6)
    t.end_fill()


''' write_distance
# @brief Write distance value in maze cell.
#
# @param x: variable with text x coordinate
# @param y: variable with text y coordinate
# @param distance: variable with distance value to target
# @param t: corresponding turtle object
#
# @retval None
'''
def write_distance(x, y, distance, t):
    t.penup()
    t.goto(x + 8, y + 16)
    t.write('%i' % distance, font=("Verdana", 13, 'bold'))


''' write_cost
# @brief Write cost values in maze cell. Used in A* algorithm
#
# @param x: variable with text x coordinate
# @param y: variable with text y coordinate
# @param cost: list with cell costs values 
# @param t: corresponding turtle object
#
# @retval None
'''
def write_cost(x, y, cost, t):
    Gcost = cost[0]
    Hcost = cost[1]
    Fcost = Gcost + Hcost
    t.penup()
    t.goto(x + 16, y + 8)
    t.write('%i' % Fcost, font=("Verdana", 14, 'bold'))
    
    t.goto(x + 6, y + 32)
    t.write('%i' % Gcost, font=("Verdana", 10, 'bold'))
    
    t.goto(x + 36, y + 32)
    t.write('%i' % Hcost, font=("Verdana", 10, 'bold'))


''' draw_wall
# @brief Draw corresponding walls and values in each field.
#
# @param maze_map: list with actual maze map with walls
# @param x: variable with offset in x direction
# @param y: variable with offset in y direction
# @param size: value with one field size (for easier change when changing window size)
# @param t: corresponding turtle object
#
# @retval None
'''
def draw_wall(maze_map, x, y, size, t):

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


''' graph_walls_convert 
# @brief Convert edges in node to value which represents walls configuration.
# Made for compatibility with visualization which was made for floodfill
# which doesn't use graph for a maze map.
#
# @param maze_field: list with connected fields to position
# @param position: variable with maze position
#
# @retval cell_value: variable with value which represents walls configuration.
'''
def graph_walls_convert(maze_field, position): #list, value
    cell_value = 15

    if not maze_field:# not visited
        cell_value = 0
        return cell_value
    
    for walls in maze_field:
        x = position - walls
        match x:
            case -16:
                cell_value -= direction.NORTH
            case -1:
                cell_value -= direction.EAST
            case 1:
                cell_value -= direction.WEST
            case 16:
                cell_value -= direction.SOUTH

    return cell_value