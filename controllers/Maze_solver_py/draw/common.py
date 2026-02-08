from typing import cast

from config.enums import Direction, Algorithms
from utils.types import Cost


def walls_from_bitmask(mask: int) -> dict[Direction, bool]:
    return {
        Direction.NORTH: bool(mask & Direction.NORTH),
        Direction.SOUTH: bool(mask & Direction.SOUTH),
        Direction.EAST: bool(mask & Direction.EAST),
        Direction.WEST: bool(mask & Direction.WEST),
    }


def walls_from_graph(pos: int, neighbors: list[int], rows: int, cols: int) -> dict[Direction, bool]:

    row = pos // cols
    col = pos % cols

    def index(r, c):
        return r * cols + c

    walls = {
        Direction.NORTH: True,
        Direction.SOUTH: True,
        Direction.EAST: True,
        Direction.WEST: True,
    }

    if row < rows - 1 and index(row + 1, col) in neighbors:
        walls[Direction.NORTH] = False
    if row > 0 and index(row - 1, col) in neighbors:
        walls[Direction.SOUTH] = False
    if col < cols - 1 and index(row, col + 1) in neighbors:
        walls[Direction.EAST] = False
    if col > 0 and index(row, col - 1) in neighbors:
        walls[Direction.WEST] = False

    return walls


def format_position_values(values, algorithm):
    """
    Convert algorithm-specific position_values into GUI-friendly format.

    Returns:
        dict[int, list[str]]
        e.g. {42: ["12", "7", "5"]}
    """
    result = {}

    if algorithm == Algorithms.FLOODFILL:
        for i, distance in enumerate(values):
            result[i] = [str(distance)]

    elif algorithm == Algorithms.A_STAR or algorithm == Algorithms.A_STAR_MOD:
        values = cast(dict[int, Cost], values)
        for i, cost in values.items():
            result[i] = [str(cost.g + cost.h), str(cost.g), str(cost.h)]

    return result
