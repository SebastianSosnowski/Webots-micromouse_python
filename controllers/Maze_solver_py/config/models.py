"""Pydantic models used for config validation."""

from pydantic import BaseModel

from config.enums import Algorithms, MazeLayout, Mode


class MazeConfig(BaseModel):
    rows: int
    columns: int
    start_position: int
    target_position: int
    tile_length: float
    visited_flag: int


class RobotConfig(BaseModel):
    model: str
    axle: float
    wheel: float
    speed: float


class SimulationConfig(BaseModel):
    mode: Mode
    algorithm: Algorithms
    maze_layout: MazeLayout
    time_step: int


class AppConfig(BaseModel):
    simulation: SimulationConfig
    robot: RobotConfig
    maze: MazeConfig
