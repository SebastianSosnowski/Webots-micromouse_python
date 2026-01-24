import yaml
from pathlib import Path
from enum import EnumType
import logging

from config.enums import Mode, Algorithm, MazeLayout

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def load_config(path: Path) -> dict:
    with open(path) as file:
        config: dict = yaml.safe_load(file)
    sim_cfg: dict = config.get("simulation", {})
    sim_cfg["mode"] = validate_simulation_parameter(sim_cfg.get("mode", ""), Mode)
    sim_cfg["algorithm"] = validate_simulation_parameter(sim_cfg.get("algorithm", ""), Algorithm)
    sim_cfg["maze_layout"] = validate_simulation_parameter(
        sim_cfg.get("maze_layout", ""), MazeLayout
    )
    config["simulation"] = sim_cfg
    return config


def validate_simulation_parameter(value: str, enum_cls: EnumType) -> EnumType:
    assert (
        value in enum_cls.__members__
    ), f"Setting '{value}' is not a valid member of {enum_cls.__name__}: {enum_cls._member_names_}"
    logger.debug("type: %s, value: %s", type(enum_cls[value]), enum_cls[value])
    return enum_cls[value]
