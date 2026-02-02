from enum import EnumType
import logging
from pathlib import Path
import yaml

from config.enums import Algorithms, MazeLayout, Mode
from config.models import AppConfig

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)


def load_config(path: Path) -> AppConfig:
    """Load configuration from a YAML file and validate simulation parameters.

    Args:
        path (Path): Path to the configuration file.

    Returns:
        dict: Loaded and validated configuration dictionary.
    """
    with open(path) as file:
        raw_cfg: dict = yaml.safe_load(file)

    config = AppConfig.model_validate(raw_cfg)
    return config


def validate_simulation_parameter(value: str, enum_cls: EnumType) -> EnumType:
    """Validate and convert a string value to an enum member.

    Args:
        value (str): String value to validate.
        enum_cls (EnumType): Enum class to validate against.

    Returns:
        EnumType: Validated enum member.
    """
    assert (
        value in enum_cls.__members__
    ), f"Setting '{value}' is not a valid member of {enum_cls.__name__}: {enum_cls._member_names_}"
    logger.debug("type: %s, value: %s", type(enum_cls[value]), enum_cls[value])
    return enum_cls[value]
