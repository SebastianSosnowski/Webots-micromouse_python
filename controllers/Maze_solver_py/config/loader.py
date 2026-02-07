import yaml
import logging.config
from pathlib import Path

from config.models import AppConfig


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
