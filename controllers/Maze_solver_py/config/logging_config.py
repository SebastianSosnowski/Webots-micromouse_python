import yaml
import logging.config
from pathlib import Path
import colorlog

def setup_logging(path: Path = Path("logging.yaml")) -> None:
    """Configure logging based on yaml settings.

    Execute once, at the start of app.

    Args:
        path (Path): Path to the logging configuration file.
    """
    with open(path) as file:
        logging_config: dict = yaml.safe_load(file)
    logging.config.dictConfig(logging_config)
    colorlog.ColoredFormatter
