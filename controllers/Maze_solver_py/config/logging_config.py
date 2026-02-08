import yaml
import logging.config
from pathlib import Path


def setup_logging(path: Path = Path("logging.yaml")) -> None:
    """Configure logging based on yaml settings.

    Execute once, at the start of app.

    Args:
        path (Path): Path to the logging configuration file.
    """
    with open(path) as file:
        logging_config: dict = yaml.safe_load(file)

    try:
        import colorlog  # noqa: F401
    except ImportError:
        # Change ColoredFormatter to basic Formatter if colorlog is not installed
        for formatter in logging_config.get("formatters", {}).values():
            if formatter.get("class") == "colorlog.ColoredFormatter":
                formatter["class"] = "logging.Formatter"
                formatter.pop("log_colors", None)

    logging.config.dictConfig(logging_config)
