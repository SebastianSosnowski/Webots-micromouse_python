from pathlib import Path
import pickle
from config.enums import MazeLayout, Algorithms


def read_file(path: Path):
    """Read a pickle file.

    Args:
        path: Path to the file.

    Returns:
        Content of a file.
    """
    with path.open("rb") as file:
        return pickle.load(file)


def write_file(path: Path, values):
    """Write data to a pickle file.

    Args:
        path: Path to the file.
        values: Any type of object with a content to write file.

    Returns:
        None
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("wb") as file:
        pickle.dump(values, file)


def create_files_directories(layout: MazeLayout, algorithm: Algorithms) -> tuple[Path, Path, Path]:
    """Create appropriate directory and file name to save results.

    Args:
        layout: Maze layout enumeration used in simulation.
        algorithm: Algorithm enumeration used in simulation.

    Returns:
        tuple[Path, Path]: Appropriate files directory and names (path_file, map_file, values_file).
    """
    base_dir = Path("Results")
    maze_dir = layout.name.capitalize()
    algo_name = algorithm.name.lower()
    path_dir = base_dir / maze_dir / f"{algo_name}_path.pkl"
    map_dir = base_dir / maze_dir / f"{algo_name}_maze.pkl"
    values_dir = base_dir / maze_dir / f"{algo_name}_algorithm_values.pkl"

    return path_dir, map_dir, values_dir


def save_results(
    path: list[int], maze_map: list | dict, values: list | dict, maze: MazeLayout, alg: Algorithms
):
    path_dir, map_dir, values_dir = create_files_directories(maze, alg)
    write_file(map_dir, maze_map)
    write_file(path_dir, path)
    write_file(values_dir, values)


def read_results(maze: MazeLayout, alg: Algorithms) -> tuple[list[int], list | dict, list | dict]:
    path_dir, map_dir, values_dir = create_files_directories(maze, alg)
    path = read_file(path_dir)
    maze_map = read_file(map_dir)
    values = read_file(values_dir)

    return path, maze_map, values
