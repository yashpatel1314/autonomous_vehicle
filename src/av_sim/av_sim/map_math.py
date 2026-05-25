"""Pure CSV loading and grid/world coordinate math — no ROS dependencies."""
import csv


def load_obstacles(path: str) -> list:
    with open(path, newline='') as f:
        return [(int(r['grid_x']), int(r['grid_y'])) for r in csv.DictReader(f)]


def load_checkpoints(path: str) -> list:
    with open(path, newline='') as f:
        rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
    return [(int(r['grid_x']), int(r['grid_y'])) for r in rows]


def cell_to_world(gx: int, gy: int, cell_size: float = 1.0) -> tuple:
    return (gx + 0.5) * cell_size, (gy + 0.5) * cell_size


def world_to_cell(wx: float, wy: float, cell_size: float = 1.0) -> tuple:
    return int(wx / cell_size), int(wy / cell_size)


def in_bounds(gx: int, gy: int, width: int, height: int) -> bool:
    return 0 <= gx < width and 0 <= gy < height
