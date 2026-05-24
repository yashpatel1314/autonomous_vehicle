"""Unit tests for scan_hits_to_grid (av_sim.scan_occupancy)."""
import math
import pytest
from av_sim.scan_occupancy import scan_hits_to_grid


# ---------------------------------------------------------------------------
# helpers
# ---------------------------------------------------------------------------

def _make_ranges(n, value):
    return [value] * n


# ---------------------------------------------------------------------------
# basic conversion
# ---------------------------------------------------------------------------

def test_single_hit_directly_ahead():
    # Robot at (0.5, 0.5) facing east (yaw=0), single ray at angle=0, range=1m
    # laser at world (0.5, 0.5), hit at (1.5, 0.5) → grid (1, 0)
    cells = scan_hits_to_grid(
        ranges=[1.0],
        angle_min=0.0,
        angle_step=0.0,
        robot_yaw=0.0,
        laser_x=0.5,
        laser_y=0.5,
        cell_size=1.0,
        grid_w=20,
        grid_h=20,
        range_max=12.0,
    )
    assert (1, 0) in cells


def test_single_hit_north():
    # laser at (5.5, 5.5), yaw=π/2, ray at angle=0 (absolute π/2) → hit north
    cells = scan_hits_to_grid(
        ranges=[2.0],
        angle_min=0.0,
        angle_step=0.0,
        robot_yaw=math.pi / 2,
        laser_x=5.5,
        laser_y=5.5,
        cell_size=1.0,
        grid_w=20,
        grid_h=20,
        range_max=12.0,
    )
    # world hit at (5.5, 7.5) → grid (5, 7)
    assert (5, 7) in cells


def test_360_scan_all_max_range_filtered():
    n = 360
    ranges = _make_ranges(n, 12.0)  # exactly at range_max → filtered out
    angle_step = 2 * math.pi / n
    cells = scan_hits_to_grid(
        ranges=ranges,
        angle_min=0.0,
        angle_step=angle_step,
        robot_yaw=0.0,
        laser_x=10.0,
        laser_y=10.0,
        cell_size=1.0,
        grid_w=20,
        grid_h=20,
        range_max=12.0,
    )
    assert len(cells) == 0


def test_zero_range_filtered():
    cells = scan_hits_to_grid(
        ranges=[0.0],
        angle_min=0.0,
        angle_step=0.0,
        robot_yaw=0.0,
        laser_x=5.0,
        laser_y=5.0,
        cell_size=1.0,
        grid_w=20,
        grid_h=20,
        range_max=12.0,
    )
    assert len(cells) == 0


def test_out_of_bounds_hit_filtered():
    # laser at (0.5, 0.5), ray pointing west at range=2 → world (-1.5, 0.5) → out of grid
    cells = scan_hits_to_grid(
        ranges=[2.0],
        angle_min=math.pi,  # pointing west
        angle_step=0.0,
        robot_yaw=0.0,
        laser_x=0.5,
        laser_y=0.5,
        cell_size=1.0,
        grid_w=20,
        grid_h=20,
        range_max=12.0,
    )
    assert len(cells) == 0


def test_multiple_rays_produce_multiple_cells():
    # Two rays, slightly different angles, hitting different cells
    cells = scan_hits_to_grid(
        ranges=[1.5, 1.5],
        angle_min=0.0,
        angle_step=math.pi / 2,  # 0° and 90°
        robot_yaw=0.0,
        laser_x=5.5,
        laser_y=5.5,
        cell_size=1.0,
        grid_w=20,
        grid_h=20,
        range_max=12.0,
    )
    # ray 0: east → (7, 5); ray 1: north → (5, 7)
    assert (7, 5) in cells
    assert (5, 7) in cells


def test_all_cells_within_grid():
    n = 360
    ranges = _make_ranges(n, 3.0)
    angle_step = 2 * math.pi / n
    cells = scan_hits_to_grid(
        ranges=ranges,
        angle_min=0.0,
        angle_step=angle_step,
        robot_yaw=0.0,
        laser_x=10.0,
        laser_y=10.0,
        cell_size=1.0,
        grid_w=20,
        grid_h=20,
        range_max=12.0,
    )
    for gx, gy in cells:
        assert 0 <= gx < 20
        assert 0 <= gy < 20
