"""Unit tests for map_math pure functions."""
import os
import tempfile
import pytest
from av_sim.map_math import (
    load_obstacles, load_checkpoints,
    cell_to_world, world_to_cell, in_bounds,
)


def _write_csv(content: str) -> str:
    f = tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False)
    f.write(content)
    f.close()
    return f.name


# ── load_obstacles ────────────────────────────────────────────────────────────

def test_load_obstacles_basic():
    path = _write_csv("grid_x,grid_y\n1,2\n3,4\n")
    try:
        result = load_obstacles(path)
    finally:
        os.unlink(path)
    assert result == [(1, 2), (3, 4)]


def test_load_obstacles_empty():
    path = _write_csv("grid_x,grid_y\n")
    try:
        result = load_obstacles(path)
    finally:
        os.unlink(path)
    assert result == []


def test_load_obstacles_single():
    path = _write_csv("grid_x,grid_y\n7,3\n")
    try:
        result = load_obstacles(path)
    finally:
        os.unlink(path)
    assert result == [(7, 3)]


# ── load_checkpoints ──────────────────────────────────────────────────────────

def test_load_checkpoints_sorted_by_order():
    path = _write_csv("order,grid_x,grid_y\n2,5,6\n1,3,4\n3,7,8\n")
    try:
        result = load_checkpoints(path)
    finally:
        os.unlink(path)
    assert result == [(3, 4), (5, 6), (7, 8)]


def test_load_checkpoints_single():
    path = _write_csv("order,grid_x,grid_y\n1,10,20\n")
    try:
        result = load_checkpoints(path)
    finally:
        os.unlink(path)
    assert result == [(10, 20)]


# ── cell_to_world ─────────────────────────────────────────────────────────────

def test_cell_to_world_origin():
    wx, wy = cell_to_world(0, 0)
    assert wx == pytest.approx(0.5)
    assert wy == pytest.approx(0.5)


def test_cell_to_world_nonzero():
    wx, wy = cell_to_world(2, 3)
    assert wx == pytest.approx(2.5)
    assert wy == pytest.approx(3.5)


def test_cell_to_world_custom_cell_size():
    wx, wy = cell_to_world(1, 1, cell_size=2.0)
    assert wx == pytest.approx(3.0)
    assert wy == pytest.approx(3.0)


# ── world_to_cell ─────────────────────────────────────────────────────────────

def test_world_to_cell_origin():
    assert world_to_cell(0.5, 0.5) == (0, 0)


def test_world_to_cell_nonzero():
    assert world_to_cell(2.7, 3.1) == (2, 3)


def test_world_to_cell_boundary():
    assert world_to_cell(1.0, 1.0) == (1, 1)


def test_world_to_cell_roundtrip():
    for gx, gy in [(0, 0), (5, 3), (14, 29)]:
        wx, wy = cell_to_world(gx, gy)
        assert world_to_cell(wx, wy) == (gx, gy)


# ── in_bounds ─────────────────────────────────────────────────────────────────

def test_in_bounds_centre():
    assert in_bounds(5, 5, 10, 10)


def test_in_bounds_origin():
    assert in_bounds(0, 0, 10, 10)


def test_in_bounds_at_limit_false():
    assert not in_bounds(10, 5, 10, 10)
    assert not in_bounds(5, 10, 10, 10)


def test_in_bounds_negative_false():
    assert not in_bounds(-1, 0, 10, 10)
    assert not in_bounds(0, -1, 10, 10)
