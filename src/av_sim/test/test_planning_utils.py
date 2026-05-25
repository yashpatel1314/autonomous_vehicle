"""Unit tests for inflate_obstacles, prune_path (av_sim.planning)."""
import pytest
from av_sim.astar_math import inflate_obstacles, prune_path


# ---------------------------------------------------------------------------
# inflate_obstacles
# ---------------------------------------------------------------------------

def test_inflate_empty():
    assert inflate_obstacles(set(), 20, 20, radius=1) == set()


def test_inflate_single_radius_1():
    result = inflate_obstacles({(5, 5)}, 20, 20, radius=1)
    # 3×3 block around (5,5)
    expected = {(x, y) for x in range(4, 7) for y in range(4, 7)}
    assert result == expected


def test_inflate_single_radius_0():
    result = inflate_obstacles({(5, 5)}, 20, 20, radius=0)
    assert result == {(5, 5)}


def test_inflate_clips_at_grid_corner():
    result = inflate_obstacles({(0, 0)}, 20, 20, radius=1)
    for gx, gy in result:
        assert 0 <= gx < 20
        assert 0 <= gy < 20
    # only quadrant that fits
    assert (0, 0) in result
    assert (1, 0) in result
    assert (0, 1) in result
    assert (1, 1) in result
    # out-of-bounds cells must not appear
    assert (-1, 0) not in result
    assert (0, -1) not in result


def test_inflate_clips_at_far_corner():
    result = inflate_obstacles({(19, 19)}, 20, 20, radius=1)
    for gx, gy in result:
        assert 0 <= gx < 20
        assert 0 <= gy < 20


def test_inflate_two_adjacent_cells_merge():
    result = inflate_obstacles({(5, 5), (5, 6)}, 20, 20, radius=1)
    # Both cells inflate and their regions overlap
    assert len(result) < 18  # less than 2 separate 3×3 blocks (18 cells)
    assert (5, 5) in result
    assert (5, 6) in result


def test_inflate_radius_2():
    result = inflate_obstacles({(10, 10)}, 20, 20, radius=2)
    expected = {(x, y) for x in range(8, 13) for y in range(8, 13)}
    assert result == expected


def test_inflate_does_not_mutate_input():
    original = {(3, 3)}
    inflate_obstacles(original, 20, 20, radius=1)
    assert original == {(3, 3)}


# ---------------------------------------------------------------------------
# prune_path
# ---------------------------------------------------------------------------

def test_prune_empty():
    assert prune_path([]) == []


def test_prune_single():
    assert prune_path([(2, 2)]) == [(2, 2)]


def test_prune_two_points():
    assert prune_path([(0, 0), (5, 0)]) == [(0, 0), (5, 0)]


def test_prune_three_collinear_horizontal():
    # (0,0)→(1,0)→(2,0): middle point is collinear, should be removed
    result = prune_path([(0, 0), (1, 0), (2, 0)])
    assert result == [(0, 0), (2, 0)]


def test_prune_three_collinear_diagonal():
    result = prune_path([(0, 0), (1, 1), (2, 2)])
    assert result == [(0, 0), (2, 2)]


def test_prune_keeps_corner():
    # 90° turn: (0,0)→(2,0)→(2,2) — middle must be kept
    result = prune_path([(0, 0), (2, 0), (2, 2)])
    assert result == [(0, 0), (2, 0), (2, 2)]


def test_prune_long_collinear_run():
    path = [(0, y) for y in range(10)]
    result = prune_path(path)
    assert result == [(0, 0), (0, 9)]


def test_prune_mixed():
    # straight then turn then straight
    path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (2, 3)]
    result = prune_path(path)
    # (1,0) is collinear → removed; (2,1),(2,2) are collinear → removed
    assert (1, 0) not in result
    assert (2, 1) not in result
    assert (2, 0) in result   # corner kept
    assert result[0] == (0, 0)
    assert result[-1] == (2, 3)


def test_prune_preserves_start_and_end():
    path = [(3, 7), (4, 7), (5, 7), (6, 8)]
    result = prune_path(path)
    assert result[0] == (3, 7)
    assert result[-1] == (6, 8)
