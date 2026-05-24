"""Unit tests for pure controller math functions."""
import math
import pytest

from av_sim.control_math import _yaw_from_quat, _normalise
from av_sim.control_math import (
    compute_cmd, heading_error,
    pure_pursuit_curvature, find_lookahead_point, pure_pursuit_cmd,
    speed_scale_from_scan,
)


# ---------------------------------------------------------------------------
# _normalise
# ---------------------------------------------------------------------------

def test_normalise_zero():
    assert _normalise(0.0) == pytest.approx(0.0)


def test_normalise_positive_small():
    assert _normalise(1.0) == pytest.approx(1.0)


def test_normalise_negative_small():
    assert _normalise(-1.0) == pytest.approx(-1.0)


def test_normalise_just_above_pi():
    result = _normalise(math.pi + 0.1)
    assert result == pytest.approx(-(math.pi - 0.1))


def test_normalise_just_below_neg_pi():
    result = _normalise(-math.pi - 0.1)
    assert result == pytest.approx(math.pi - 0.1)


def test_normalise_two_pi():
    assert _normalise(2 * math.pi) == pytest.approx(0.0, abs=1e-9)


def test_normalise_large_positive():
    result = _normalise(5 * math.pi)
    assert abs(result) <= math.pi


def test_normalise_large_negative():
    result = _normalise(-7 * math.pi)
    assert abs(result) <= math.pi


# ---------------------------------------------------------------------------
# _yaw_from_quat
# ---------------------------------------------------------------------------

class FakeQuat:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


def _quat_from_yaw(yaw):
    return FakeQuat(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


def test_yaw_zero():
    q = _quat_from_yaw(0.0)
    assert _yaw_from_quat(q) == pytest.approx(0.0, abs=1e-9)


def test_yaw_pi_over_2():
    q = _quat_from_yaw(math.pi / 2)
    assert _yaw_from_quat(q) == pytest.approx(math.pi / 2, abs=1e-6)


def test_yaw_pi():
    q = _quat_from_yaw(math.pi)
    result = _yaw_from_quat(q)
    assert abs(result) == pytest.approx(math.pi, abs=1e-6)


def test_yaw_neg_pi_over_4():
    q = _quat_from_yaw(-math.pi / 4)
    assert _yaw_from_quat(q) == pytest.approx(-math.pi / 4, abs=1e-6)


def test_yaw_roundtrip():
    for yaw in [-math.pi + 0.01, -1.0, 0.0, 1.0, math.pi - 0.01]:
        q = _quat_from_yaw(yaw)
        assert _yaw_from_quat(q) == pytest.approx(yaw, abs=1e-6)


# ---------------------------------------------------------------------------
# heading_error
# ---------------------------------------------------------------------------

def test_heading_error_facing_target():
    # Robot at (0,0) facing east (yaw=0), target directly east
    err = heading_error(0.0, 0.0, 0.0, 1.0, 0.0)
    assert err == pytest.approx(0.0, abs=1e-9)


def test_heading_error_target_north():
    # Robot facing east, target directly north → 90° left turn
    err = heading_error(0.0, 0.0, 0.0, 0.0, 1.0)
    assert err == pytest.approx(math.pi / 2, abs=1e-6)


def test_heading_error_target_behind():
    # Robot facing east, target behind → ±180°
    err = heading_error(0.0, 0.0, 0.0, -1.0, 0.0)
    assert abs(err) == pytest.approx(math.pi, abs=1e-6)


def test_heading_error_facing_southwest():
    # Robot at (5,5) facing north (pi/2), target at (4,4) → SW
    err = heading_error(5.0, 5.0, math.pi / 2, 4.0, 4.0)
    # target_yaw = atan2(-1,-1) = -3π/4; err = -3π/4 - π/2 = -5π/4 → normalised +3π/4
    assert err == pytest.approx(3 * math.pi / 4, abs=1e-5) or \
           err == pytest.approx(-5 * math.pi / 4 + 2 * math.pi, abs=1e-5)


# ---------------------------------------------------------------------------
# compute_cmd
# ---------------------------------------------------------------------------

def test_compute_cmd_drives_forward_when_aligned():
    lin, ang = compute_cmd(dist=2.0, heading_err=0.0)
    assert lin > 0
    assert ang == pytest.approx(0.0)


def test_compute_cmd_rotates_in_place_large_error():
    lin, ang = compute_cmd(dist=2.0, heading_err=math.pi / 2)
    assert lin == pytest.approx(0.0)
    assert ang > 0


def test_compute_cmd_rotates_left_for_positive_error():
    lin, ang = compute_cmd(dist=1.0, heading_err=0.5)
    assert ang > 0


def test_compute_cmd_rotates_right_for_negative_error():
    lin, ang = compute_cmd(dist=1.0, heading_err=-0.5)
    assert ang < 0


def test_compute_cmd_clamps_linear():
    lin, ang = compute_cmd(dist=100.0, heading_err=0.0)
    assert lin <= 0.5  # MAX_LINEAR


def test_compute_cmd_clamps_angular():
    lin, ang = compute_cmd(dist=1.0, heading_err=math.pi)
    assert abs(ang) <= 1.2  # MAX_ANGULAR


def test_compute_cmd_zero_dist_stops():
    lin, ang = compute_cmd(dist=0.0, heading_err=0.0)
    assert lin == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# pure_pursuit_curvature
# ---------------------------------------------------------------------------

def test_pure_pursuit_goal_directly_ahead():
    # Robot at origin facing east; goal 2m east → curvature = 0
    kappa = pure_pursuit_curvature(0.0, 0.0, 0.0, 2.0, 0.0)
    assert kappa == pytest.approx(0.0, abs=1e-9)


def test_pure_pursuit_goal_to_left():
    # Robot at origin facing east; goal directly north → turn left (kappa > 0)
    kappa = pure_pursuit_curvature(0.0, 0.0, 0.0, 0.0, 2.0)
    assert kappa > 0


def test_pure_pursuit_goal_to_right():
    # Robot at origin facing east; goal directly south → turn right (kappa < 0)
    kappa = pure_pursuit_curvature(0.0, 0.0, 0.0, 0.0, -2.0)
    assert kappa < 0


def test_pure_pursuit_symmetry():
    # Equal distance left and right should give equal magnitude curvature
    kl = pure_pursuit_curvature(0.0, 0.0, 0.0, 1.0, 1.0)
    kr = pure_pursuit_curvature(0.0, 0.0, 0.0, 1.0, -1.0)
    assert abs(kl) == pytest.approx(abs(kr), rel=1e-6)
    assert kl > 0
    assert kr < 0


def test_pure_pursuit_behind_robot():
    # Goal is directly behind → α = ±π → sin(±π) ≈ 0
    kappa = pure_pursuit_curvature(0.0, 0.0, 0.0, -1.0, 0.0)
    assert abs(kappa) < 1e-6


def test_pure_pursuit_zero_distance():
    # Goal at same position → curvature = 0 (no division by zero)
    kappa = pure_pursuit_curvature(3.0, 4.0, 1.0, 3.0, 4.0)
    assert kappa == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# find_lookahead_point
# ---------------------------------------------------------------------------

def test_find_lookahead_basic():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    pt, idx = find_lookahead_point(path, 0.0, 0.0, 0, 1.5)
    x, y = pt
    assert math.hypot(x, y) >= 1.5


def test_find_lookahead_returns_last_if_path_short():
    path = [(0.0, 0.0), (0.5, 0.0)]  # all points < 2m away
    pt, idx = find_lookahead_point(path, 0.0, 0.0, 0, 2.0)
    assert pt == (0.5, 0.0)
    assert idx == 1


def test_find_lookahead_respects_start_idx():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    # Start at idx=2; lookahead=0.5 → first eligible is (2,0)
    pt, idx = find_lookahead_point(path, 1.8, 0.0, 2, 0.5)
    assert idx >= 2


def test_find_lookahead_exact_distance():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    pt, idx = find_lookahead_point(path, 0.0, 0.0, 0, 1.0)
    # First point at exactly 1.0 distance is (1,0)
    assert pt[0] == pytest.approx(1.0)
    assert pt[1] == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# pure_pursuit_cmd
# ---------------------------------------------------------------------------

def test_pure_pursuit_cmd_forward():
    lin, ang = pure_pursuit_cmd(dist=3.0, curvature=0.0)
    assert lin > 0
    assert ang == pytest.approx(0.0, abs=1e-9)


def test_pure_pursuit_cmd_turn_left():
    lin, ang = pure_pursuit_cmd(dist=2.0, curvature=1.0)
    assert lin > 0
    assert ang > 0


def test_pure_pursuit_cmd_turn_right():
    lin, ang = pure_pursuit_cmd(dist=2.0, curvature=-1.0)
    assert lin > 0
    assert ang < 0


def test_pure_pursuit_cmd_clamps_angular():
    lin, ang = pure_pursuit_cmd(dist=3.0, curvature=100.0)
    assert abs(ang) <= 1.2  # MAX_ANGULAR


def test_pure_pursuit_cmd_zero_dist():
    lin, ang = pure_pursuit_cmd(dist=0.0, curvature=0.5)
    assert lin == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# speed_scale_from_scan
# ---------------------------------------------------------------------------

def test_speed_scale_full_speed_far():
    assert speed_scale_from_scan(min_dist=5.0) == pytest.approx(1.0)


def test_speed_scale_full_speed_at_threshold():
    assert speed_scale_from_scan(min_dist=2.0) == pytest.approx(1.0)


def test_speed_scale_zero_at_stop_dist():
    assert speed_scale_from_scan(min_dist=0.6) == pytest.approx(0.0)


def test_speed_scale_zero_below_stop():
    assert speed_scale_from_scan(min_dist=0.1) == pytest.approx(0.0)


def test_speed_scale_midpoint():
    # midpoint between stop=0.6 and full=2.0 is 1.3 → scale = 0.5
    assert speed_scale_from_scan(min_dist=1.3) == pytest.approx(0.5, abs=1e-6)


def test_speed_scale_monotone():
    dists = [0.0, 0.5, 0.8, 1.2, 1.8, 2.5]
    scales = [speed_scale_from_scan(d) for d in dists]
    for a, b in zip(scales, scales[1:]):
        assert b >= a


def test_speed_scale_clamps_to_unit():
    assert 0.0 <= speed_scale_from_scan(0.0) <= 1.0
    assert 0.0 <= speed_scale_from_scan(100.0) <= 1.0


def test_speed_scale_custom_thresholds():
    # stop=1.0, full=3.0, dist=2.0 → midpoint → 0.5
    assert speed_scale_from_scan(
        2.0, stop_dist=1.0, full_speed_dist=3.0
    ) == pytest.approx(0.5)
