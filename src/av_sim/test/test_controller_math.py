"""Unit tests for pure controller math functions."""
import math
import pytest

from av_sim.control_math import (
    _normalise, _yaw_from_quat,
    heading_error, compute_cmd,
    pure_pursuit_curvature, find_lookahead_point, pure_pursuit_cmd,
)


# ── _normalise ────────────────────────────────────────────────────────────────

def test_normalise_zero():
    assert _normalise(0.0) == pytest.approx(0.0)

def test_normalise_positive_small():
    assert _normalise(1.0) == pytest.approx(1.0)

def test_normalise_negative_small():
    assert _normalise(-1.0) == pytest.approx(-1.0)

def test_normalise_just_above_pi():
    assert _normalise(math.pi + 0.1) == pytest.approx(-(math.pi - 0.1))

def test_normalise_just_below_neg_pi():
    assert _normalise(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)

def test_normalise_two_pi():
    assert _normalise(2 * math.pi) == pytest.approx(0.0, abs=1e-9)

def test_normalise_large_positive():
    assert abs(_normalise(5 * math.pi)) <= math.pi

def test_normalise_large_negative():
    assert abs(_normalise(-7 * math.pi)) <= math.pi


# ── _yaw_from_quat ────────────────────────────────────────────────────────────

class FakeQuat:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w

def _quat_from_yaw(yaw):
    return FakeQuat(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))

def test_yaw_zero():
    assert _yaw_from_quat(_quat_from_yaw(0.0)) == pytest.approx(0.0, abs=1e-9)

def test_yaw_pi_over_2():
    assert _yaw_from_quat(_quat_from_yaw(math.pi / 2)) == pytest.approx(math.pi / 2, abs=1e-6)

def test_yaw_pi():
    assert abs(_yaw_from_quat(_quat_from_yaw(math.pi))) == pytest.approx(math.pi, abs=1e-6)

def test_yaw_neg_pi_over_4():
    assert _yaw_from_quat(_quat_from_yaw(-math.pi / 4)) == pytest.approx(-math.pi / 4, abs=1e-6)

def test_yaw_roundtrip():
    for yaw in [-math.pi + 0.01, -1.0, 0.0, 1.0, math.pi - 0.01]:
        assert _yaw_from_quat(_quat_from_yaw(yaw)) == pytest.approx(yaw, abs=1e-6)


# ── heading_error ─────────────────────────────────────────────────────────────

def test_heading_error_facing_target():
    assert heading_error(0.0, 0.0, 0.0, 1.0, 0.0) == pytest.approx(0.0, abs=1e-9)

def test_heading_error_target_north():
    assert heading_error(0.0, 0.0, 0.0, 0.0, 1.0) == pytest.approx(math.pi / 2, abs=1e-6)

def test_heading_error_target_behind():
    assert abs(heading_error(0.0, 0.0, 0.0, -1.0, 0.0)) == pytest.approx(math.pi, abs=1e-6)

def test_heading_error_facing_southwest():
    err = heading_error(5.0, 5.0, math.pi / 2, 4.0, 4.0)
    assert err == pytest.approx(3 * math.pi / 4, abs=1e-5) or \
           err == pytest.approx(-5 * math.pi / 4 + 2 * math.pi, abs=1e-5)


# ── compute_cmd ───────────────────────────────────────────────────────────────

def test_compute_cmd_drives_forward_when_aligned():
    lin, ang = compute_cmd(dist=2.0, heading_err=0.0)
    assert lin > 0
    assert ang == pytest.approx(0.0)

def test_compute_cmd_rotates_in_place_large_error():
    lin, ang = compute_cmd(dist=2.0, heading_err=math.pi / 2)
    assert lin == pytest.approx(0.0)
    assert ang > 0

def test_compute_cmd_rotates_left_for_positive_error():
    _, ang = compute_cmd(dist=1.0, heading_err=0.5)
    assert ang > 0

def test_compute_cmd_rotates_right_for_negative_error():
    _, ang = compute_cmd(dist=1.0, heading_err=-0.5)
    assert ang < 0

def test_compute_cmd_clamps_linear():
    lin, _ = compute_cmd(dist=100.0, heading_err=0.0)
    assert lin <= 0.5

def test_compute_cmd_clamps_angular():
    _, ang = compute_cmd(dist=1.0, heading_err=math.pi)
    assert abs(ang) <= 1.2

def test_compute_cmd_zero_dist_stops():
    lin, _ = compute_cmd(dist=0.0, heading_err=0.0)
    assert lin == pytest.approx(0.0)


# ── pure_pursuit_curvature ────────────────────────────────────────────────────

def test_pure_pursuit_goal_directly_ahead():
    assert pure_pursuit_curvature(0.0, 0.0, 0.0, 2.0, 0.0) == pytest.approx(0.0, abs=1e-9)

def test_pure_pursuit_goal_to_left():
    assert pure_pursuit_curvature(0.0, 0.0, 0.0, 0.0, 2.0) > 0

def test_pure_pursuit_goal_to_right():
    assert pure_pursuit_curvature(0.0, 0.0, 0.0, 0.0, -2.0) < 0

def test_pure_pursuit_symmetry():
    kl = pure_pursuit_curvature(0.0, 0.0, 0.0, 1.0,  1.0)
    kr = pure_pursuit_curvature(0.0, 0.0, 0.0, 1.0, -1.0)
    assert abs(kl) == pytest.approx(abs(kr), rel=1e-6)
    assert kl > 0 and kr < 0

def test_pure_pursuit_behind_robot():
    assert abs(pure_pursuit_curvature(0.0, 0.0, 0.0, -1.0, 0.0)) < 1e-6

def test_pure_pursuit_zero_distance():
    assert pure_pursuit_curvature(3.0, 4.0, 1.0, 3.0, 4.0) == pytest.approx(0.0)


# ── find_lookahead_point ──────────────────────────────────────────────────────

def test_find_lookahead_basic():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    pt, _ = find_lookahead_point(path, 0.0, 0.0, 0, 1.5)
    assert math.hypot(pt[0], pt[1]) >= 1.5

def test_find_lookahead_returns_last_if_path_short():
    path = [(0.0, 0.0), (0.5, 0.0)]
    pt, idx = find_lookahead_point(path, 0.0, 0.0, 0, 2.0)
    assert pt == (0.5, 0.0)
    assert idx == 1

def test_find_lookahead_respects_start_idx():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]
    _, idx = find_lookahead_point(path, 1.8, 0.0, 2, 0.5)
    assert idx >= 2

def test_find_lookahead_exact_distance():
    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    pt, _ = find_lookahead_point(path, 0.0, 0.0, 0, 1.0)
    assert pt[0] == pytest.approx(1.0)
    assert pt[1] == pytest.approx(0.0)


# ── pure_pursuit_cmd ──────────────────────────────────────────────────────────

# --- alpha=0 (heading aligned): behaviour identical to original ---

def test_pure_pursuit_cmd_forward():
    lin, ang = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=0.0)
    assert lin > 0
    assert ang == pytest.approx(0.0, abs=1e-9)

def test_pure_pursuit_cmd_turn_left():
    lin, ang = pure_pursuit_cmd(dist=2.0, curvature=1.0, alpha=0.0)
    assert lin > 0 and ang > 0

def test_pure_pursuit_cmd_turn_right():
    lin, ang = pure_pursuit_cmd(dist=2.0, curvature=-1.0, alpha=0.0)
    assert lin > 0 and ang < 0

def test_pure_pursuit_cmd_clamps_angular():
    _, ang = pure_pursuit_cmd(dist=3.0, curvature=100.0, alpha=0.0)
    assert abs(ang) <= 1.2

def test_pure_pursuit_cmd_zero_dist():
    lin, _ = pure_pursuit_cmd(dist=0.0, curvature=0.5, alpha=0.0)
    assert lin == pytest.approx(0.0)

# --- cos(alpha) speed scaling ---

def test_pure_pursuit_cmd_45deg_scales_speed():
    alpha = math.pi / 4
    lin_scaled, _ = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=alpha)
    lin_full,   _ = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=0.0)
    assert lin_scaled == pytest.approx(lin_full * math.cos(alpha), rel=1e-6)

def test_pure_pursuit_cmd_45deg_uses_pure_pursuit_angular():
    alpha = math.pi / 4
    lin, ang = pure_pursuit_cmd(dist=3.0, curvature=1.0, alpha=alpha)
    assert lin > 0
    assert ang == pytest.approx(lin * 1.0, rel=1e-6)

# --- rotate-in-place mode (heading error >= 90 deg) ---

def test_pure_pursuit_cmd_90deg_stops():
    lin, _ = pure_pursuit_cmd(dist=3.0, curvature=1.0, alpha=math.pi / 2)
    assert lin == pytest.approx(0.0, abs=1e-9)

def test_pure_pursuit_cmd_90deg_rotates_left():
    _, ang = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=math.pi / 2)
    assert ang > 0

def test_pure_pursuit_cmd_135deg_stops_and_rotates_left():
    lin, ang = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=3 * math.pi / 4)
    assert lin == pytest.approx(0.0, abs=1e-9)
    assert ang > 0

def test_pure_pursuit_cmd_neg_90deg_rotates_right():
    _, ang = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=-math.pi / 2)
    assert ang < 0

def test_pure_pursuit_cmd_180deg_stops():
    lin, _ = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=math.pi)
    assert lin == pytest.approx(0.0, abs=1e-9)

def test_pure_pursuit_cmd_rotate_clamps_angular():
    _, ang = pure_pursuit_cmd(dist=3.0, curvature=0.0, alpha=math.pi)
    assert abs(ang) <= 1.2
