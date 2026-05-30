"""Unit tests for pure controller math functions."""
import math
import pytest

from av_sim.control_math import (
    _normalise, _yaw_from_quat,
    heading_error, compute_cmd,
    find_nearest_segment, stanley_cmd,
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


# ── find_nearest_segment ──────────────────────────────────────────────────────

def test_find_nearest_segment_on_path():
    path = [(0.0, 0.0), (4.0, 0.0)]
    fx, fy, cte, hdg, idx = find_nearest_segment(path, 2.0, 0.0, 0)
    assert fx == pytest.approx(2.0)
    assert fy == pytest.approx(0.0)
    assert abs(cte) < 1e-9

def test_find_nearest_segment_left_of_path():
    # Robot north of east-going segment → cte > 0
    path = [(0.0, 0.0), (4.0, 0.0)]
    _, _, cte, _, _ = find_nearest_segment(path, 2.0, 1.0, 0)
    assert cte > 0

def test_find_nearest_segment_right_of_path():
    # Robot south of east-going segment → cte < 0
    path = [(0.0, 0.0), (4.0, 0.0)]
    _, _, cte, _, _ = find_nearest_segment(path, 2.0, -1.0, 0)
    assert cte < 0

def test_find_nearest_segment_cte_magnitude():
    path = [(0.0, 0.0), (4.0, 0.0)]
    _, _, cte, _, _ = find_nearest_segment(path, 2.0, 0.5, 0)
    assert abs(cte) == pytest.approx(0.5, abs=1e-6)

def test_find_nearest_segment_before_start_clamps_to_first():
    path = [(2.0, 0.0), (4.0, 0.0)]
    fx, fy, _, _, _ = find_nearest_segment(path, 0.0, 0.0, 0)
    assert fx == pytest.approx(2.0)
    assert fy == pytest.approx(0.0)

def test_find_nearest_segment_past_end_clamps_to_last():
    path = [(0.0, 0.0), (2.0, 0.0)]
    fx, fy, _, _, _ = find_nearest_segment(path, 5.0, 0.0, 0)
    assert fx == pytest.approx(2.0)
    assert fy == pytest.approx(0.0)

def test_find_nearest_segment_heading():
    path = [(0.0, 0.0), (0.0, 4.0)]   # heading north = π/2
    _, _, _, hdg, _ = find_nearest_segment(path, 0.0, 2.0, 0)
    assert hdg == pytest.approx(math.pi / 2, abs=1e-6)

def test_find_nearest_segment_respects_start_idx():
    path = [(0.0, 0.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0)]
    _, _, _, _, idx = find_nearest_segment(path, 5.0, 0.0, 2)
    assert idx >= 2

def test_find_nearest_segment_single_point_path():
    path = [(3.0, 3.0)]
    fx, fy, cte, hdg, idx = find_nearest_segment(path, 1.0, 1.0, 0)
    assert fx == pytest.approx(3.0)
    assert fy == pytest.approx(3.0)
    assert idx == 0


# ── stanley_cmd ───────────────────────────────────────────────────────────────

def test_stanley_cmd_aligned_no_cte():
    lin, ang = stanley_cmd(dist=3.0, psi_e=0.0, cte=0.0, speed=0.3)
    assert lin > 0
    assert ang == pytest.approx(0.0, abs=1e-9)

def test_stanley_cmd_positive_psi_e_turns_left():
    _, ang = stanley_cmd(dist=2.0, psi_e=0.3, cte=0.0, speed=0.3)
    assert ang > 0

def test_stanley_cmd_negative_psi_e_turns_right():
    _, ang = stanley_cmd(dist=2.0, psi_e=-0.3, cte=0.0, speed=0.3)
    assert ang < 0

def test_stanley_cmd_robot_left_of_path_turns_right():
    # cte > 0 → robot left → should turn right (negative angular)
    _, ang = stanley_cmd(dist=2.0, psi_e=0.0, cte=0.5, speed=0.3)
    assert ang < 0

def test_stanley_cmd_robot_right_of_path_turns_left():
    # cte < 0 → robot right → should turn left (positive angular)
    _, ang = stanley_cmd(dist=2.0, psi_e=0.0, cte=-0.5, speed=0.3)
    assert ang > 0

def test_stanley_cmd_zero_dist_stops():
    lin, _ = stanley_cmd(dist=0.0, psi_e=0.0, cte=0.0, speed=0.3)
    assert lin == pytest.approx(0.0)

def test_stanley_cmd_clamps_linear():
    lin, _ = stanley_cmd(dist=100.0, psi_e=0.0, cte=0.0, speed=0.3)
    assert lin <= 0.5

def test_stanley_cmd_clamps_angular():
    _, ang = stanley_cmd(dist=2.0, psi_e=0.0, cte=100.0, speed=0.3)
    assert abs(ang) <= 1.2

def test_stanley_cmd_90deg_stops_linear():
    lin, _ = stanley_cmd(dist=3.0, psi_e=math.pi / 2, cte=0.0, speed=0.3)
    assert lin == pytest.approx(0.0, abs=1e-9)

def test_stanley_cmd_90deg_rotates_left():
    _, ang = stanley_cmd(dist=3.0, psi_e=math.pi / 2, cte=0.0, speed=0.3)
    assert ang > 0

def test_stanley_cmd_neg_90deg_rotates_right():
    _, ang = stanley_cmd(dist=3.0, psi_e=-math.pi / 2, cte=0.0, speed=0.3)
    assert ang < 0

def test_stanley_cmd_speed_scaling():
    alpha = math.pi / 4
    lin_45, _ = stanley_cmd(dist=3.0, psi_e=alpha, cte=0.0, speed=0.3)
    lin_0,  _ = stanley_cmd(dist=3.0, psi_e=0.0,   cte=0.0, speed=0.3)
    assert lin_45 == pytest.approx(lin_0 * math.cos(alpha), rel=1e-6)

def test_stanley_cmd_softening_at_zero_speed():
    # Should not raise and angular should be bounded
    lin, ang = stanley_cmd(dist=2.0, psi_e=0.0, cte=0.5, speed=0.0)
    assert abs(ang) <= 1.2
