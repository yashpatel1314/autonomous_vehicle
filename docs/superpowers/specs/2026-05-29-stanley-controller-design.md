# Stanley Controller — Design Spec
**Date:** 2026-05-29
**Status:** Approved

## Problem

Pure Pursuit chases a lookahead point rather than tracking the path line. At corners
the lookahead point is already around the bend, so the robot cuts wide. Reducing
lookahead distance helps but never eliminates the problem — it is structural.

## Design

### Algorithm

The Stanley controller was developed for Stanford's DARPA Grand Challenge car. It
minimises two errors simultaneously on each control tick:

| Error | Definition | Effect |
|-------|------------|--------|
| `ψ_e` | path tangent heading − robot yaw | aligns robot direction to path |
| `e` | signed lateral distance from robot to nearest path point (+ = left of path) | pulls robot back onto the line |

Angular command:

```
ω = K_ψ·ψ_e  −  arctan(K_e·e / (v + K_soft))
```

- `K_ψ·ψ_e` — proportional heading correction
- `arctan(K_e·e / v)` — cross-track correction that saturates gracefully
- `K_soft` — softening constant prevents division-by-zero at standstill

Speed scaling by `cos(ψ_e)` (carried over from the Pure Pursuit turn fix) slows the
robot smoothly as heading error grows. At `|ψ_e| ≥ 90°` linear speed drops to zero and
angular is switched to `KP_ANGULAR·ψ_e` (P-control, rotate in place).

### Initial Gains

```
K_PSI  = 1.0   # heading gain
K_E    = 1.0   # cross-track gain
K_SOFT = 0.5   # m/s softening
```

### Files Changed

**`src/av_sim/av_sim/control_math.py`**

Remove: `LOOKAHEAD_DIST`, `pure_pursuit_curvature`, `find_lookahead_point`, `pure_pursuit_cmd`

Add:

```python
K_PSI  = 1.0
K_E    = 1.0
K_SOFT = 0.5

def find_nearest_segment(path, robot_x, robot_y, start_idx):
    """Return (foot_x, foot_y, signed_cte, path_heading, segment_idx).
    signed_cte > 0 means robot is to the LEFT of the path direction."""

def stanley_cmd(dist, psi_e, cte, speed):
    """Stanley controller. psi_e = path_heading − robot_yaw.
    Returns (linear_x, angular_z)."""
```

**`src/av_sim/av_sim/controller.py`**

- Import `_normalise`, `find_nearest_segment`, `stanley_cmd`; drop Pure Pursuit imports
- Add `self._robot_speed` updated from `odom.twist.twist.linear.x`
- `_control_loop`: call `find_nearest_segment`, compute `psi_e = _normalise(path_hdg − robot_yaw)`, call `stanley_cmd`

**`src/av_sim/test/test_controller_math.py`**

Replace Pure Pursuit test sections with Stanley equivalents covering:
- `find_nearest_segment`: on-path (cte≈0), left, right, before start, past end
- `stanley_cmd`: aligned, heading errors, CTE corrections, rotate-in-place, clamping

### What Does Not Change

- A* planning, inflation, path pruning — untouched
- `_normalise`, `_yaw_from_quat`, `heading_error`, `compute_cmd` — untouched
- All stuck-detection and checkpoint logic — untouched
- `MAX_LINEAR`, `MAX_ANGULAR`, `KP_LINEAR`, `KP_ANGULAR` — untouched

## Tuning Guide

| Symptom | Adjustment |
|---------|-----------|
| Robot oscillates on straight sections | Reduce `K_E` |
| Robot cuts corners slightly | Increase `K_E` |
| Slow heading alignment | Increase `K_PSI` |
| Overshoot / oscillate into turns | Reduce `K_PSI` |
| Wild swings at near-zero speed | Increase `K_SOFT` |
