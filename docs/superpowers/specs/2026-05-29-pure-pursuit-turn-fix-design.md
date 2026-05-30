# Pure Pursuit Turn Fix — Design Spec
**Date:** 2026-05-29
**Status:** Approved

## Problem

The robot makes catastrophically wide turns at near-180° direction changes (cp2→cp3, cp3→cp4). It overshoots into the world boundary and beyond before correcting.

**Root cause:** Pure Pursuit computes steering as `κ = 2·sin(α)/d` and angular velocity as `angular = linear × κ`. When heading error `α` is near ±180°, `sin(α) ≈ 0` → near-zero curvature → robot drives straight despite needing to reverse direction. Compounding this, `angular` is derived from `linear`, so stopping linear speed also kills the angular command — the robot cannot rotate in place under the current formula.

## Design

### Behaviour

Two modes, determined by heading error `α` to the lookahead point each control tick:

| Heading error | Linear speed | Angular command |
|---------------|-------------|-----------------|
| < 90° | `min(MAX_LINEAR, KP_LINEAR × dist) × cos(α)` | Pure Pursuit: `linear × curvature` |
| ≥ 90° | 0 | P-control: `KP_ANGULAR × α` (clamped to `MAX_ANGULAR`) |

**Speed scaling by `cos(α)`** smoothly ramps speed to zero as heading error approaches 90°. The robot naturally slows before sharp turns without needing a separate braking system. This also provides a clean foundation for raising `MAX_LINEAR` later — the cos scaling adapts automatically at any top speed.

**Decoupled angular at large errors** means the robot rotates in place when stopped, using the existing `KP_ANGULAR` gain. Once aligned within 90°, Pure Pursuit resumes and speed ramps back up via the cos factor.

### Files Changed

**`src/av_sim/av_sim/control_math.py`**

`pure_pursuit_cmd(dist, curvature)` → `pure_pursuit_cmd(dist, curvature, alpha=0.0)`

```python
def pure_pursuit_cmd(dist: float, curvature: float, alpha: float = 0.0):
    speed_scale = max(0.0, math.cos(alpha))
    linear = min(MAX_LINEAR, KP_LINEAR * dist) * speed_scale
    if speed_scale > 1e-6:          # heading error < 90°
        angular = linear * curvature
    else:                            # heading error ≥ 90° — rotate in place
        angular = KP_ANGULAR * alpha
    return linear, max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
```

`alpha` defaults to `0.0` so the old call signature remains valid.

**`src/av_sim/av_sim/controller.py`**

- Add `heading_error` to import from `control_math`
- In `_control_loop`: compute `alpha` using `heading_error()` after finding the lookahead point, pass it to `pure_pursuit_cmd`

```python
alpha    = heading_error(self._robot_x, self._robot_y, self._robot_yaw, lp[0], lp[1])
k        = pure_pursuit_curvature(self._robot_x, self._robot_y, self._robot_yaw, lp[0], lp[1])
lin, ang = pure_pursuit_cmd(dist=dist_to_end, curvature=k, alpha=alpha)
```

**`src/av_sim/test/test_controller_math.py`**

- Update existing `pure_pursuit_cmd` calls to pass `alpha=0.0` explicitly (preserves existing behaviour assertions)
- Add tests for: 45° heading error (speed scaled, PP angular), 90° (linear=0, P angular), 135° (linear=0, P angular, correct sign)

### What Does Not Change

- A* planning, inflation, path pruning — untouched
- `LOOKAHEAD_DIST`, `MAX_LINEAR`, `MAX_ANGULAR`, `KP_LINEAR` — untouched
- `pure_pursuit_curvature` function — untouched
- All other controller logic — untouched

## Future Speed Work

To increase top speed after this fix: raise `MAX_LINEAR` in `control_math.py`. The `cos(α)` factor automatically adjusts turn entry speed at any top speed value. No structural changes required.
