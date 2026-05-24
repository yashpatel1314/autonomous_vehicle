"""Pure controller math — no ROS dependencies."""
import math


def _yaw_from_quat(q) -> float:
    """Extract yaw (rotation about Z) from a quaternion object with x/y/z/w fields."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

KP_LINEAR = 0.5
KP_ANGULAR = 2.0
MAX_LINEAR = 0.5
MAX_ANGULAR = 1.2
TURN_THRESHOLD = 0.25
LOOKAHEAD_DIST = 1.5     # metres for pure pursuit
FULL_SPEED_DIST = 2.0    # metres — no speed reduction beyond this
STOP_DIST = 0.6          # metres — emergency stop threshold


def _normalise(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def heading_error(robot_x: float, robot_y: float, robot_yaw: float,
                  target_x: float, target_y: float) -> float:
    """Signed heading error (rad) from current yaw to the target position."""
    target_yaw = math.atan2(target_y - robot_y, target_x - robot_x)
    return _normalise(target_yaw - robot_yaw)


def compute_cmd(dist: float, heading_err: float):
    """Proportional controller: return (linear_x, angular_z).

    Rotates in place when |heading_err| > TURN_THRESHOLD, otherwise drives
    forward with proportional angular correction.
    """
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * heading_err))
    if abs(heading_err) > TURN_THRESHOLD:
        return 0.0, angular
    linear = min(MAX_LINEAR, KP_LINEAR * dist)
    return linear, angular


def speed_scale_from_scan(min_dist: float,
                           stop_dist: float = STOP_DIST,
                           full_speed_dist: float = FULL_SPEED_DIST) -> float:
    """Return a [0, 1] speed multiplier based on the nearest forward obstacle.

    Linearly ramps from 0 at *stop_dist* up to 1 at *full_speed_dist*.
    Values outside that range are clamped.
    """
    if min_dist >= full_speed_dist:
        return 1.0
    if min_dist <= stop_dist:
        return 0.0
    return (min_dist - stop_dist) / (full_speed_dist - stop_dist)


# ---------------------------------------------------------------------------
# Pure Pursuit
# ---------------------------------------------------------------------------

def pure_pursuit_curvature(robot_x: float, robot_y: float, robot_yaw: float,
                           goal_x: float, goal_y: float) -> float:
    """Signed curvature κ = 2·sin(α)/d to steer toward (goal_x, goal_y).

    Positive curvature → turn left; negative → turn right.
    Returns 0 when the goal coincides with the robot position.
    """
    dx = goal_x - robot_x
    dy = goal_y - robot_y
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0
    alpha = _normalise(math.atan2(dy, dx) - robot_yaw)
    return 2.0 * math.sin(alpha) / d


def find_lookahead_point(path: list, robot_x: float, robot_y: float,
                         start_idx: int, dist: float):
    """Return ((x, y), idx) of the first path point ≥ *dist* from the robot.

    Searches forward from *start_idx*.  Falls back to the last point if no
    point is far enough.
    """
    for i in range(start_idx, len(path)):
        px, py = path[i]
        if math.hypot(px - robot_x, py - robot_y) >= dist:
            return (px, py), i
    return path[-1], len(path) - 1


def pure_pursuit_cmd(dist: float, curvature: float):
    """Pure Pursuit controller: return (linear_x, angular_z).

    *dist* is the remaining path length (used for speed scaling).
    *curvature* is the output of pure_pursuit_curvature().
    """
    linear = min(MAX_LINEAR, KP_LINEAR * dist)
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, linear * curvature))
    return linear, angular