"""Pure controller math — no ROS dependencies."""
import math

KP_LINEAR    = 0.5
KP_ANGULAR   = 2.0
MAX_LINEAR   = 0.5
MAX_ANGULAR  = 1.2
TURN_THRESHOLD  = 0.25
LOOKAHEAD_DIST  = 1.5   # metres — pure pursuit lookahead


def _yaw_from_quat(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _normalise(angle: float) -> float:
    while angle >  math.pi: angle -= 2.0 * math.pi
    while angle < -math.pi: angle += 2.0 * math.pi
    return angle


def heading_error(robot_x, robot_y, robot_yaw, target_x, target_y) -> float:
    return _normalise(math.atan2(target_y - robot_y, target_x - robot_x) - robot_yaw)


def compute_cmd(dist: float, heading_err: float):
    """Proportional controller — turn in place when misaligned, else drive."""
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * heading_err))
    if abs(heading_err) > TURN_THRESHOLD:
        return 0.0, angular
    return min(MAX_LINEAR, KP_LINEAR * dist), angular


def pure_pursuit_curvature(robot_x, robot_y, robot_yaw, goal_x, goal_y) -> float:
    """Signed curvature κ = 2·sin(α)/d to steer toward the goal."""
    dx, dy = goal_x - robot_x, goal_y - robot_y
    d = math.hypot(dx, dy)
    if d < 1e-6:
        return 0.0
    alpha = _normalise(math.atan2(dy, dx) - robot_yaw)
    return 2.0 * math.sin(alpha) / d


def find_lookahead_point(path: list, robot_x: float, robot_y: float,
                         start_idx: int, dist: float):
    """Return ((x, y), idx) of the first path point at least *dist* ahead."""
    for i in range(start_idx, len(path)):
        px, py = path[i]
        if math.hypot(px - robot_x, py - robot_y) >= dist:
            return (px, py), i
    return path[-1], len(path) - 1


def pure_pursuit_cmd(dist: float, curvature: float):
    """Return (linear_x, angular_z). Slows as end-of-path distance shrinks."""
    linear  = min(MAX_LINEAR, KP_LINEAR * dist)
    angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, linear * curvature))
    return linear, angular
