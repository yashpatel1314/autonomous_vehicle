"""Pure controller math — no ROS dependencies."""
import math

KP_LINEAR    = 0.5
KP_ANGULAR   = 2.0
MAX_LINEAR   = 0.5
MAX_ANGULAR  = 1.2
TURN_THRESHOLD = 0.25
K_PSI        = 1.0   # Stanley: path-heading-error gain
K_E          = 1.0   # Stanley: cross-track-error gain
K_SOFT       = 0.5   # Stanley: speed softening (prevents ÷0 at standstill)


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


def find_nearest_segment(path: list, robot_x: float, robot_y: float, start_idx: int):
    """Return (foot_x, foot_y, signed_cte, path_heading, segment_idx).

    Scans path segments from start_idx forward and returns the one closest to
    the robot.  signed_cte > 0 means the robot is to the LEFT of the path
    direction (cross product of segment direction × foot-to-robot vector).
    """
    if len(path) < 2:
        return path[0][0], path[0][1], 0.0, 0.0, 0

    best_dist    = float('inf')
    best_foot    = (path[start_idx][0], path[start_idx][1])
    best_cte     = 0.0
    best_heading = 0.0
    best_idx     = start_idx

    for i in range(start_idx, len(path) - 1):
        ax, ay = path[i]
        bx, by = path[i + 1]
        dx, dy = bx - ax, by - ay
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-10:
            continue
        t = max(0.0, min(1.0, ((robot_x - ax) * dx + (robot_y - ay) * dy) / seg_len_sq))
        fx, fy = ax + t * dx, ay + t * dy
        d = math.hypot(robot_x - fx, robot_y - fy)
        if d < best_dist:
            best_dist    = d
            best_foot    = (fx, fy)
            best_heading = math.atan2(dy, dx)
            # positive when robot is left of path (CCW from path direction)
            best_cte = (math.cos(best_heading) * (robot_y - fy) -
                        math.sin(best_heading) * (robot_x - fx))
            best_idx = i

    return best_foot[0], best_foot[1], best_cte, best_heading, best_idx


def stanley_cmd(dist: float, psi_e: float, cte: float, speed: float):
    """Stanley path-following controller.

    psi_e: path tangent heading minus robot yaw, normalised to (-π, π]
    cte:   signed cross-track error in metres (positive = robot left of path)
    speed: current forward speed in m/s
    Returns (linear_x, angular_z).
    """
    speed_scale = max(0.0, math.cos(psi_e))
    linear = min(MAX_LINEAR, KP_LINEAR * dist) * speed_scale
    if speed_scale <= 1e-6:
        # Heading error ≥ 90° — rotate in place with P-control
        angular = KP_ANGULAR * psi_e
    else:
        angular = K_PSI * psi_e - math.atan2(K_E * cte, speed + K_SOFT)
    return linear, max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))
