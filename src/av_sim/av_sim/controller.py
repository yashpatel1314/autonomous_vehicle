#!/usr/bin/env python3
"""
Robot Controller node.

Follows a nav_msgs/Path by publishing geometry_msgs/Twist commands.

Strategy (per control loop):
  1. Compute heading error to current waypoint.
  2. If |error| > TURN_THRESHOLD: rotate in place.
  3. Otherwise: drive forward with proportional angular correction.
  4. Advance waypoint index when within WAYPOINT_RADIUS of the current target.

Topics subscribed:
  /planned_path  nav_msgs/Path
  /odom          nav_msgs/Odometry

Topic published:
  /cmd_vel       geometry_msgs/Twist
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


# Control gains and thresholds
KP_LINEAR = 0.5      # proportional gain for forward speed
KP_ANGULAR = 2.0     # proportional gain for heading correction
MAX_LINEAR = 0.5     # m/s
MAX_ANGULAR = 1.2    # rad/s
WAYPOINT_RADIUS = 0.35  # m — advance to next waypoint within this distance
TURN_THRESHOLD = 0.25   # rad — rotate in place if heading error exceeds this


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _normalise(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self._path: list = []   # list of (x, y) waypoints in world frame
        self._wp_idx = 0        # current waypoint index
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._done = False

        self.create_subscription(Path, '/planned_path', self._cb_path, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self._control_loop)

    # ------------------------------------------------------------------

    def _cb_path(self, msg: Path):
        self._path = [(ps.pose.position.x, ps.pose.position.y)
                      for ps in msg.poses]
        self._wp_idx = 0
        self._done = False
        self.get_logger().info(f'Received path with {len(self._path)} waypoints')

    def _cb_odom(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = _yaw_from_quat(msg.pose.pose.orientation)

    # ------------------------------------------------------------------

    def _control_loop(self):
        if self._done or not self._path:
            return

        # Skip waypoints the robot has already passed.
        while self._wp_idx < len(self._path):
            tx, ty = self._path[self._wp_idx]
            dist = math.hypot(tx - self._robot_x, ty - self._robot_y)
            if dist > WAYPOINT_RADIUS:
                break
            self._wp_idx += 1

        if self._wp_idx >= len(self._path):
            self._stop()
            self._done = True
            self.get_logger().info('Goal reached — robot stopped.')
            return

        tx, ty = self._path[self._wp_idx]
        dist = math.hypot(tx - self._robot_x, ty - self._robot_y)
        target_yaw = math.atan2(ty - self._robot_y, tx - self._robot_x)
        err = _normalise(target_yaw - self._robot_yaw)

        cmd = Twist()
        if abs(err) > TURN_THRESHOLD:
            # Rotate in place
            cmd.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * err))
        else:
            cmd.linear.x = min(MAX_LINEAR, KP_LINEAR * dist)
            cmd.angular.z = max(-MAX_ANGULAR, min(MAX_ANGULAR, KP_ANGULAR * err))

        self._cmd_pub.publish(cmd)

    def _stop(self):
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
