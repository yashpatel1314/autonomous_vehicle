#!/usr/bin/env python3
"""Robot Controller node.

Follows a nav_msgs/Path using Pure Pursuit steering with adaptive speed
scaling based on the nearest forward scan obstacle.

Safety layers:
  - Emergency stop: any scan return within STOP_DIST in forward ±45° → zero vel
  - Adaptive speed: gradually slows from FULL_SPEED_DIST down to STOP_DIST
  - Stuck detection: no movement > MIN_MOVE_M in STUCK_TIMEOUT s →
      publish Empty on /replan_request + reset waypoint index

TF:
  Broadcasts odom→base_link from every /odom message so RViz2 shows live
  robot pose.

Milestone events:
  /current_checkpoint  std_msgs/Int32  (1-indexed, on each reach)
  /mission_complete    std_msgs/Bool   (True once all checkpoints done)

Visualization:
  /checkpoint_markers  visualization_msgs/MarkerArray
      Spheres + labels; green = pending, grey = reached.
  /lookahead_marker    visualization_msgs/Marker
      Orange sphere at the current Pure Pursuit lookahead point.

Topics subscribed:
  /planned_path        nav_msgs/Path
  /odom                nav_msgs/Odometry
  /map/checkpoints     geometry_msgs/PoseArray
  /scan                sensor_msgs/LaserScan

Topics published:
  /cmd_vel             geometry_msgs/Twist
  /current_checkpoint  std_msgs/Int32
  /mission_complete    std_msgs/Bool
  /replan_request      std_msgs/Empty
  /checkpoint_markers  visualization_msgs/MarkerArray
  /lookahead_marker    visualization_msgs/Marker
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, Int32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from av_sim.control_math import (
    _normalise,
    find_lookahead_point,
    pure_pursuit_cmd,
    pure_pursuit_curvature,
    speed_scale_from_scan,
    LOOKAHEAD_DIST,
    MAX_LINEAR,
    KP_LINEAR,
    STOP_DIST,
)

WAYPOINT_RADIUS = 0.35    # m — waypoint considered reached
STOP_ARC = math.radians(45)     # half-angle of emergency-stop scan arc
STUCK_TIMEOUT = 6.0       # s — declare stuck after no movement
MIN_MOVE_M = 0.08         # m — threshold for "has moved"


def _yaw_from_quat(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


# Keep module-level alias so tests that import from this module still work.
_normalise = _normalise  # noqa: F811


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self._path: list = []
        self._wp_idx = 0
        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0
        self._done = False

        self._checkpoints: list = []
        self._next_cp_idx = 0
        self._mission_complete_sent = False

        # Emergency stop + adaptive speed
        self._e_stop = False
        self._min_forward_dist = float('inf')

        # Stuck detection
        self._last_move_x = 0.0
        self._last_move_y = 0.0
        self._last_move_time = self.get_clock().now()

        # TF broadcaster for odom→base_link
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Path, '/planned_path', self._cb_path, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(PoseArray, '/map/checkpoints',
                                 self._cb_checkpoints, 10)
        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)

        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cp_pub = self.create_publisher(Int32, '/current_checkpoint', 10)
        self._mc_pub = self.create_publisher(Bool, '/mission_complete', 10)
        self._replan_pub = self.create_publisher(Empty, '/replan_request', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/checkpoint_markers', 10)
        self._lookahead_pub = self.create_publisher(
            Marker, '/lookahead_marker', 10)

        self.create_timer(0.05, self._control_loop)
        self.create_timer(1.0, self._publish_markers)

    # ------------------------------------------------------------------
    # Callbacks

    def _cb_path(self, msg: Path):
        self._path = [(ps.pose.position.x, ps.pose.position.y)
                      for ps in msg.poses]
        self._wp_idx = 0
        self._done = False
        self._next_cp_idx = 0
        self._mission_complete_sent = False
        self.get_logger().info(f'Received path with {len(self._path)} waypoints')

    def _cb_odom(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        self._robot_yaw = _yaw_from_quat(msg.pose.pose.orientation)
        self._broadcast_tf(msg)

    def _broadcast_tf(self, odom_msg: Odometry):
        t = TransformStamped()
        t.header.stamp = odom_msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        p = odom_msg.pose.pose.position
        q = odom_msg.pose.pose.orientation
        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w
        self._tf_broadcaster.sendTransform(t)

    def _cb_checkpoints(self, msg: PoseArray):
        self._checkpoints = [
            (p.position.x, p.position.y) for p in msg.poses
        ]

    def _cb_scan(self, msg: LaserScan):
        """Set emergency stop flag and compute minimum forward obstacle distance."""
        min_dist = float('inf')
        self._e_stop = False
        for i, r in enumerate(msg.ranges):
            if r <= 0.0 or r >= msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if abs(angle) <= STOP_ARC:
                if r < min_dist:
                    min_dist = r
                if r < STOP_DIST:
                    self._e_stop = True
        self._min_forward_dist = min_dist

    # ------------------------------------------------------------------
    # Control loop helpers

    def _check_checkpoint_reached(self):
        if self._next_cp_idx >= len(self._checkpoints):
            return
        cx, cy = self._checkpoints[self._next_cp_idx]
        if math.hypot(cx - self._robot_x, cy - self._robot_y) <= WAYPOINT_RADIUS:
            reached = self._next_cp_idx + 1
            self._cp_pub.publish(Int32(data=reached))
            self.get_logger().info(f'Checkpoint {reached} reached')
            self._next_cp_idx += 1
            self._publish_markers()

            if (self._next_cp_idx >= len(self._checkpoints)
                    and not self._mission_complete_sent):
                self._mc_pub.publish(Bool(data=True))
                self._mission_complete_sent = True
                self.get_logger().info('Mission complete!')

    def _check_stuck(self):
        moved = math.hypot(self._robot_x - self._last_move_x,
                           self._robot_y - self._last_move_y)
        now = self.get_clock().now()
        if moved >= MIN_MOVE_M:
            self._last_move_x = self._robot_x
            self._last_move_y = self._robot_y
            self._last_move_time = now
            return
        elapsed = (now - self._last_move_time).nanoseconds * 1e-9
        if elapsed > STUCK_TIMEOUT and not self._done and self._path:
            self.get_logger().warn('Robot stuck — requesting replan')
            self._replan_pub.publish(Empty())
            self._last_move_time = now
            self._wp_idx = 0

    # ------------------------------------------------------------------
    # Main loop

    def _control_loop(self):
        self._check_checkpoint_reached()
        self._check_stuck()

        if self._e_stop:
            self._stop()
            return

        if self._done or not self._path:
            return

        # Advance past already-reached waypoints
        while self._wp_idx < len(self._path):
            tx, ty = self._path[self._wp_idx]
            if math.hypot(tx - self._robot_x, ty - self._robot_y) > WAYPOINT_RADIUS:
                break
            self._wp_idx += 1

        if self._wp_idx >= len(self._path):
            self._stop()
            self._done = True
            self.get_logger().info('End of path reached — robot stopped.')
            return

        # Pure Pursuit
        lookahead_pt, _ = find_lookahead_point(
            self._path, self._robot_x, self._robot_y,
            self._wp_idx, LOOKAHEAD_DIST,
        )
        self._publish_lookahead_marker(lookahead_pt)

        kappa = pure_pursuit_curvature(
            self._robot_x, self._robot_y, self._robot_yaw,
            lookahead_pt[0], lookahead_pt[1],
        )

        dist_end = math.hypot(
            self._path[-1][0] - self._robot_x,
            self._path[-1][1] - self._robot_y,
        )
        lin, ang = pure_pursuit_cmd(dist_end, kappa)

        # Adaptive speed: scale linear vel by proximity to scan obstacles
        scale = speed_scale_from_scan(self._min_forward_dist)
        lin *= scale

        cmd = Twist()
        cmd.linear.x = lin
        cmd.angular.z = ang
        self._cmd_pub.publish(cmd)

    def _stop(self):
        self._cmd_pub.publish(Twist())

    # ------------------------------------------------------------------
    # Visualization

    def _publish_lookahead_marker(self, pt):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'lookahead'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pt[0]
        m.pose.position.y = pt[1]
        m.pose.position.z = 0.15
        m.pose.orientation.w = 1.0
        m.scale.x = 0.25
        m.scale.y = 0.25
        m.scale.z = 0.25
        m.color.r = 1.0
        m.color.g = 0.5
        m.color.b = 0.0
        m.color.a = 0.9
        self._lookahead_pub.publish(m)

    def _publish_markers(self):
        if not self._checkpoints:
            return
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (cx, cy) in enumerate(self._checkpoints):
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = now
            m.ns = 'checkpoints'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = cx
            m.pose.position.y = cy
            m.pose.position.z = 0.4
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.4
            if i < self._next_cp_idx:
                m.color.r = m.color.g = m.color.b = 0.5
                m.color.a = 0.7
            else:
                m.color.r = 0.1
                m.color.g = 0.9
                m.color.b = 0.1
                m.color.a = 0.9
            ma.markers.append(m)

            label = Marker()
            label.header = m.header
            label.ns = 'checkpoint_labels'
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position.x = cx
            label.pose.position.y = cy
            label.pose.position.z = 0.8
            label.pose.orientation.w = 1.0
            label.scale.z = 0.35
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = str(i + 1)
            ma.markers.append(label)

        self._marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()