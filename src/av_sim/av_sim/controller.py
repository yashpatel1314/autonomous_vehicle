"""controller — Stanley path follower with external path override support."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, Int32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from av_sim.control_math import (
    _yaw_from_quat,
    _normalise,
    find_nearest_segment,
    stanley_cmd,
)

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

WAYPOINT_RADIUS   = 0.35   # m — path waypoint considered reached
CHECKPOINT_RADIUS = 0.75   # m — checkpoint detection radius
STUCK_TIMEOUT     = 6.0    # s
MIN_MOVE_M        = 0.08   # m


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self._planned_path: list  = []   # [(x,y)]
        self._override_path: list = []   # [(x,y)]
        self._using_override      = False
        self._planned_idx         = 0
        self._override_idx        = 0

        self._robot_x     = 0.5
        self._robot_y     = 0.5
        self._robot_yaw   = 0.0
        self._robot_speed = 0.0   # m/s forward speed from odometry

        self._checkpoints: list = []   # [(wx,wy)]
        self._cp_idx            = 0

        self._last_pos       = (0.5, 0.5)
        self._last_move_time = self.get_clock().now()

        self._tf_br = TransformBroadcaster(self)

        self.create_subscription(Path,      '/planned_path',  self._on_planned,  _LATCHED)
        self.create_subscription(Path,      '/override_path', self._on_override, 10)
        self.create_subscription(Odometry,  '/odom',          self._on_odom,     10)
        self.create_subscription(PoseArray, '/checkpoints',   self._on_checkpoints, _LATCHED)

        self._cmd_pub     = self.create_publisher(Twist,  '/cmd_vel',            10)
        self._marker_pub  = self.create_publisher(Marker, '/lookahead_marker',   10)
        self._cp_pub      = self.create_publisher(Int32,  '/checkpoint_reached', 10)
        self._replan_pub  = self.create_publisher(Empty,  '/replan_request',     10)
        self._exhaust_pub = self.create_publisher(Empty,  '/override_exhausted', 10)

        self.create_timer(0.1, self._control_loop)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _on_planned(self, msg):
        self._planned_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._planned_idx = 0

    def _on_override(self, msg):
        self._override_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._using_override = True
        self._override_idx = 0
        self.get_logger().info('Switched to override path')

    def _on_odom(self, msg):
        p = msg.pose.pose
        self._robot_x     = p.position.x
        self._robot_y     = p.position.y
        self._robot_yaw   = _yaw_from_quat(p.orientation)
        self._robot_speed = abs(msg.twist.twist.linear.x)
        self._broadcast_tf(msg)

    def _on_checkpoints(self, msg):
        self._checkpoints = [(p.position.x, p.position.y) for p in msg.poses]

    # ── control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        self._check_stuck()

        active = self._override_path if self._using_override else self._planned_path
        if not active:
            self._publish_cmd(0.0, 0.0)
            return

        self._check_checkpoints()

        cur_idx = self._override_idx if self._using_override else self._planned_idx
        fx, fy, cte, path_hdg, new_idx = find_nearest_segment(
            active, self._robot_x, self._robot_y, cur_idx
        )
        if self._using_override:
            self._override_idx = max(self._override_idx, new_idx)
        else:
            self._planned_idx = max(self._planned_idx, new_idx)

        dist_to_end = math.hypot(
            active[-1][0] - self._robot_x,
            active[-1][1] - self._robot_y,
        )

        if self._using_override and new_idx >= len(self._override_path) - 2 and dist_to_end < 0.5:
            self._using_override = False
            self._override_idx = 0
            self._exhaust_pub.publish(Empty())
            self.get_logger().info('Override path exhausted, reverting to planned path')
            return

        psi_e    = _normalise(path_hdg - self._robot_yaw)
        lin, ang = stanley_cmd(dist_to_end, psi_e, cte, self._robot_speed)
        self._publish_cmd(lin, ang)
        self._publish_lookahead((fx, fy))

    def _check_checkpoints(self):
        if self._cp_idx >= len(self._checkpoints):
            return
        tx, ty = self._checkpoints[self._cp_idx]
        if math.hypot(tx - self._robot_x, ty - self._robot_y) < CHECKPOINT_RADIUS:
            msg = Int32()
            msg.data = self._cp_idx
            self._cp_pub.publish(msg)
            self.get_logger().info(f'Checkpoint {self._cp_idx} reached')
            self._cp_idx += 1

    def _check_stuck(self):
        now = self.get_clock().now()
        if math.hypot(self._robot_x - self._last_pos[0],
                      self._robot_y - self._last_pos[1]) > MIN_MOVE_M:
            self._last_pos = (self._robot_x, self._robot_y)
            self._last_move_time = now
        elif (now - self._last_move_time).nanoseconds / 1e9 > STUCK_TIMEOUT:
            self.get_logger().warning('Robot stuck — requesting replan')
            self._replan_pub.publish(Empty())
            self._last_move_time = now

    # ── helpers ───────────────────────────────────────────────────────────────

    def _publish_cmd(self, linear, angular):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self._cmd_pub.publish(t)

    def _publish_lookahead(self, pt):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pt[0]
        m.pose.position.y = pt[1]
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
        m.lifetime = Duration(sec=0, nanosec=200_000_000)
        self._marker_pub.publish(m)

    def _broadcast_tf(self, odom_msg):
        t = TransformStamped()
        t.header = odom_msg.header
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation      = odom_msg.pose.pose.orientation
        self._tf_br.sendTransform(t)


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()
