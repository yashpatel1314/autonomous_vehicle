"""astar_planner — replanning A* node with lidar dynamic obstacle layer."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Int32

from av_sim.astar_math import astar, inflate_obstacles, prune_path
from av_sim.map_math import cell_to_world, world_to_cell, in_bounds

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class AstarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')
        self.declare_parameter('grid_width',       30)
        self.declare_parameter('grid_height',      30)
        self.declare_parameter('cell_size',         1.0)
        self.declare_parameter('inflation_radius',  1)

        self._gw  = self.get_parameter('grid_width').value
        self._gh  = self.get_parameter('grid_height').value
        self._cs  = self.get_parameter('cell_size').value
        self._inf = self.get_parameter('inflation_radius').value

        self._static_obs: set = set()
        self._scan_obs:   set = set()
        self._checkpoints: list = []   # [(wx, wy), ...]
        self._cp_idx: int = 0
        self._robot_x: float = 0.5
        self._robot_y: float = 0.5
        self._robot_yaw: float = 0.0
        self._last_path: list = []     # [(wx, wy), ...]

        self.create_subscription(OccupancyGrid, '/map/obstacles',      self._on_map,       _LATCHED)
        self.create_subscription(PoseArray,     '/checkpoints',        self._on_checkpoints, _LATCHED)
        self.create_subscription(Odometry,      '/odom',               self._on_odom,      10)
        self.create_subscription(LaserScan,     '/scan',               self._on_scan,      10)
        self.create_subscription(Int32,         '/checkpoint_reached', self._on_cp_reached, 10)
        self.create_subscription(Empty,         '/replan_request',     self._on_replan,    10)

        self._path_pub    = self.create_publisher(Path,         '/planned_path', 1)
        self._inflated_pub = self.create_publisher(OccupancyGrid, '/map/inflated', 1)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _on_map(self, msg):
        self._static_obs = set()
        w, h = msg.info.width, msg.info.height
        for i, v in enumerate(msg.data):
            if v == 100:
                self._static_obs.add((i % w, i // w))
        self._replan()

    def _on_checkpoints(self, msg):
        self._checkpoints = [(p.position.x, p.position.y) for p in msg.poses]
        self._cp_idx = 0
        self._replan()

    def _on_odom(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _on_scan(self, msg):
        new_obs: set = set()
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                wx = self._robot_x + r * math.cos(self._robot_yaw + angle)
                wy = self._robot_y + r * math.sin(self._robot_yaw + angle)
                gx, gy = world_to_cell(wx, wy, self._cs)
                if in_bounds(gx, gy, self._gw, self._gh):
                    new_obs.add((gx, gy))
            angle += msg.angle_increment

        added = new_obs - self._scan_obs
        self._scan_obs = new_obs
        if added and self._path_crosses(added):
            self._replan()

    def _on_cp_reached(self, msg):
        self._cp_idx = msg.data + 1
        if self._cp_idx < len(self._checkpoints):
            self._replan()
        else:
            self.get_logger().info('All checkpoints reached!')

    def _on_replan(self, _msg):
        self._replan()

    # ── planning ───────────────────────────────────────────────────────────────

    def _path_crosses(self, cells: set) -> bool:
        for wx, wy in self._last_path:
            if world_to_cell(wx, wy, self._cs) in cells:
                return True
        return False

    def _replan(self):
        if not self._checkpoints or self._cp_idx >= len(self._checkpoints):
            return

        all_obs  = self._static_obs | self._scan_obs
        inflated = inflate_obstacles(all_obs, self._gw, self._gh, self._inf)

        inf_msg = OccupancyGrid()
        inf_msg.header.frame_id = 'map'
        inf_msg.info.resolution = self._cs
        inf_msg.info.width  = self._gw
        inf_msg.info.height = self._gh
        inf_data = [0] * (self._gw * self._gh)
        for igx, igy in inflated:
            inf_data[igy * self._gw + igx] = 100
        inf_msg.data = inf_data
        self._inflated_pub.publish(inf_msg)

        sx = max(0, min(self._gw - 1, int(self._robot_x / self._cs)))
        sy = max(0, min(self._gh - 1, int(self._robot_y / self._cs)))
        tx, ty = self._checkpoints[self._cp_idx]
        gx = max(0, min(self._gw - 1, int(tx / self._cs)))
        gy = max(0, min(self._gh - 1, int(ty / self._cs)))

        cells = astar((sx, sy), (gx, gy), inflated, self._gw, self._gh)
        if cells is None:
            cells = astar((sx, sy), (gx, gy), all_obs, self._gw, self._gh)
        if cells is None:
            self.get_logger().warn(f'No path from ({sx},{sy}) to ({gx},{gy})')
            return

        cells = prune_path(cells)
        world_pts = [cell_to_world(c[0], c[1], self._cs) for c in cells]
        self._last_path = world_pts

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in world_pts:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f'Replanned: {len(cells)} waypoints → cp[{self._cp_idx}] ({tx:.1f},{ty:.1f})'
        )


def main():
    rclpy.init()
    node = AstarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()
