"""map_manager — loads CSVs and publishes static obstacle + checkpoint data."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

from av_sim.map_math import load_obstacles, load_checkpoints, cell_to_world

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MapManager(Node):

    def __init__(self):
        super().__init__('map_manager')
        self.declare_parameter('obstacles_csv',   '')
        self.declare_parameter('checkpoints_csv', '')
        self.declare_parameter('cell_size',       1.0)
        self.declare_parameter('grid_width',      30)
        self.declare_parameter('grid_height',     30)

        obs_csv = self.get_parameter('obstacles_csv').value
        cp_csv  = self.get_parameter('checkpoints_csv').value
        cs      = self.get_parameter('cell_size').value
        gw      = self.get_parameter('grid_width').value
        gh      = self.get_parameter('grid_height').value

        obs = load_obstacles(obs_csv)
        cps = load_checkpoints(cp_csv)

        obs_pub    = self.create_publisher(OccupancyGrid, '/map/obstacles',      _LATCHED)
        cp_pub     = self.create_publisher(PoseArray,     '/checkpoints',        _LATCHED)
        marker_pub = self.create_publisher(MarkerArray,   '/checkpoint_markers', _LATCHED)

        obs_pub.publish(self._build_grid(obs, gw, gh, cs))
        cp_pub.publish(self._build_pose_array(cps, cs))
        marker_pub.publish(self._build_markers(cps, cs))

        self.get_logger().info(
            f'Published {len(obs)} obstacles, {len(cps)} checkpoints'
        )

    def _build_grid(self, obstacles, gw, gh, cs):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = cs
        msg.info.width  = gw
        msg.info.height = gh
        data = [0] * (gw * gh)
        for gx, gy in obstacles:
            if 0 <= gx < gw and 0 <= gy < gh:
                data[gy * gw + gx] = 100
        msg.data = data
        return msg

    def _build_pose_array(self, checkpoints, cs):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        for gx, gy in checkpoints:
            wx, wy = cell_to_world(gx, gy, cs)
            p = Pose()
            p.position.x = wx
            p.position.y = wy
            p.orientation.w = 1.0
            msg.poses.append(p)
        return msg

    def _build_markers(self, checkpoints, cs):
        arr = MarkerArray()
        for i, (gx, gy) in enumerate(checkpoints):
            wx, wy = cell_to_world(gx, gy, cs)
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'checkpoints'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.6
            m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.1, 0.9
            m.lifetime = Duration()
            arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    node = MapManager()
    rclpy.spin(node)
    rclpy.shutdown()
