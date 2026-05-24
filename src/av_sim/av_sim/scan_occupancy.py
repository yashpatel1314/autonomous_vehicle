#!/usr/bin/env python3
"""Scan Occupancy node.

Converts live /scan readings to a GridCells message on /map/scan_obstacles.
Cells decay after DECAY_TIME seconds so dynamic obstacles that move away
eventually disappear from the occupancy layer.

Topics subscribed:
  /scan   sensor_msgs/LaserScan
  /odom   nav_msgs/Odometry  (robot pose for scan→world transform)

Topic published:
  /map/scan_obstacles  nav_msgs/GridCells
"""

import math
import time

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Point
    from nav_msgs.msg import GridCells, Odometry
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import Header
except ImportError:
    rclpy = None  # type: ignore[assignment]
    Node = object  # type: ignore[assignment,misc]

# Laser link offset from base_link (must match robot.urdf)
LASER_OFFSET_X = 0.20   # m forward
LASER_OFFSET_Y = 0.00

DECAY_TIME = 3.0          # seconds before a detected cell expires
PUBLISH_HZ = 5.0


def scan_hits_to_grid(ranges, angle_min: float, angle_step: float,
                      robot_yaw: float, laser_x: float, laser_y: float,
                      cell_size: float, grid_w: int, grid_h: int,
                      range_max: float) -> set:
    """Convert laser scan ranges to a set of occupied grid (gx, gy) cells.

    Pure function — no ROS dependencies.  Exported for unit testing.
    """
    cells: set = set()
    for i, r in enumerate(ranges):
        if r <= 0.0 or r >= range_max:
            continue
        angle = angle_min + i * angle_step + robot_yaw
        wx = laser_x + r * math.cos(angle)
        wy = laser_y + r * math.sin(angle)
        gx = int(wx / cell_size)
        gy = int(wy / cell_size)
        if 0 <= gx < grid_w and 0 <= gy < grid_h:
            cells.add((gx, gy))
    return cells


class ScanOccupancy(Node):

    def __init__(self):
        super().__init__('scan_occupancy')

        self.declare_parameter('grid_width', 20)
        self.declare_parameter('grid_height', 20)
        self.declare_parameter('cell_size', 1.0)

        self._gw = self.get_parameter('grid_width').get_parameter_value().integer_value
        self._gh = self.get_parameter('grid_height').get_parameter_value().integer_value
        self._cs = self.get_parameter('cell_size').get_parameter_value().double_value

        self._robot_x = 0.0
        self._robot_y = 0.0
        self._robot_yaw = 0.0

        # {(gx, gy): last_seen_monotonic_time}
        self._cells: dict = {}

        self.create_subscription(LaserScan, '/scan', self._cb_scan, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self._pub = self.create_publisher(GridCells, '/map/scan_obstacles', 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)

    # ------------------------------------------------------------------

    def _cb_odom(self, msg: Odometry):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _cb_scan(self, msg: LaserScan):
        # Transform laser_link origin to world frame
        cos_y = math.cos(self._robot_yaw)
        sin_y = math.sin(self._robot_yaw)
        laser_x = (self._robot_x
                   + LASER_OFFSET_X * cos_y
                   - LASER_OFFSET_Y * sin_y)
        laser_y = (self._robot_y
                   + LASER_OFFSET_X * sin_y
                   + LASER_OFFSET_Y * cos_y)

        new_cells = scan_hits_to_grid(
            msg.ranges, msg.angle_min, msg.angle_increment,
            self._robot_yaw, laser_x, laser_y,
            self._cs, self._gw, self._gh, msg.range_max,
        )
        now = time.monotonic()
        for cell in new_cells:
            self._cells[cell] = now

    def _publish(self):
        now = time.monotonic()
        # Evict expired cells
        self._cells = {
            cell: t for cell, t in self._cells.items()
            if now - t < DECAY_TIME
        }

        msg = GridCells()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='map',
        )
        msg.cell_width = self._cs
        msg.cell_height = self._cs
        for gx, gy in self._cells:
            wx = (gx + 0.5) * self._cs
            wy = (gy + 0.5) * self._cs
            msg.cells.append(Point(x=wx, y=wy, z=0.0))
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanOccupancy()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
