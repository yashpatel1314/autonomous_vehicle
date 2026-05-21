#!/usr/bin/env python3
"""
Map Manager node.

Reads obstacles.csv and checkpoints.csv from the config directory,
then publishes their contents as ROS2 topics for the planner to consume.

Topics published:
  /map/obstacles    nav_msgs/GridCells   — one cell per obstacle row
  /map/checkpoints  geometry_msgs/PoseArray — ordered checkpoint poses
"""

import csv
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import GridCells
from std_msgs.msg import Header


class MapManager(Node):

    def __init__(self):
        super().__init__('map_manager')

        self.declare_parameter('config_dir', '')
        self.declare_parameter('cell_size', 1.0)

        config_dir = self.get_parameter('config_dir').get_parameter_value().string_value
        self._cell_size = self.get_parameter('cell_size').get_parameter_value().double_value

        self._obstacles = self._load_obstacles(
            os.path.join(config_dir, 'obstacles.csv'))
        self._checkpoints = self._load_checkpoints(
            os.path.join(config_dir, 'checkpoints.csv'))

        self._obs_pub = self.create_publisher(GridCells, '/map/obstacles', 10)
        self._cp_pub = self.create_publisher(PoseArray, '/map/checkpoints', 10)

        # Publish at 1 Hz so late-joining subscribers always get data.
        self.create_timer(1.0, self._publish)

        self.get_logger().info(
            f'Loaded {len(self._obstacles)} obstacle(s) and '
            f'{len(self._checkpoints)} checkpoint(s) '
            f'(cell_size={self._cell_size} m)')

    # ------------------------------------------------------------------

    def _load_obstacles(self, path: str):
        obstacles = []
        with open(path, newline='') as f:
            for row in csv.DictReader(f):
                obstacles.append((int(row['grid_x']), int(row['grid_y'])))
        return obstacles

    def _load_checkpoints(self, path: str):
        checkpoints = []
        with open(path, newline='') as f:
            rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
        for row in rows:
            checkpoints.append((int(row['grid_x']), int(row['grid_y'])))
        return checkpoints

    def _grid_to_world(self, gx: int, gy: int):
        """Return (x, y) world coordinates at the centre of grid cell (gx, gy)."""
        cs = self._cell_size
        return (gx + 0.5) * cs, (gy + 0.5) * cs

    # ------------------------------------------------------------------

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='map')

        # --- obstacles ---
        gc = GridCells()
        gc.header = header
        gc.cell_width = self._cell_size
        gc.cell_height = self._cell_size
        for gx, gy in self._obstacles:
            wx, wy = self._grid_to_world(gx, gy)
            gc.cells.append(Point(x=wx, y=wy, z=0.0))
        self._obs_pub.publish(gc)

        # --- checkpoints ---
        pa = PoseArray()
        pa.header = header
        for gx, gy in self._checkpoints:
            wx, wy = self._grid_to_world(gx, gy)
            pose = Pose()
            pose.position.x = wx
            pose.position.y = wy
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            pa.poses.append(pose)
        self._cp_pub.publish(pa)


def main(args=None):
    rclpy.init(args=args)
    node = MapManager()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
