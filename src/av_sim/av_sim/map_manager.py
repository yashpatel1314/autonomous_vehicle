#!/usr/bin/env python3
"""Map Manager node.

Reads obstacles.csv and checkpoints.csv from explicit file paths (ROS params),
then publishes their contents as ROS2 topics for the planner to consume.

Parameters:
  obstacles_csv   — absolute path to obstacles CSV
  checkpoints_csv — absolute path to checkpoints CSV
  cell_size       — metres per grid cell (default 1.0)

Topics published:
  /map/obstacles    nav_msgs/GridCells   — one cell per obstacle row
  /map/checkpoints  geometry_msgs/PoseArray — ordered checkpoint poses
"""

import csv

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, PoseArray
from nav_msgs.msg import GridCells
from std_msgs.msg import Header


class MapManager(Node):

    def __init__(self):
        super().__init__('map_manager')

        self.declare_parameter('obstacles_csv', '')
        self.declare_parameter('checkpoints_csv', '')
        self.declare_parameter('cell_size', 1.0)

        obs_path = self.get_parameter('obstacles_csv').get_parameter_value().string_value
        cp_path = self.get_parameter('checkpoints_csv').get_parameter_value().string_value
        self._cell_size = self.get_parameter('cell_size').get_parameter_value().double_value

        self._obstacles = self._load_obstacles(obs_path)
        self._checkpoints = self._load_checkpoints(cp_path)

        self._obs_pub = self.create_publisher(GridCells, '/map/obstacles', 10)
        self._cp_pub = self.create_publisher(PoseArray, '/map/checkpoints', 10)

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
        with open(path, newline='') as f:
            rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
        return [(int(r['grid_x']), int(r['grid_y'])) for r in rows]

    def _grid_to_world(self, gx: int, gy: int):
        cs = self._cell_size
        return (gx + 0.5) * cs, (gy + 0.5) * cs

    # ------------------------------------------------------------------

    def _publish(self):
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id='map')

        gc = GridCells()
        gc.header = header
        gc.cell_width = self._cell_size
        gc.cell_height = self._cell_size
        for gx, gy in self._obstacles:
            wx, wy = self._grid_to_world(gx, gy)
            gc.cells.append(Point(x=wx, y=wy, z=0.0))
        self._obs_pub.publish(gc)

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
