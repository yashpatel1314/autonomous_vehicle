#!/usr/bin/env python3
"""
A* Planner node.

Subscribes to the obstacle grid and ordered checkpoint list, then computes
an optimal path through all checkpoints in sequence (avoiding obstacles) and
publishes it as a nav_msgs/Path.

Topics subscribed:
  /map/obstacles    nav_msgs/GridCells
  /map/checkpoints  geometry_msgs/PoseArray
  /odom             nav_msgs/Odometry  (robot start position)

Topic published:
  /planned_path     nav_msgs/Path
"""

import heapq
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import GridCells, Odometry, Path
from geometry_msgs.msg import PoseArray


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('grid_width', 20)
        self.declare_parameter('grid_height', 20)
        self.declare_parameter('cell_size', 1.0)

        self._gw = self.get_parameter('grid_width').get_parameter_value().integer_value
        self._gh = self.get_parameter('grid_height').get_parameter_value().integer_value
        self._cs = self.get_parameter('cell_size').get_parameter_value().double_value

        self._obstacles: set = set()
        self._checkpoints: list = []
        self._start_grid = None        # (gx, gy) set once from first odom
        self._path_published = False

        self.create_subscription(GridCells, '/map/obstacles', self._cb_obs, 10)
        self.create_subscription(PoseArray, '/map/checkpoints', self._cb_cp, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)

        self._path_pub = self.create_publisher(Path, '/planned_path', 10)

    # ------------------------------------------------------------------
    # Callbacks

    def _cb_obs(self, msg: GridCells):
        cs = self._cs
        self._obstacles = {
            (round(p.x / cs - 0.5), round(p.y / cs - 0.5))
            for p in msg.cells
        }
        self._try_plan()

    def _cb_cp(self, msg: PoseArray):
        cs = self._cs
        self._checkpoints = [
            (round(p.position.x / cs - 0.5), round(p.position.y / cs - 0.5))
            for p in msg.poses
        ]
        self._try_plan()

    def _cb_odom(self, msg: Odometry):
        if self._start_grid is not None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cs = self._cs
        self._start_grid = (int(x / cs), int(y / cs))
        self.get_logger().info(f'Robot start grid: {self._start_grid}')
        self._try_plan()

    # ------------------------------------------------------------------
    # Planning

    def _try_plan(self):
        if self._path_published:
            return
        if not self._obstacles and not self._checkpoints:
            return
        if not self._checkpoints:
            return
        if self._start_grid is None:
            return

        waypoints = [self._start_grid] + self._checkpoints
        full_path: list = []

        for i in range(len(waypoints) - 1):
            segment = self._astar(waypoints[i], waypoints[i + 1])
            if segment is None:
                self.get_logger().error(
                    f'No path from {waypoints[i]} to {waypoints[i+1]}')
                return
            # Avoid duplicating the junction node between segments.
            if full_path:
                segment = segment[1:]
            full_path.extend(segment)

        ros_path = self._grid_path_to_ros(full_path)
        self._path_pub.publish(ros_path)
        self._path_published = True
        self.get_logger().info(
            f'Path published: {len(full_path)} waypoints through '
            f'{len(self._checkpoints)} checkpoint(s)')

    def _astar(self, start, goal):
        """8-directional A* on the grid. Returns list of (gx, gy) or None."""
        if start == goal:
            return [start]

        MOVES = [
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
        ]

        def h(n):
            return math.hypot(goal[0] - n[0], goal[1] - n[1])

        open_heap = [(h(start), start)]
        came_from = {}
        g = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return list(reversed(path))

            for dx, dy, cost in MOVES:
                nb = (current[0] + dx, current[1] + dy)
                if not (0 <= nb[0] < self._gw and 0 <= nb[1] < self._gh):
                    continue
                if nb in self._obstacles:
                    continue
                ng = g[current] + cost
                if ng < g.get(nb, float('inf')):
                    came_from[nb] = current
                    g[nb] = ng
                    heapq.heappush(open_heap, (ng + h(nb), nb))

        return None

    def _grid_path_to_ros(self, grid_path) -> Path:
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        cs = self._cs
        for gx, gy in grid_path:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = (gx + 0.5) * cs
            ps.pose.position.y = (gy + 0.5) * cs
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        return path_msg


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
