#!/usr/bin/env python3
"""A* Planner node.

Subscribes to the static obstacle grid, live scan obstacles, and ordered
checkpoint list.  Computes an inflated, pruned path through all *remaining*
checkpoints (those not yet reached) starting from the robot's *current*
position.  Re-publishes whenever inputs change or a replan is forced.

Checkpoint progress is tracked via /current_checkpoint so that after a
replan the robot is not re-routed through already-completed checkpoints.

Topics subscribed:
  /map/obstacles       nav_msgs/GridCells
  /map/scan_obstacles  nav_msgs/GridCells
  /map/checkpoints     geometry_msgs/PoseArray
  /odom                nav_msgs/Odometry       (live robot position)
  /current_checkpoint  std_msgs/Int32          (how many checkpoints reached)
  /replan_request      std_msgs/Empty

Topic published:
  /planned_path        nav_msgs/Path
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import GridCells, Odometry, Path
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Empty, Int32

from av_sim.planning import astar, inflate_obstacles, prune_path


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('grid_width', 20)
        self.declare_parameter('grid_height', 20)
        self.declare_parameter('cell_size', 1.0)
        self.declare_parameter('inflation_radius', 1)

        self._gw = self.get_parameter('grid_width').get_parameter_value().integer_value
        self._gh = self.get_parameter('grid_height').get_parameter_value().integer_value
        self._cs = self.get_parameter('cell_size').get_parameter_value().double_value
        self._ir = self.get_parameter('inflation_radius').get_parameter_value().integer_value

        self._static_obs: set = set()
        self._scan_obs: set = set()
        self._checkpoints: list = []   # all checkpoints in order
        self._current_grid = None      # live robot grid position, updated every odom
        self._reached = 0              # number of checkpoints already completed

        self._last_obs_key: frozenset = frozenset()
        self._last_cp_key: tuple = ()
        self._last_reached: int = -1

        self.create_subscription(GridCells, '/map/obstacles', self._cb_static_obs, 10)
        self.create_subscription(GridCells, '/map/scan_obstacles', self._cb_scan_obs, 10)
        self.create_subscription(PoseArray, '/map/checkpoints', self._cb_cp, 10)
        self.create_subscription(Odometry, '/odom', self._cb_odom, 10)
        self.create_subscription(Int32, '/current_checkpoint', self._cb_checkpoint, 10)
        self.create_subscription(Empty, '/replan_request', self._cb_replan, 10)

        self._path_pub = self.create_publisher(Path, '/planned_path', 10)

    # ------------------------------------------------------------------
    # Callbacks

    def _cb_static_obs(self, msg: GridCells):
        cs = self._cs
        self._static_obs = {
            (round(p.x / cs - 0.5), round(p.y / cs - 0.5))
            for p in msg.cells
        }
        self._try_plan()

    def _cb_scan_obs(self, msg: GridCells):
        cs = self._cs
        self._scan_obs = {
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
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        cs = self._cs
        new_grid = (int(x / cs), int(y / cs))
        first = self._current_grid is None
        self._current_grid = new_grid
        if first:
            self.get_logger().info(f'Robot start grid: {self._current_grid}')
            self._try_plan()

    def _cb_checkpoint(self, msg: Int32):
        self._reached = msg.data
        # Force a replan from current position through remaining checkpoints.
        self._last_reached = -1
        self._try_plan()

    def _cb_replan(self, _msg: Empty):
        self._last_obs_key = frozenset()
        self._last_cp_key = ()
        self._last_reached = -1
        self._try_plan()

    # ------------------------------------------------------------------
    # Planning

    def _try_plan(self):
        remaining = self._checkpoints[self._reached:]
        if not remaining or self._current_grid is None:
            return

        combined = self._static_obs | self._scan_obs
        obs_key = frozenset(combined)
        cp_key = tuple(remaining)
        if (obs_key == self._last_obs_key
                and cp_key == self._last_cp_key
                and self._reached == self._last_reached):
            return

        # Inflate ONLY static obstacles. Adding scan cells to the inflated set
        # can seal narrow gate passages when lidar noise falls in the gap region.
        # Scan obstacles are still respected as point obstacles (no inflation).
        static_inflated = inflate_obstacles(
            self._static_obs, self._gw, self._gh, self._ir)

        waypoints = [self._current_grid] + list(remaining)
        full_path: list = []

        for i in range(len(waypoints) - 1):
            seg_start = waypoints[i]
            seg_goal = waypoints[i + 1]

            inflated = (static_inflated | self._scan_obs) - {seg_start, seg_goal}
            segment = astar(seg_start, seg_goal, inflated, self._gw, self._gh)
            if segment is None:
                # Scan obstacles blocking — retry with static inflation only
                static_inf = static_inflated - {seg_start, seg_goal}
                segment = astar(seg_start, seg_goal, static_inf, self._gw, self._gh)
            if segment is None:
                segment = astar(
                    seg_start, seg_goal, self._static_obs, self._gw, self._gh)
            if segment is None:
                self.get_logger().error(
                    f'No path at all from {seg_start} to {seg_goal}')
                return
            if full_path:
                segment = segment[1:]
            full_path.extend(segment)

        pruned = prune_path(full_path)

        self._last_obs_key = obs_key
        self._last_cp_key = cp_key
        self._last_reached = self._reached

        ros_path = self._grid_path_to_ros(pruned)
        self._path_pub.publish(ros_path)
        self.get_logger().info(
            f'Path published: {len(full_path)} raw → {len(pruned)} pruned waypoints '
            f'(start={self._current_grid}, remaining checkpoints={len(remaining)})')

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