#!/usr/bin/env python3
"""A* Planner node.

Subscribes to the static obstacle grid and ordered checkpoint list.  On each
change (new obstacles, new checkpoints, checkpoint reached, or forced replan)
it computes an inflated path from the robot's current position through all
remaining checkpoints and publishes it on /planned_path.

There is no lidar/scan layer.  All obstacle avoidance is based on the static
grid.  This eliminates GPU sensor rendering and makes the plan deterministic.

Topics subscribed:
  /map/obstacles      nav_msgs/GridCells
  /map/checkpoints    geometry_msgs/PoseArray
  /odom               nav_msgs/Odometry
  /current_checkpoint std_msgs/Int32
  /replan_request     std_msgs/Empty

Topic published:
  /planned_path       nav_msgs/Path
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import GridCells, Odometry, Path
from std_msgs.msg import Empty, Int32

from av_sim.planning import astar, inflate_obstacles, prune_path


class AStarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('grid_width',       20)
        self.declare_parameter('grid_height',      20)
        self.declare_parameter('cell_size',         1.0)
        self.declare_parameter('inflation_radius',  1)

        self._gw = self.get_parameter('grid_width').get_parameter_value().integer_value
        self._gh = self.get_parameter('grid_height').get_parameter_value().integer_value
        self._cs = self.get_parameter('cell_size').get_parameter_value().double_value
        self._ir = self.get_parameter('inflation_radius').get_parameter_value().integer_value

        self._static_obs: set  = set()
        self._checkpoints: list = []
        self._current_grid      = None   # live robot grid cell (gx, gy)
        self._reached           = 0      # checkpoints already completed

        # Change-detection keys to avoid redundant replanning
        self._last_obs_key:    frozenset = frozenset()
        self._last_cp_key:     tuple     = ()
        self._last_reached:    int       = -1

        self.create_subscription(GridCells, '/map/obstacles',      self._cb_obs,        10)
        self.create_subscription(PoseArray, '/map/checkpoints',    self._cb_cp,         10)
        self.create_subscription(Odometry,  '/odom',               self._cb_odom,       10)
        self.create_subscription(Int32,     '/current_checkpoint', self._cb_checkpoint, 10)
        self.create_subscription(Empty,     '/replan_request',     self._cb_replan,     10)

        self._path_pub = self.create_publisher(Path, '/planned_path', 10)

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_obs(self, msg: GridCells):
        cs = self._cs
        self._static_obs = {
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
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        cs = self._cs
        new_grid = (int(x / cs), int(y / cs))
        first = self._current_grid is None
        self._current_grid = new_grid
        if first:
            self.get_logger().info(f'Robot start grid: {self._current_grid}')
            self._try_plan()

    def _cb_checkpoint(self, msg: Int32):
        self._reached = msg.data
        self._last_reached = -1   # force replan from new position
        self._try_plan()

    def _cb_replan(self, _msg: Empty):
        self._last_obs_key = frozenset()
        self._last_cp_key  = ()
        self._last_reached = -1
        self._try_plan()

    # ── Planning ──────────────────────────────────────────────────────────────

    def _try_plan(self):
        remaining = self._checkpoints[self._reached:]
        if not remaining or self._current_grid is None:
            return

        obs_key = frozenset(self._static_obs)
        cp_key  = tuple(remaining)
        if (obs_key == self._last_obs_key
                and cp_key == self._last_cp_key
                and self._reached == self._last_reached):
            return

        inflated = inflate_obstacles(self._static_obs, self._gw, self._gh, self._ir)

        waypoints = [self._current_grid] + list(remaining)
        full_path: list = []

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            goal  = waypoints[i + 1]

            # Try inflated obstacles first (better clearance)
            seg = astar(start, goal,
                        inflated - {start, goal},
                        self._gw, self._gh)

            # Fall back to raw obstacles if inflation blocks the corridor
            if seg is None:
                seg = astar(start, goal,
                            self._static_obs - {start, goal},
                            self._gw, self._gh)

            if seg is None:
                self.get_logger().error(f'No path from {start} to {goal}')
                return

            full_path.extend(seg if not full_path else seg[1:])

        pruned = prune_path(full_path)

        self._last_obs_key = obs_key
        self._last_cp_key  = cp_key
        self._last_reached = self._reached

        self._path_pub.publish(self._to_ros_path(pruned))
        self.get_logger().info(
            f'Path: {len(full_path)} cells → {len(pruned)} pruned  '
            f'start={self._current_grid}  remaining={len(remaining)}')

    def _to_ros_path(self, grid_path) -> Path:
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        cs = self._cs
        for gx, gy in grid_path:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x  = (gx + 0.5) * cs
            ps.pose.position.y  = (gy + 0.5) * cs
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
