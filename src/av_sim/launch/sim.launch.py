"""
sim.launch.py — single entry-point for the AV simulation.

Execution order:
  1. Read obstacles.csv and checkpoints.csv from the installed config/ dir.
  2. Generate a complete Gazebo Fortress world SDF (ground + obstacles + checkpoints)
     and write it to /tmp/av_sim_world.sdf.
  3. Launch Ignition Gazebo with that world.
  4. Publish the URDF via robot_state_publisher.
  5. Spawn the robot into Gazebo.
  6. Start ros_gz_bridge for cmd_vel / odom / clock.
  7. Start map_manager, astar_planner, and controller nodes.

Grid / world parameters (change here to experiment):
  CELL_SIZE   = 1.0 m   — physical size of one grid cell
  GRID_W      = 20      — number of columns
  GRID_H      = 20      — number of rows
  Robot spawns at grid (1, 1) → world (1.5, 1.5), yaw = 0.
"""

import csv
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction, TimerAction
from launch_ros.actions import Node

# ------------------------------------------------------------------
# Grid / world constants
# ------------------------------------------------------------------
CELL_SIZE = 1.0   # metres per grid cell
GRID_W = 20       # columns
GRID_H = 20       # rows

ROBOT_SPAWN_X = 1.5   # world x  (grid 1,1 centre)
ROBOT_SPAWN_Y = 1.5   # world y
ROBOT_SPAWN_Z = 0.175 # base_link height above ground

WORLD_NAME = 'sim_world'
WORLD_TMP_PATH = '/tmp/av_sim_world.sdf'


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _load_obstacles(config_dir: str):
    path = os.path.join(config_dir, 'obstacles.csv')
    result = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            result.append((int(row['grid_x']), int(row['grid_y'])))
    return result


def _load_checkpoints(config_dir: str):
    path = os.path.join(config_dir, 'checkpoints.csv')
    with open(path, newline='') as f:
        rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
    return [(int(r['grid_x']), int(r['grid_y'])) for r in rows]


def _cell_centre(gx, gy):
    return (gx + 0.5) * CELL_SIZE, (gy + 0.5) * CELL_SIZE


def _obstacle_sdf(gx, gy):
    x, y = _cell_centre(gx, gy)
    sz = CELL_SIZE * 0.85   # slightly smaller than cell so corners are visible
    return f"""\
    <model name="obstacle_{gx}_{gy}">
      <static>true</static>
      <pose>{x} {y} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>{sz} {sz} 1.0</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{sz} {sz} 1.0</size></box></geometry>
          <material>
            <ambient>0.8 0.2 0.2 1</ambient>
            <diffuse>0.8 0.2 0.2 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def _checkpoint_sdf(order, gx, gy):
    x, y = _cell_centre(gx, gy)
    return f"""\
    <model name="checkpoint_{order}">
      <static>true</static>
      <pose>{x} {y} 0.1 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><cylinder><radius>0.25</radius><length>0.2</length></cylinder></geometry>
        </collision>
        <visual name="vis">
          <geometry><cylinder><radius>0.25</radius><length>0.2</length></cylinder></geometry>
          <material>
            <ambient>0.1 0.8 0.1 1</ambient>
            <diffuse>0.1 0.8 0.1 1</diffuse>
            <specular>0.2 0.8 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>"""


def _build_world_sdf(obstacles, checkpoints) -> str:
    obs_models = '\n'.join(_obstacle_sdf(gx, gy) for gx, gy in obstacles)
    cp_models = '\n'.join(
        _checkpoint_sdf(i + 1, gx, gy)
        for i, (gx, gy) in enumerate(checkpoints)
    )
    return f"""\
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="{WORLD_NAME}">

    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Required Fortress system plugins -->
    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse>
      <specular>0.5 0.5 0.5 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{obs_models}

{cp_models}

  </world>
</sdf>
"""


# ------------------------------------------------------------------
# Launch entry-point
# ------------------------------------------------------------------

def generate_launch_description():
    pkg_share = get_package_share_directory('av_sim')
    config_dir = os.path.join(pkg_share, 'config')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    def setup(context):
        # 1. Load CSV data and write the generated world file.
        obstacles = _load_obstacles(config_dir)
        checkpoints = _load_checkpoints(config_dir)
        world_sdf = _build_world_sdf(obstacles, checkpoints)
        with open(WORLD_TMP_PATH, 'w') as f:
            f.write(world_sdf)

        with open(urdf_path, 'r') as f:
            robot_desc = f.read()

        # 2. Gazebo Fortress
        gz_sim = Node(
            package='ros_gz_sim',
            executable='gz_sim',
            arguments=[WORLD_TMP_PATH, '-r'],
            output='screen',
        )

        # 3. Robot state publisher (broadcasts TF from URDF)
        rsp = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen',
        )

        # 4. Spawn robot (waits 3 s for Gazebo to initialise)
        spawn = TimerAction(
            period=3.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', 'robot',
                    '-string', robot_desc,
                    '-x', str(ROBOT_SPAWN_X),
                    '-y', str(ROBOT_SPAWN_Y),
                    '-z', str(ROBOT_SPAWN_Z),
                    '-R', '0', '-P', '0', '-Y', '0',
                ],
                output='screen',
            )],
        )

        # 5. ros_gz_bridge: cmd_vel (ROS→IGN), odom (IGN→ROS), clock (IGN→ROS)
        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
                f'/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
                f'/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            ],
            output='screen',
        )

        # 6. Application nodes (start 5 s after launch so Gazebo is ready)
        app_nodes = TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='av_sim',
                    executable='map_manager',
                    parameters=[{
                        'config_dir': config_dir,
                        'cell_size': CELL_SIZE,
                    }],
                    output='screen',
                ),
                Node(
                    package='av_sim',
                    executable='astar_planner',
                    parameters=[{
                        'grid_width': GRID_W,
                        'grid_height': GRID_H,
                        'cell_size': CELL_SIZE,
                    }],
                    output='screen',
                ),
                Node(
                    package='av_sim',
                    executable='controller',
                    output='screen',
                ),
            ],
        )

        return [gz_sim, rsp, spawn, bridge, app_nodes]

    return LaunchDescription([OpaqueFunction(function=setup)])
