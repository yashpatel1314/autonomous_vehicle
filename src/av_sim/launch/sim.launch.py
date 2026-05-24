"""sim.launch.py — single entry-point for the AV simulation.

Execution order:
  1. Read obstacles + checkpoints CSVs (paths come from launch args).
  2. Generate a complete Gazebo Fortress world SDF and write to /tmp.
  3. Launch Ignition Gazebo with that world.
  4. Publish the URDF via robot_state_publisher.
  5. Publish a static map→odom identity transform (perfect odometry assumption).
  6. Spawn the robot into Gazebo.
  7. Start ros_gz_bridge for cmd_vel / odom / scan / clock.
  8. Start map_manager, astar_planner, controller, scan_occupancy nodes.
  9. Start RViz2 (skipped when no_rviz:=true).

Launch arguments:
  obstacles_csv   — path to obstacles CSV  (default: package config/obstacles.csv)
  checkpoints_csv — path to checkpoints CSV (default: package config/checkpoints.csv)
  no_rviz         — set to 'true' to skip RViz2 (useful headless / in CI)

Grid / world parameters (change here to experiment):
  CELL_SIZE = 1.0 m   — physical size of one grid cell
  GRID_W    = 20      — number of columns
  GRID_H    = 20      — number of rows
  Robot spawns at grid (1, 1) → world (1.5, 1.5), yaw = 0.
"""

import csv
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ------------------------------------------------------------------
# Grid / world constants
# ------------------------------------------------------------------
CELL_SIZE = 1.0
GRID_W = 20
GRID_H = 20

ROBOT_SPAWN_X = 1.5
ROBOT_SPAWN_Y = 1.5
ROBOT_SPAWN_Z = 0.175

WORLD_NAME = 'sim_world'
WORLD_TMP_PATH = '/tmp/av_sim_world.sdf'


# ------------------------------------------------------------------
# Helpers
# ------------------------------------------------------------------

def _load_obstacles(path: str):
    result = []
    with open(path, newline='') as f:
        for row in csv.DictReader(f):
            result.append((int(row['grid_x']), int(row['grid_y'])))
    return result


def _load_checkpoints(path: str):
    with open(path, newline='') as f:
        rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
    return [(int(r['grid_x']), int(r['grid_y'])) for r in rows]


def _cell_centre(gx, gy):
    return (gx + 0.5) * CELL_SIZE, (gy + 0.5) * CELL_SIZE


def _obstacle_sdf(gx, gy):
    x, y = _cell_centre(gx, gy)
    sz = CELL_SIZE * 0.85
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
    <plugin filename="ignition-gazebo-sensors-system"
            name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

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

def _setup(context, pkg_share):
    obstacles_csv = context.launch_configurations['obstacles_csv']
    checkpoints_csv = context.launch_configurations['checkpoints_csv']

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_cfg = os.path.join(pkg_share, 'rviz', 'av_sim.rviz')

    obstacles = _load_obstacles(obstacles_csv)
    checkpoints = _load_checkpoints(checkpoints_csv)
    world_sdf = _build_world_sdf(obstacles, checkpoints)
    with open(WORLD_TMP_PATH, 'w') as f:
        f.write(world_sdf)

    with open(urdf_path, 'r') as f:
        robot_desc = f.read()

    # Gazebo Fortress — call ign CLI directly; ros_ign_gazebo does not ship
    # an ign_gazebo wrapper binary (ign is provided by ignition-fortress apt pkg)
    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', WORLD_TMP_PATH, '-r'],
        output='screen',
    )

    # Robot state publisher (broadcasts TF from URDF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )

    # Static map→odom identity transform (perfect odometry assumption)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--frame-id', 'map',
            '--child-frame-id', 'odom',
        ],
        output='screen',
    )

    # Spawn robot (waits 3 s for Gazebo to initialise)
    spawn = TimerAction(
        period=3.0,
        actions=[Node(
            package='ros_ign_gazebo',
            executable='create',
            arguments=[
                '-name', 'robot',
                '-file', urdf_path,
                '-x', str(ROBOT_SPAWN_X),
                '-y', str(ROBOT_SPAWN_Y),
                '-z', str(ROBOT_SPAWN_Z),
                '-R', '0', '-P', '0', '-Y', '0',
            ],
            output='screen',
        )],
    )

    # ros_ign_bridge: cmd_vel (ROS→IGN), odom/clock/scan (IGN→ROS)
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
        ],
        output='screen',
    )

    # Application nodes (start 5 s after launch so Gazebo is ready)
    app_nodes = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='av_sim',
                executable='map_manager',
                parameters=[{
                    'obstacles_csv': obstacles_csv,
                    'checkpoints_csv': checkpoints_csv,
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
                    'inflation_radius': 1,
                }],
                output='screen',
            ),
            Node(
                package='av_sim',
                executable='controller',
                output='screen',
            ),
            Node(
                package='av_sim',
                executable='scan_occupancy',
                parameters=[{
                    'grid_width': GRID_W,
                    'grid_height': GRID_H,
                    'cell_size': CELL_SIZE,
                }],
                output='screen',
            ),
        ],
    )

    # RViz2 for live visualization (skipped when no_rviz:=true)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('no_rviz')),
    )

    return [gz_sim, rsp, static_tf, spawn, bridge, app_nodes, rviz]


def generate_launch_description():
    pkg_share = get_package_share_directory('av_sim')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacles_csv',
            default_value=os.path.join(config_dir, 'obstacles.csv'),
            description='Absolute path to the obstacles CSV file',
        ),
        DeclareLaunchArgument(
            'checkpoints_csv',
            default_value=os.path.join(config_dir, 'checkpoints.csv'),
            description='Absolute path to the checkpoints CSV file',
        ),
        DeclareLaunchArgument(
            'no_rviz',
            default_value='false',
            description='Set to true to skip launching RViz2',
        ),
        OpaqueFunction(function=lambda ctx: _setup(ctx, pkg_share)),
    ])
