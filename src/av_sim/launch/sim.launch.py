"""sim.launch.py — Gazebo Classic 11 launch for AV simulation.

Execution order:
  1. Parse obstacles.csv and checkpoints.csv.
  2. Generate SDF world file and write to /tmp.
  3. Launch gzserver (physics) + gzclient (GUI, unless headless:=true).
  4. robot_state_publisher — publishes /robot_description and /tf topic.
  5. static_transform_publisher — map → odom identity TF.
  6. spawn_entity.py after 3 s — drops robot into Gazebo.
  7. map_manager, astar_planner, controller after 5 s.

Gazebo Classic rendering is stable under Mesa software rendering because:
  - Uses Ogre 1.x (not Ogre 2), which does not require GL 3.3 deferred shading
  - No GzSceneManager / SceneBroadcaster interaction (Ignition-only)
  - CPU ray sensor — no off-screen GPU framebuffer

Launch args:
  obstacles_csv   — default: package config/obstacles.csv
  checkpoints_csv — default: package config/checkpoints.csv
  headless        — default false  (set true for CI / no display)
"""

import csv
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CELL_SIZE    = 1.0
GRID_W       = 30
GRID_H       = 30
SPAWN_X      = 0.5
SPAWN_Y      = 0.5
SPAWN_Z      = 0.175
WORLD_PATH   = '/tmp/av_sim_world.sdf'


# ── CSV helpers ───────────────────────────────────────────────────────────────

def _load_obstacles(path):
    with open(path, newline='') as f:
        return [(int(r['grid_x']), int(r['grid_y'])) for r in csv.DictReader(f)]


def _load_checkpoints(path):
    with open(path, newline='') as f:
        rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
        return [(int(r['grid_x']), int(r['grid_y'])) for r in rows]


def _centre(gx, gy):
    return (gx + 0.5) * CELL_SIZE, (gy + 0.5) * CELL_SIZE


# ── SDF builders ─────────────────────────────────────────────────────────────

def _obstacle_sdf(gx, gy):
    x, y = _centre(gx, gy)
    return f"""\
    <model name="obs_{gx}_{gy}">
      <static>true</static>
      <pose>{x} {y} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>0.8 0.8 1.0</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>0.8 0.8 1.0</size></box></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def _checkpoint_sdf(order, gx, gy):
    x, y = _centre(gx, gy)
    return f"""\
    <model name="cp_{order}">
      <static>true</static>
      <pose>{x} {y} 0.1 0 0 0</pose>
      <link name="link">
        <visual name="vis">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Green</name>
            </script>
          </material>
        </visual>
      </link>
    </model>"""


def _build_world(obstacles, checkpoints):
    walls = '\n'.join(_obstacle_sdf(gx, gy) for gx, gy in obstacles)
    cps   = '\n'.join(_checkpoint_sdf(i + 1, gx, gy)
                      for i, (gx, gy) in enumerate(checkpoints))
    return f"""\
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="av_sim">

    <physics type="ode">
      <max_step_size>0.002</max_step_size>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>

    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>

{walls}

{cps}

  </world>
</sdf>
"""


# ── Launch ────────────────────────────────────────────────────────────────────

def _setup(context, pkg_share, gazebo_ros_share):
    obstacles_csv   = context.launch_configurations['obstacles_csv']
    checkpoints_csv = context.launch_configurations['checkpoints_csv']
    headless = context.launch_configurations.get('headless', 'false').lower() == 'true'

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')

    obstacles   = _load_obstacles(obstacles_csv)
    checkpoints = _load_checkpoints(checkpoints_csv)

    try:
        with open(WORLD_PATH, 'w') as f:
            f.write(_build_world(obstacles, checkpoints))
    except OSError as e:
        raise RuntimeError(f'Cannot write SDF world to {WORLD_PATH}: {e}') from e

    with open(urdf_path) as f:
        robot_desc = f.read()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world':   WORLD_PATH,
            'verbose': 'false',
            'gui':     'false' if headless else 'true',
        }.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        output='screen',
    )

    spawn = TimerAction(period=3.0, actions=[
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'robot',
                '-file',   urdf_path,
                '-x', str(SPAWN_X),
                '-y', str(SPAWN_Y),
                '-z', str(SPAWN_Z),
            ],
            output='screen',
        )
    ])

    app = TimerAction(period=5.0, actions=[
        Node(
            package='av_sim',
            executable='map_manager',
            parameters=[{
                'obstacles_csv':   obstacles_csv,
                'checkpoints_csv': checkpoints_csv,
                'cell_size':       CELL_SIZE,
                'grid_width':      GRID_W,
                'grid_height':     GRID_H,
            }],
            output='screen',
        ),
        Node(
            package='av_sim',
            executable='astar_planner',
            parameters=[{
                'grid_width':       GRID_W,
                'grid_height':      GRID_H,
                'cell_size':        CELL_SIZE,
                'inflation_radius': 2,
            }],
            output='screen',
        ),
        Node(
            package='av_sim',
            executable='controller',
            output='screen',
        ),
    ])

    return [gazebo, rsp, static_tf, spawn, app]


def generate_launch_description():
    pkg_share        = get_package_share_directory('av_sim')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    config_dir       = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacles_csv',
            default_value=os.path.join(config_dir, 'obstacles.csv'),
        ),
        DeclareLaunchArgument(
            'checkpoints_csv',
            default_value=os.path.join(config_dir, 'checkpoints.csv'),
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='true = gzserver only, no GUI window',
        ),
        OpaqueFunction(
            function=lambda ctx: _setup(ctx, pkg_share, gazebo_ros_share)
        ),
    ])
