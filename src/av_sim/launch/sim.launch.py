"""sim.launch.py — clean entry-point for the AV serpentine-maze simulation.

Execution order:
  1. Parse obstacles.csv and checkpoints.csv.
  2. Generate an SDF world and write to /tmp.
  3. Launch Ignition Gazebo (ign CLI).
  4. Publish URDF via robot_state_publisher.
  5. Publish static map→odom identity TF.
  6. Spawn robot after 3 s.
  7. ros_ign_bridge: cmd_vel, odom, clock  (NO /scan — no GPU sensors).
  8. Start map_manager, astar_planner, controller after 5 s.
  9. Optionally start RViz2.

Rendering is stable under Mesa software renderer because:
  - No ignition-gazebo-sensors-system plugin  →  no GPU off-screen render context
  - cast_shadows false                        →  no shadow-map recomputation
  - max_step_size 0.004 (250 Hz)             →  lower scene-update pressure
  - render_rate 60                            →  capped Ogre GUI frame rate
  - engine ogre                               →  Ogre1 works with Mesa; Ogre2 needs GL 3.3

Launch args:
  obstacles_csv   — default: package config/obstacles.csv
  checkpoints_csv — default: package config/checkpoints.csv
  headless        — default true  (Gazebo server-only, no GUI window)
  no_rviz         — default false (RViz2 is the primary visualization)
"""

import csv
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                             OpaqueFunction, TimerAction)
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Grid constants ────────────────────────────────────────────────────────────
CELL_SIZE = 1.0
GRID_W    = 20
GRID_H    = 20

ROBOT_SPAWN_X = 0.5
ROBOT_SPAWN_Y = 0.5
ROBOT_SPAWN_Z = 0.175

WORLD_TMP_PATH = '/tmp/av_sim_world.sdf'


# ── CSV helpers ───────────────────────────────────────────────────────────────

def _load_obstacles(path: str):
    with open(path, newline='') as f:
        return [(int(r['grid_x']), int(r['grid_y'])) for r in csv.DictReader(f)]


def _load_checkpoints(path: str):
    with open(path, newline='') as f:
        rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
    return [(int(r['grid_x']), int(r['grid_y'])) for r in rows]


def _cell_centre(gx, gy):
    return (gx + 0.5) * CELL_SIZE, (gy + 0.5) * CELL_SIZE


# ── SDF snippets ──────────────────────────────────────────────────────────────

def _wall_sdf(gx, gy):
    x, y = _cell_centre(gx, gy)
    sz = CELL_SIZE * 0.70
    return f"""\
    <model name="wall_{gx}_{gy}">
      <static>true</static>
      <pose>{x} {y} 0.5 0 0 0</pose>
      <link name="link">
        <collision name="col">
          <geometry><box><size>{sz} {sz} 1.0</size></box></geometry>
        </collision>
        <visual name="vis">
          <geometry><box><size>{sz} {sz} 1.0</size></box></geometry>
          <material>
            <ambient>0.7 0.15 0.15 1</ambient>
            <diffuse>0.7 0.15 0.15 1</diffuse>
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
        <visual name="vis">
          <geometry><cylinder><radius>0.3</radius><length>0.2</length></cylinder></geometry>
          <material>
            <ambient>0.1 0.8 0.1 1</ambient>
            <diffuse>0.1 0.8 0.1 1</diffuse>
          </material>
        </visual>
      </link>
    </model>"""


def _build_world_sdf(obstacles, checkpoints) -> str:
    walls = '\n'.join(_wall_sdf(gx, gy) for gx, gy in obstacles)
    cps   = '\n'.join(_checkpoint_sdf(i + 1, gx, gy)
                      for i, (gx, gy) in enumerate(checkpoints))
    return f"""\
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="av_sim">

    <physics name="250hz" type="ignored">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="ignition-gazebo-physics-system"
            name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system"
            name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
            name="ignition::gazebo::systems::SceneBroadcaster"/>

    <gui fullscreen="0">
      <plugin filename="MinimalScene" name="3D View">
        <ignition-gui>
          <title>3D View</title>
          <property type="bool" key="showTitleBar">false</property>
          <property type="bool" key="resizable">false</property>
          <property type="double" key="z">0</property>
          <property type="string" key="state">docked</property>
        </ignition-gui>
        <engine>ogre</engine>
        <scene>scene</scene>
        <ambient_light>0.5 0.5 0.5</ambient_light>
        <background_color>0.2 0.2 0.3</background_color>
        <camera_pose>10 -4 28 0 0.75 1.5708</camera_pose>
        <render_rate>60</render_rate>
      </plugin>
      <plugin filename="GzSceneManager" name="Scene Manager">
        <ignition-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width"     type="double">5</property>
          <property key="height"    type="double">5</property>
          <property key="state"     type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
      <plugin filename="InteractiveViewControl" name="Interactive view control">
        <ignition-gui>
          <property key="resizable" type="bool">false</property>
          <property key="width"     type="double">5</property>
          <property key="height"    type="double">5</property>
          <property key="state"     type="string">floating</property>
          <property key="showTitleBar" type="bool">false</property>
        </ignition-gui>
      </plugin>
    </gui>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="col">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
        </collision>
        <visual name="vis">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material>
            <ambient>0.6 0.6 0.6 1</ambient>
            <diffuse>0.6 0.6 0.6 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

{walls}

{cps}

  </world>
</sdf>
"""


# ── Launch ────────────────────────────────────────────────────────────────────

def _setup(context, pkg_share):
    obstacles_csv   = context.launch_configurations['obstacles_csv']
    checkpoints_csv = context.launch_configurations['checkpoints_csv']
    headless = context.launch_configurations.get('headless', 'false').lower() == 'true'

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    rviz_cfg  = os.path.join(pkg_share, 'rviz', 'av_sim.rviz')

    obstacles   = _load_obstacles(obstacles_csv)
    checkpoints = _load_checkpoints(checkpoints_csv)

    with open(WORLD_TMP_PATH, 'w') as f:
        f.write(_build_world_sdf(obstacles, checkpoints))

    with open(urdf_path) as f:
        robot_desc = f.read()

    gz_cmd = ['ign', 'gazebo', '-s', WORLD_TMP_PATH, '-r'] if headless \
             else ['ign', 'gazebo', WORLD_TMP_PATH, '-r']
    gz = ExecuteProcess(cmd=gz_cmd, output='screen')

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
        )
    ])

    # No /scan bridge — no GPU sensor off-screen rendering
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen',
    )

    app = TimerAction(period=5.0, actions=[
        Node(
            package='av_sim',
            executable='map_manager',
            parameters=[{
                'obstacles_csv':   obstacles_csv,
                'checkpoints_csv': checkpoints_csv,
                'cell_size':       CELL_SIZE,
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
                'inflation_radius': 1,
            }],
            output='screen',
        ),
        Node(
            package='av_sim',
            executable='controller',
            output='screen',
        ),
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
        condition=UnlessCondition(LaunchConfiguration('no_rviz')),
    )

    return [gz, rsp, static_tf, spawn, bridge, app, rviz]


def generate_launch_description():
    pkg_share  = get_package_share_directory('av_sim')
    config_dir = os.path.join(pkg_share, 'config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'obstacles_csv',
            default_value=os.path.join(config_dir, 'obstacles.csv'),
            description='Path to obstacles CSV',
        ),
        DeclareLaunchArgument(
            'checkpoints_csv',
            default_value=os.path.join(config_dir, 'checkpoints.csv'),
            description='Path to checkpoints CSV',
        ),
        DeclareLaunchArgument(
            'no_rviz',
            default_value='false',
            description='Set true to suppress RViz2 (CI / headless-only runs)',
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='true',
            description='Run Gazebo server-only (no GUI window) — default on',
        ),
        OpaqueFunction(function=lambda ctx: _setup(ctx, pkg_share)),
    ])
