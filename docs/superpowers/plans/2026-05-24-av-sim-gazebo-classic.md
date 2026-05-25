# AV Sim — Gazebo Classic 11 Rewrite Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Completely rewrite the AV simulation on Gazebo Classic 11 (Ogre 1.x, Mesa-stable), replacing Ignition Fortress which caused unrecoverable viewport flickering under Mesa software rendering in Docker/WSL2.

**Architecture:** Gazebo Classic 11 runs as `gzserver` + `gzclient` in Docker. The robot URDF uses `libgazebo_ros_diff_drive.so` (publishes `/odom`, subscribes `/cmd_vel`) and `libgazebo_ros_ray_sensor.so` (CPU ray, publishes `/scan`) — no bridge node needed. Three ROS 2 nodes (`map_manager`, `astar_planner`, `controller`) provide A* path planning and Pure Pursuit control. An `/override_path` topic lets external nodes replace the auto-planned path at runtime.

**Tech Stack:** ROS 2 Humble · Gazebo Classic 11 · `ros-humble-gazebo-ros-pkgs` · Python 3.10 · Docker + WSL2/WSLg

---

## File Map

| Path | Action | Responsibility |
|------|--------|----------------|
| `Dockerfile` | Rewrite | Gazebo Classic packages (drop Ignition) |
| `docker-compose.yml` | Rewrite | Remove Ignition env vars, update commands |
| `docker/entrypoint.sh` | Keep | Already correct |
| `src/av_sim/package.xml` | Modify | Replace ign deps with `gazebo_ros` |
| `src/av_sim/setup.py` | Modify | Remove rviz data_files entry |
| `src/av_sim/config/obstacles.csv` | Rewrite | Open field scattered layout (30×30 grid) |
| `src/av_sim/config/checkpoints.csv` | Rewrite | Open field checkpoints |
| `src/av_sim/urdf/robot.urdf` | Rewrite | Diff-drive plugin + CPU ray sensor (Gazebo Classic) |
| `src/av_sim/launch/sim.launch.py` | Rewrite | gzserver/gzclient launch, no bridge node |
| `src/av_sim/av_sim/map_math.py` | Create | Pure CSV loading + grid↔world math (no ROS) |
| `src/av_sim/av_sim/astar_math.py` | Create | Rename of `planning.py` (no API change) |
| `src/av_sim/av_sim/planning.py` | Delete | Replaced by `astar_math.py` |
| `src/av_sim/av_sim/map_manager.py` | Rewrite | Publishes OccupancyGrid + PoseArray (latched) |
| `src/av_sim/av_sim/astar_planner.py` | Rewrite | Adds `/scan` dynamic layer + new topic names |
| `src/av_sim/av_sim/control_math.py` | Keep | Already complete; all tests pass |
| `src/av_sim/av_sim/controller.py` | Rewrite | Adds `/override_path` + new checkpoint topics |
| `src/av_sim/test/test_map_math.py` | Create | Unit tests for `map_math.py` |
| `src/av_sim/test/test_astar.py` | Modify | Update import: `planning` → `astar_math` |
| `src/av_sim/test/test_planning_utils.py` | Modify | Update import: `planning` → `astar_math` |
| `src/av_sim/test/test_controller_math.py` | Keep | Already passing; no changes |
| `src/av_sim/rviz/av_sim.rviz` | Delete | Gazebo Classic has its own GUI |

---

## Task 1: Scaffold — package metadata and file cleanup

**Files:**
- Modify: `src/av_sim/package.xml`
- Modify: `src/av_sim/setup.py`
- Delete: `src/av_sim/av_sim/planning.py`, `src/av_sim/rviz/av_sim.rviz`

- [ ] **Step 1: Delete files no longer needed**

```bash
rm src/av_sim/av_sim/planning.py
rm src/av_sim/rviz/av_sim.rviz
```

- [ ] **Step 2: Rewrite `src/av_sim/package.xml`**

Replace the entire file with:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>av_sim</name>
  <version>0.4.0</version>
  <description>Autonomous vehicle simulation — A* + Pure Pursuit in Gazebo Classic 11</description>
  <maintainer email="user@example.com">User</maintainer>
  <license>Apache-2.0</license>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

- [ ] **Step 3: Rewrite `src/av_sim/setup.py`**

Replace the entire file with:

```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'av_sim'

setup(
    name=package_name,
    version='0.4.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.csv')),
        (os.path.join('share', package_name, 'urdf'),   glob('urdf/*.urdf')),
        (os.path.join('share', package_name),           ['pytest.ini']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Autonomous vehicle simulation — A* + Pure Pursuit in Gazebo Classic 11',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_manager   = av_sim.map_manager:main',
            'astar_planner = av_sim.astar_planner:main',
            'controller    = av_sim.controller:main',
        ],
    },
)
```

- [ ] **Step 4: Commit**

```bash
git add src/av_sim/package.xml src/av_sim/setup.py
git add -u src/av_sim/av_sim/planning.py src/av_sim/rviz/av_sim.rviz
git commit -m "chore: scaffold Gazebo Classic rewrite — drop ign deps and rviz"
```

---

## Task 2: Docker foundation

**Files:**
- Rewrite: `Dockerfile`
- Rewrite: `docker-compose.yml`

- [ ] **Step 1: Rewrite `Dockerfile`**

Replace the entire file with:

```dockerfile
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# Gazebo Classic 11 + ROS 2 bridge (already in humble-desktop repos)
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-ros \
        ros-humble-gazebo-plugins \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY src/ src/

RUN . /opt/ros/humble/setup.sh \
    && colcon build \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
         --event-handlers console_direct+ \
    && rm -rf build/ log/ src/

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Rewrite `docker-compose.yml`**

Replace the entire file with:

```yaml
# av_sim docker-compose — Gazebo Classic 11
#
# Services:
#   sim          — Gazebo Classic GUI + all ROS nodes  (primary)
#   sim-headless — gzserver only, no display           (CI)
#   test         — pytest unit test suite
#
# Quick start (WSL2 terminal):
#   docker compose up sim
#
# The Gazebo Classic GUI window opens via WSLg X11 forwarding.
# Ogre 1.x under Mesa software rendering is stable — no flickering.

x-common: &common
  build: .
  image: av_sim:latest
  network_mode: host

x-display-env: &display-env
  DISPLAY: ${DISPLAY:-:0}
  QT_X11_NO_MITSHM: "1"
  LIBGL_ALWAYS_SOFTWARE: "1"

services:

  sim:
    <<: *common
    environment:
      <<: *display-env
    volumes:
      - /mnt/wslg/.X11-unix:/tmp/.X11-unix:rw
    command: >
      bash -c "
        ros2 launch av_sim sim.launch.py
      "

  sim-headless:
    <<: *common
    environment:
      LIBGL_ALWAYS_SOFTWARE: "1"
    command: ros2 launch av_sim sim.launch.py headless:=true

  test:
    <<: *common
    environment:
      LIBGL_ALWAYS_SOFTWARE: "1"
    command: >
      bash -c "
        source /opt/ros/humble/setup.bash &&
        source /ws/install/setup.bash &&
        colcon test --packages-select av_sim --event-handlers console_direct+ &&
        colcon test-result --all --verbose
      "
```

- [ ] **Step 3: Build and verify it compiles**

```bash
docker compose build 2>&1 | tail -20
```

Expected: `Successfully built` (or `=> exporting to image` in BuildKit). The build will fail at `colcon build` if any Python syntax errors are present. Fix any reported errors before continuing.

- [ ] **Step 4: Commit**

```bash
git add Dockerfile docker-compose.yml
git commit -m "feat: switch Docker base to Gazebo Classic 11 (drop Ignition Fortress)"
```

---

## Task 3: Configuration files

**Files:**
- Rewrite: `src/av_sim/config/obstacles.csv`
- Rewrite: `src/av_sim/config/checkpoints.csv`

- [ ] **Step 1: Write `src/av_sim/config/obstacles.csv`**

Open field layout (30×30 grid, cell size 1 m). These cells are blocked — boxes placed here in Gazebo.

```
grid_x,grid_y
4,2
5,2
9,8
9,9
15,4
16,4
22,8
22,9
2,15
3,15
11,19
12,19
20,14
20,15
7,25
8,25
17,25
18,25
```

- [ ] **Step 2: Write `src/av_sim/config/checkpoints.csv`**

Five checkpoints that require traversal through the scattered obstacle field.

```
order,grid_x,grid_y
1,5,5
2,24,5
3,24,24
4,5,24
5,14,14
```

Robot spawns at world position (0.5, 0.5) — cell (0, 0). Checkpoint 1 at (5.5, 5.5) is the first target.

- [ ] **Step 3: Commit**

```bash
git add src/av_sim/config/
git commit -m "feat: open field config — 18 obstacles, 5 checkpoints on 30x30 grid"
```

---

## Task 4: Robot URDF

**Files:**
- Rewrite: `src/av_sim/urdf/robot.urdf`

- [ ] **Step 1: Write `src/av_sim/urdf/robot.urdf`**

Replace the entire file with:

```xml
<?xml version="1.0"?>
<robot name="av_robot">

  <!-- ── Chassis ─────────────────────────────────────────────────── -->
  <link name="base_link">
    <visual>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
      <material name="blue"><color rgba="0.2 0.4 0.8 1.0"/></material>
    </visual>
    <collision>
      <geometry><box size="0.4 0.3 0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.035" ixy="0" ixz="0" iyy="0.068" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <!-- ── Left wheel ─────────────────────────────────────────────── -->
  <joint name="left_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="left_wheel"/>
    <origin xyz="0 0.17 -0.025" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="left_wheel">
    <visual>
      <geometry><cylinder radius="0.1" length="0.04"/></geometry>
      <material name="dark"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.1" length="0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00125" ixy="0" ixz="0" iyy="0.00125" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- ── Right wheel ────────────────────────────────────────────── -->
  <joint name="right_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="right_wheel"/>
    <origin xyz="0 -0.17 -0.025" rpy="-1.5708 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <link name="right_wheel">
    <visual>
      <geometry><cylinder radius="0.1" length="0.04"/></geometry>
      <material name="dark"><color rgba="0.1 0.1 0.1 1"/></material>
    </visual>
    <collision>
      <geometry><cylinder radius="0.1" length="0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.00125" ixy="0" ixz="0" iyy="0.00125" iyz="0" izz="0.0025"/>
    </inertial>
  </link>

  <!-- ── Rear caster ────────────────────────────────────────────── -->
  <joint name="caster_joint" type="fixed">
    <parent link="base_link"/>
    <child link="caster_link"/>
    <origin xyz="-0.15 0 -0.07" rpy="0 0 0"/>
  </joint>
  <link name="caster_link">
    <visual>
      <geometry><sphere radius="0.04"/></geometry>
      <material name="grey"><color rgba="0.6 0.6 0.6 1"/></material>
    </visual>
    <collision>
      <geometry><sphere radius="0.04"/></geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="6.4e-5" ixy="0" ixz="0" iyy="6.4e-5" iyz="0" izz="6.4e-5"/>
    </inertial>
  </link>
  <gazebo reference="caster_link">
    <mu1>0.0</mu1>
    <mu2>0.0</mu2>
  </gazebo>

  <!-- ── Lidar mount ────────────────────────────────────────────── -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
  </joint>
  <link name="laser_link">
    <visual>
      <geometry><cylinder radius="0.03" length="0.04"/></geometry>
      <material name="black"><color rgba="0 0 0 1"/></material>
    </visual>
  </link>

  <!-- ── Gazebo: differential drive ────────────────────────────── -->
  <gazebo>
    <plugin name="diff_drive" filename="libgazebo_ros_diff_drive.so">
      <left_joint>left_wheel_joint</left_joint>
      <right_joint>right_wheel_joint</right_joint>
      <wheel_separation>0.34</wheel_separation>
      <wheel_diameter>0.2</wheel_diameter>
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>
      <command_topic>cmd_vel</command_topic>
      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>
    </plugin>
  </gazebo>

  <!-- ── Gazebo: CPU ray sensor (no GPU off-screen rendering) ──── -->
  <gazebo reference="laser_link">
    <sensor type="ray" name="lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <visualize>false</visualize>
      <ray>
        <scan>
          <horizontal>
            <samples>180</samples>
            <resolution>1</resolution>
            <min_angle>-3.14159265</min_angle>
            <max_angle>3.14159265</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.1</min>
          <max>8.0</max>
          <resolution>0.01</resolution>
        </range>
      </ray>
      <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <remapping>~/out:=scan</remapping>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type>
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>

</robot>
```

- [ ] **Step 2: Verify URDF parses (run inside Docker)**

```bash
docker compose run --rm test bash -c \
  "source /opt/ros/humble/setup.bash && \
   check_urdf /ws/src/av_sim/urdf/robot.urdf"
```

Expected output ends with: `Successfully Parsed XML`

- [ ] **Step 3: Commit**

```bash
git add src/av_sim/urdf/robot.urdf
git commit -m "feat: URDF — Gazebo Classic diff_drive + CPU ray sensor (no gpu_lidar)"
```

---

## Task 5: `map_math.py` — pure CSV + coordinate math (TDD)

**Files:**
- Create: `src/av_sim/test/test_map_math.py`
- Create: `src/av_sim/av_sim/map_math.py`

- [ ] **Step 1: Write the failing tests**

Create `src/av_sim/test/test_map_math.py`:

```python
"""Unit tests for map_math pure functions."""
import os
import tempfile
import pytest
from av_sim.map_math import (
    load_obstacles, load_checkpoints,
    cell_to_world, world_to_cell, in_bounds,
)


def _write_csv(content: str) -> str:
    f = tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False)
    f.write(content)
    f.close()
    return f.name


# ── load_obstacles ────────────────────────────────────────────────────────────

def test_load_obstacles_basic():
    path = _write_csv("grid_x,grid_y\n1,2\n3,4\n")
    try:
        result = load_obstacles(path)
    finally:
        os.unlink(path)
    assert result == [(1, 2), (3, 4)]


def test_load_obstacles_empty():
    path = _write_csv("grid_x,grid_y\n")
    try:
        result = load_obstacles(path)
    finally:
        os.unlink(path)
    assert result == []


def test_load_obstacles_single():
    path = _write_csv("grid_x,grid_y\n7,3\n")
    try:
        result = load_obstacles(path)
    finally:
        os.unlink(path)
    assert result == [(7, 3)]


# ── load_checkpoints ──────────────────────────────────────────────────────────

def test_load_checkpoints_sorted_by_order():
    path = _write_csv("order,grid_x,grid_y\n2,5,6\n1,3,4\n3,7,8\n")
    try:
        result = load_checkpoints(path)
    finally:
        os.unlink(path)
    assert result == [(3, 4), (5, 6), (7, 8)]


def test_load_checkpoints_single():
    path = _write_csv("order,grid_x,grid_y\n1,10,20\n")
    try:
        result = load_checkpoints(path)
    finally:
        os.unlink(path)
    assert result == [(10, 20)]


# ── cell_to_world ─────────────────────────────────────────────────────────────

def test_cell_to_world_origin():
    wx, wy = cell_to_world(0, 0)
    assert wx == pytest.approx(0.5)
    assert wy == pytest.approx(0.5)


def test_cell_to_world_nonzero():
    wx, wy = cell_to_world(2, 3)
    assert wx == pytest.approx(2.5)
    assert wy == pytest.approx(3.5)


def test_cell_to_world_custom_cell_size():
    wx, wy = cell_to_world(1, 1, cell_size=2.0)
    assert wx == pytest.approx(3.0)
    assert wy == pytest.approx(3.0)


# ── world_to_cell ─────────────────────────────────────────────────────────────

def test_world_to_cell_origin():
    assert world_to_cell(0.5, 0.5) == (0, 0)


def test_world_to_cell_nonzero():
    assert world_to_cell(2.7, 3.1) == (2, 3)


def test_world_to_cell_boundary():
    assert world_to_cell(1.0, 1.0) == (1, 1)


def test_world_to_cell_roundtrip():
    for gx, gy in [(0, 0), (5, 3), (14, 29)]:
        wx, wy = cell_to_world(gx, gy)
        assert world_to_cell(wx, wy) == (gx, gy)


# ── in_bounds ─────────────────────────────────────────────────────────────────

def test_in_bounds_centre():
    assert in_bounds(5, 5, 10, 10)


def test_in_bounds_origin():
    assert in_bounds(0, 0, 10, 10)


def test_in_bounds_at_limit_false():
    assert not in_bounds(10, 5, 10, 10)
    assert not in_bounds(5, 10, 10, 10)


def test_in_bounds_negative_false():
    assert not in_bounds(-1, 0, 10, 10)
    assert not in_bounds(0, -1, 10, 10)
```

- [ ] **Step 2: Run tests to confirm they fail**

```bash
docker compose run --rm test bash -c \
  "cd /ws && source install/setup.bash && \
   python3 -m pytest src/av_sim/test/test_map_math.py -v 2>&1 | head -30"
```

Expected: `ModuleNotFoundError: No module named 'av_sim.map_math'`

- [ ] **Step 3: Write `src/av_sim/av_sim/map_math.py`**

```python
"""Pure CSV loading and grid/world coordinate math — no ROS dependencies."""
import csv


def load_obstacles(path: str) -> list:
    with open(path, newline='') as f:
        return [(int(r['grid_x']), int(r['grid_y'])) for r in csv.DictReader(f)]


def load_checkpoints(path: str) -> list:
    with open(path, newline='') as f:
        rows = sorted(csv.DictReader(f), key=lambda r: int(r['order']))
    return [(int(r['grid_x']), int(r['grid_y'])) for r in rows]


def cell_to_world(gx: int, gy: int, cell_size: float = 1.0) -> tuple:
    return (gx + 0.5) * cell_size, (gy + 0.5) * cell_size


def world_to_cell(wx: float, wy: float, cell_size: float = 1.0) -> tuple:
    return int(wx / cell_size), int(wy / cell_size)


def in_bounds(gx: int, gy: int, width: int, height: int) -> bool:
    return 0 <= gx < width and 0 <= gy < height
```

- [ ] **Step 4: Rebuild and run tests**

```bash
docker compose build --no-cache 2>&1 | tail -5
docker compose run --rm test bash -c \
  "cd /ws && source install/setup.bash && \
   python3 -m pytest src/av_sim/test/test_map_math.py -v"
```

Expected: all tests `PASSED`.

- [ ] **Step 5: Commit**

```bash
git add src/av_sim/av_sim/map_math.py src/av_sim/test/test_map_math.py
git commit -m "feat: map_math — pure CSV loading and grid/world coordinate functions"
```

---

## Task 6: `astar_math.py` — rename `planning.py` + update tests

`planning.py` already has correct `astar`, `inflate_obstacles`, and `prune_path` implementations. This task renames it and updates all test imports.

**Files:**
- Create: `src/av_sim/av_sim/astar_math.py` (copy of planning.py)
- Modify: `src/av_sim/test/test_astar.py`
- Modify: `src/av_sim/test/test_planning_utils.py`

- [ ] **Step 1: Create `src/av_sim/av_sim/astar_math.py`**

Copy `planning.py` content exactly:

```python
"""Pure A* path-planning algorithm — no ROS dependencies."""
import heapq
import math


def astar(start, goal, obstacles: set, grid_w: int, grid_h: int):
    """8-directional A* on an integer grid.

    Returns an ordered list of (gx, gy) cells from *start* to *goal*, or
    None if no path exists.  *obstacles* is a set of blocked (gx, gy) cells.
    """
    if goal in obstacles:
        return None

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
            if not (0 <= nb[0] < grid_w and 0 <= nb[1] < grid_h):
                continue
            if nb in obstacles:
                continue
            ng = g[current] + cost
            if ng < g.get(nb, float('inf')):
                came_from[nb] = current
                g[nb] = ng
                heapq.heappush(open_heap, (ng + h(nb), nb))

    return None


def inflate_obstacles(obstacles: set, grid_w: int, grid_h: int,
                      radius: int = 1) -> set:
    """Expand each obstacle cell by *radius* grid cells in all 8 directions."""
    inflated: set = set()
    for gx, gy in obstacles:
        for dx in range(-radius, radius + 1):
            for dy in range(-radius, radius + 1):
                nx, ny = gx + dx, gy + dy
                if 0 <= nx < grid_w and 0 <= ny < grid_h:
                    inflated.add((nx, ny))
    return inflated


def prune_path(path) -> list:
    """Remove collinear intermediate waypoints."""
    if len(path) <= 2:
        return list(path)
    result = [path[0]]
    for i in range(1, len(path) - 1):
        prev = result[-1]
        curr = path[i]
        nxt = path[i + 1]
        cross = ((curr[0] - prev[0]) * (nxt[1] - prev[1]) -
                 (curr[1] - prev[1]) * (nxt[0] - prev[0]))
        if cross != 0:
            result.append(curr)
    result.append(path[-1])
    return result
```

- [ ] **Step 2: Update `src/av_sim/test/test_astar.py` import line**

Change:
```python
from av_sim.planning import astar
```
To:
```python
from av_sim.astar_math import astar
```

- [ ] **Step 3: Update `src/av_sim/test/test_planning_utils.py` import line**

Change:
```python
from av_sim.planning import inflate_obstacles, prune_path
```
To:
```python
from av_sim.astar_math import inflate_obstacles, prune_path
```

- [ ] **Step 4: Run all tests to verify nothing broke**

```bash
docker compose run --rm test bash -c \
  "cd /ws && source install/setup.bash && \
   python3 -m pytest src/av_sim/test/ -v --ignore=src/av_sim/test/test_map_math.py"
```

Expected: `test_astar.py` passes, `test_planning_utils.py` passes, `test_controller_math.py` passes.

- [ ] **Step 5: Commit**

```bash
git add src/av_sim/av_sim/astar_math.py \
        src/av_sim/test/test_astar.py \
        src/av_sim/test/test_planning_utils.py
git commit -m "refactor: rename planning.py → astar_math.py, update test imports"
```

---

## Task 7: `map_manager.py` — rewrite for new topic contract

**Files:**
- Rewrite: `src/av_sim/av_sim/map_manager.py`

- [ ] **Step 1: Write `src/av_sim/av_sim/map_manager.py`**

Replace the entire file:

```python
"""map_manager — loads CSVs and publishes static obstacle + checkpoint data."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

from av_sim.map_math import load_obstacles, load_checkpoints, cell_to_world

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class MapManager(Node):

    def __init__(self):
        super().__init__('map_manager')
        self.declare_parameter('obstacles_csv',   '')
        self.declare_parameter('checkpoints_csv', '')
        self.declare_parameter('cell_size',       1.0)
        self.declare_parameter('grid_width',      30)
        self.declare_parameter('grid_height',     30)

        obs_csv = self.get_parameter('obstacles_csv').value
        cp_csv  = self.get_parameter('checkpoints_csv').value
        cs      = self.get_parameter('cell_size').value
        gw      = self.get_parameter('grid_width').value
        gh      = self.get_parameter('grid_height').value

        obs = load_obstacles(obs_csv)
        cps = load_checkpoints(cp_csv)

        obs_pub    = self.create_publisher(OccupancyGrid, '/map/obstacles',      _LATCHED)
        cp_pub     = self.create_publisher(PoseArray,     '/checkpoints',        _LATCHED)
        marker_pub = self.create_publisher(MarkerArray,   '/checkpoint_markers', _LATCHED)

        obs_pub.publish(self._build_grid(obs, gw, gh, cs))
        cp_pub.publish(self._build_pose_array(cps, cs))
        marker_pub.publish(self._build_markers(cps, cs))

        self.get_logger().info(
            f'Published {len(obs)} obstacles, {len(cps)} checkpoints'
        )

    def _build_grid(self, obstacles, gw, gh, cs):
        msg = OccupancyGrid()
        msg.header.frame_id = 'map'
        msg.info.resolution = cs
        msg.info.width  = gw
        msg.info.height = gh
        data = [0] * (gw * gh)
        for gx, gy in obstacles:
            if 0 <= gx < gw and 0 <= gy < gh:
                data[gy * gw + gx] = 100
        msg.data = data
        return msg

    def _build_pose_array(self, checkpoints, cs):
        msg = PoseArray()
        msg.header.frame_id = 'map'
        for gx, gy in checkpoints:
            wx, wy = cell_to_world(gx, gy, cs)
            p = Pose()
            p.position.x = wx
            p.position.y = wy
            p.orientation.w = 1.0
            msg.poses.append(p)
        return msg

    def _build_markers(self, checkpoints, cs):
        arr = MarkerArray()
        for i, (gx, gy) in enumerate(checkpoints):
            wx, wy = cell_to_world(gx, gy, cs)
            m = Marker()
            m.header.frame_id = 'map'
            m.ns = 'checkpoints'
            m.id = i
            m.type = Marker.CYLINDER
            m.action = Marker.ADD
            m.pose.position.x = wx
            m.pose.position.y = wy
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.6
            m.scale.z = 0.1
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.8, 0.1, 0.9
            m.lifetime = Duration()
            arr.markers.append(m)
        return arr


def main():
    rclpy.init()
    node = MapManager()
    rclpy.spin(node)
    rclpy.shutdown()
```

- [ ] **Step 2: Commit**

```bash
git add src/av_sim/av_sim/map_manager.py
git commit -m "feat: map_manager — OccupancyGrid + PoseArray with latched QoS"
```

---

## Task 8: `astar_planner.py` — rewrite with `/scan` dynamic layer

**Files:**
- Rewrite: `src/av_sim/av_sim/astar_planner.py`

- [ ] **Step 1: Write `src/av_sim/av_sim/astar_planner.py`**

Replace the entire file:

```python
"""astar_planner — replanning A* node with lidar dynamic obstacle layer."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Int32

from av_sim.astar_math import astar, inflate_obstacles, prune_path
from av_sim.map_math import cell_to_world, world_to_cell, in_bounds

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)


class AstarPlanner(Node):

    def __init__(self):
        super().__init__('astar_planner')
        self.declare_parameter('grid_width',       30)
        self.declare_parameter('grid_height',      30)
        self.declare_parameter('cell_size',         1.0)
        self.declare_parameter('inflation_radius',  1)

        self._gw  = self.get_parameter('grid_width').value
        self._gh  = self.get_parameter('grid_height').value
        self._cs  = self.get_parameter('cell_size').value
        self._inf = self.get_parameter('inflation_radius').value

        self._static_obs: set = set()
        self._scan_obs:   set = set()
        self._checkpoints: list = []   # [(wx, wy), ...]
        self._cp_idx: int = 0
        self._robot_x: float = 0.5
        self._robot_y: float = 0.5
        self._robot_yaw: float = 0.0
        self._last_path: list = []     # [(wx, wy), ...]

        self.create_subscription(OccupancyGrid, '/map/obstacles',      self._on_map,       _LATCHED)
        self.create_subscription(PoseArray,     '/checkpoints',        self._on_checkpoints, _LATCHED)
        self.create_subscription(Odometry,      '/odom',               self._on_odom,      10)
        self.create_subscription(LaserScan,     '/scan',               self._on_scan,      10)
        self.create_subscription(Int32,         '/checkpoint_reached', self._on_cp_reached, 10)
        self.create_subscription(Empty,         '/replan_request',     self._on_replan,    10)

        self._path_pub    = self.create_publisher(Path,         '/planned_path', 1)
        self._inflated_pub = self.create_publisher(OccupancyGrid, '/map/inflated', 1)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _on_map(self, msg):
        self._static_obs = set()
        w, h = msg.info.width, msg.info.height
        for i, v in enumerate(msg.data):
            if v == 100:
                self._static_obs.add((i % w, i // w))
        self._replan()

    def _on_checkpoints(self, msg):
        self._checkpoints = [(p.position.x, p.position.y) for p in msg.poses]
        self._cp_idx = 0
        self._replan()

    def _on_odom(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _on_scan(self, msg):
        new_obs: set = set()
        angle = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                wx = self._robot_x + r * math.cos(self._robot_yaw + angle)
                wy = self._robot_y + r * math.sin(self._robot_yaw + angle)
                gx, gy = world_to_cell(wx, wy, self._cs)
                if in_bounds(gx, gy, self._gw, self._gh):
                    new_obs.add((gx, gy))
            angle += msg.angle_increment

        added = new_obs - self._scan_obs
        self._scan_obs = new_obs
        if added and self._path_crosses(added):
            self._replan()

    def _on_cp_reached(self, msg):
        self._cp_idx = msg.data + 1
        if self._cp_idx < len(self._checkpoints):
            self._replan()
        else:
            self.get_logger().info('All checkpoints reached!')

    def _on_replan(self, _msg):
        self._replan()

    # ── planning ───────────────────────────────────────────────────────────────

    def _path_crosses(self, cells: set) -> bool:
        for wx, wy in self._last_path:
            if world_to_cell(wx, wy, self._cs) in cells:
                return True
        return False

    def _replan(self):
        if not self._checkpoints or self._cp_idx >= len(self._checkpoints):
            return

        all_obs  = self._static_obs | self._scan_obs
        inflated = inflate_obstacles(all_obs, self._gw, self._gh, self._inf)

        sx = max(0, min(self._gw - 1, int(self._robot_x / self._cs)))
        sy = max(0, min(self._gh - 1, int(self._robot_y / self._cs)))
        tx, ty = self._checkpoints[self._cp_idx]
        gx = max(0, min(self._gw - 1, int(tx / self._cs)))
        gy = max(0, min(self._gh - 1, int(ty / self._cs)))

        cells = astar((sx, sy), (gx, gy), inflated, self._gw, self._gh)
        if cells is None:
            cells = astar((sx, sy), (gx, gy), all_obs, self._gw, self._gh)
        if cells is None:
            self.get_logger().warn(f'No path from ({sx},{sy}) to ({gx},{gy})')
            return

        cells = prune_path(cells)
        world_pts = [cell_to_world(c[0], c[1], self._cs) for c in cells]
        self._last_path = world_pts

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        for wx, wy in world_pts:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        self._path_pub.publish(path_msg)
        self.get_logger().info(
            f'Replanned: {len(cells)} waypoints → cp[{self._cp_idx}] ({tx:.1f},{ty:.1f})'
        )


def main():
    rclpy.init()
    node = AstarPlanner()
    rclpy.spin(node)
    rclpy.shutdown()
```

- [ ] **Step 2: Commit**

```bash
git add src/av_sim/av_sim/astar_planner.py
git commit -m "feat: astar_planner — OccupancyGrid input, /scan dynamic layer, new checkpoint topics"
```

---

## Task 9: `controller.py` — rewrite with override path support

**Files:**
- Rewrite: `src/av_sim/av_sim/controller.py`

- [ ] **Step 1: Write `src/av_sim/av_sim/controller.py`**

Replace the entire file:

```python
"""controller — Pure Pursuit follower with external path override support."""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseArray, TransformStamped, Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Empty, Int32
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker

from av_sim.control_math import (
    _yaw_from_quat,
    find_lookahead_point,
    pure_pursuit_cmd,
    pure_pursuit_curvature,
    LOOKAHEAD_DIST,
)

_LATCHED = QoSProfile(
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE,
)

WAYPOINT_RADIUS   = 0.35   # m — path waypoint considered reached
CHECKPOINT_RADIUS = 0.75   # m — checkpoint detection radius
STUCK_TIMEOUT     = 6.0    # s
MIN_MOVE_M        = 0.08   # m


class Controller(Node):

    def __init__(self):
        super().__init__('controller')

        self._planned_path: list  = []   # [(x,y)]
        self._override_path: list = []   # [(x,y)]
        self._using_override      = False
        self._path_idx            = 0

        self._robot_x   = 0.5
        self._robot_y   = 0.5
        self._robot_yaw = 0.0

        self._checkpoints: list = []   # [(wx,wy)]
        self._cp_idx            = 0

        self._last_pos       = (0.5, 0.5)
        self._last_move_time = self.get_clock().now().nanoseconds / 1e9

        self._tf_br = TransformBroadcaster(self)

        self.create_subscription(Path,      '/planned_path',  self._on_planned,  10)
        self.create_subscription(Path,      '/override_path', self._on_override, 10)
        self.create_subscription(Odometry,  '/odom',          self._on_odom,     10)
        self.create_subscription(PoseArray, '/checkpoints',   self._on_checkpoints, _LATCHED)

        self._cmd_pub    = self.create_publisher(Twist,  '/cmd_vel',             10)
        self._marker_pub = self.create_publisher(Marker, '/lookahead_marker',    10)
        self._cp_pub     = self.create_publisher(Int32,  '/checkpoint_reached',  10)
        self._replan_pub = self.create_publisher(Empty,  '/replan_request',      10)
        self._exhaust_pub = self.create_publisher(Empty, '/override_exhausted',  10)

        self.create_timer(0.1, self._control_loop)

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _on_planned(self, msg):
        self._planned_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not self._using_override:
            self._path_idx = 0

    def _on_override(self, msg):
        self._override_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._using_override = True
        self._path_idx = 0
        self.get_logger().info('Switched to override path')

    def _on_odom(self, msg):
        p = msg.pose.pose
        self._robot_x   = p.position.x
        self._robot_y   = p.position.y
        self._robot_yaw = _yaw_from_quat(p.orientation)
        self._broadcast_tf(msg)

    def _on_checkpoints(self, msg):
        self._checkpoints = [(p.position.x, p.position.y) for p in msg.poses]

    # ── control loop ──────────────────────────────────────────────────────────

    def _control_loop(self):
        active = self._override_path if self._using_override else self._planned_path
        if not active:
            self._publish_cmd(0.0, 0.0)
            return

        self._check_checkpoints()
        self._check_stuck()

        lp, new_idx = find_lookahead_point(
            active, self._robot_x, self._robot_y, self._path_idx, LOOKAHEAD_DIST
        )
        self._path_idx = new_idx

        dist_to_end = math.hypot(
            active[-1][0] - self._robot_x,
            active[-1][1] - self._robot_y,
        )

        if self._using_override and new_idx >= len(self._override_path) - 1 and dist_to_end < 0.5:
            self._using_override = False
            self._path_idx = 0
            self._exhaust_pub.publish(Empty())
            self.get_logger().info('Override path exhausted, reverting to planned path')
            return

        k   = pure_pursuit_curvature(self._robot_x, self._robot_y, self._robot_yaw, lp[0], lp[1])
        lin, ang = pure_pursuit_cmd(dist=dist_to_end, curvature=k)
        self._publish_cmd(lin, ang)
        self._publish_lookahead(lp)

    def _check_checkpoints(self):
        if self._cp_idx >= len(self._checkpoints):
            return
        tx, ty = self._checkpoints[self._cp_idx]
        if math.hypot(tx - self._robot_x, ty - self._robot_y) < CHECKPOINT_RADIUS:
            msg = Int32()
            msg.data = self._cp_idx
            self._cp_pub.publish(msg)
            self.get_logger().info(f'Checkpoint {self._cp_idx} reached')
            self._cp_idx += 1

    def _check_stuck(self):
        now = self.get_clock().now().nanoseconds / 1e9
        if math.hypot(self._robot_x - self._last_pos[0],
                      self._robot_y - self._last_pos[1]) > MIN_MOVE_M:
            self._last_pos = (self._robot_x, self._robot_y)
            self._last_move_time = now
        elif now - self._last_move_time > STUCK_TIMEOUT:
            self.get_logger().warn('Robot stuck — requesting replan')
            self._replan_pub.publish(Empty())
            self._last_move_time = now

    # ── helpers ───────────────────────────────────────────────────────────────

    def _publish_cmd(self, linear, angular):
        t = Twist()
        t.linear.x  = float(linear)
        t.angular.z = float(angular)
        self._cmd_pub.publish(t)

    def _publish_lookahead(self, pt):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = pt[0]
        m.pose.position.y = pt[1]
        m.pose.position.z = 0.2
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.2
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.5, 0.0, 1.0
        m.lifetime = Duration(sec=0, nanosec=200_000_000)
        self._marker_pub.publish(m)

    def _broadcast_tf(self, odom_msg):
        t = TransformStamped()
        t.header = odom_msg.header
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_link'
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.translation.z = odom_msg.pose.pose.position.z
        t.transform.rotation      = odom_msg.pose.pose.orientation
        self._tf_br.sendTransform(t)


def main():
    rclpy.init()
    node = Controller()
    rclpy.spin(node)
    rclpy.shutdown()
```

- [ ] **Step 2: Commit**

```bash
git add src/av_sim/av_sim/controller.py
git commit -m "feat: controller — override_path support, new checkpoint/replan topics"
```

---

## Task 10: `sim.launch.py` — rewrite for Gazebo Classic

**Files:**
- Rewrite: `src/av_sim/launch/sim.launch.py`

- [ ] **Step 1: Write `src/av_sim/launch/sim.launch.py`**

Replace the entire file:

```python
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
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, OpaqueFunction, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
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

    with open(WORLD_PATH, 'w') as f:
        f.write(_build_world(obstacles, checkpoints))

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
```

- [ ] **Step 2: Commit**

```bash
git add src/av_sim/launch/sim.launch.py
git commit -m "feat: sim.launch.py — Gazebo Classic 11, gzserver+gzclient, no bridge node"
```

---

## Task 11: Full build + smoke test

- [ ] **Step 1: Full clean build**

```bash
docker compose build --no-cache 2>&1 | tail -30
```

Expected: `Successfully built` with no Python errors. If `colcon build` fails, fix the reported import error or syntax error in the indicated file and rebuild.

- [ ] **Step 2: Run unit test suite**

```bash
docker compose run --rm test
```

Expected: all tests in `test_map_math.py`, `test_astar.py`, `test_planning_utils.py`, `test_controller_math.py` pass. Fix any failures before continuing.

- [ ] **Step 3: Smoke test — headless (verify Gazebo starts and nodes connect)**

Run in one terminal:
```bash
docker compose up sim-headless
```

In a second terminal after ~10 s:
```bash
docker compose exec sim-headless bash -c \
  "source /ws/install/setup.bash && \
   ros2 topic list | grep -E 'planned_path|odom|scan|cmd_vel|checkpoints'"
```

Expected output includes all of:
```
/checkpoints
/cmd_vel
/odom
/planned_path
/scan
```

If `/scan` is missing: the lidar plugin did not load. Check robot.urdf `<sensor type="ray">` block and confirm `ros-humble-gazebo-plugins` is installed.

If `/planned_path` is missing: `astar_planner` did not receive `/map/obstacles` or `/checkpoints`. Check `map_manager` logs.

- [ ] **Step 4: Visual smoke test — GUI (verify no flickering)**

```bash
docker compose up sim
```

Expected: Gazebo Classic window opens showing the flat field with red obstacle boxes and green checkpoint cylinders. The blue robot appears after ~3 s and starts moving after ~5 s. The viewport must **not** flicker. If it flickers, see troubleshooting note below.

> **Troubleshooting rendering:** If Gazebo shows a black window or crashes with `Failed to create OpenGL context`, add `MESA_GL_VERSION_OVERRIDE: "3.3"` to the `x-display-env` block in `docker-compose.yml`. This forces Mesa to advertise OpenGL 3.3 to applications that check the version string before rendering.

- [ ] **Step 5: Verify checkpoint navigation**

Watch the terminal logs from `docker compose up sim`. After ~5 s you should see:
```
[controller]: Checkpoint 0 reached
[astar_planner]: Replanned: N waypoints → cp[1] (24.5, 5.5)
```
Each checkpoint should be reached in sequence 0→4.

- [ ] **Step 6: Verify lidar scan rate**

In a second terminal while `sim` is running:
```bash
docker compose exec sim bash -c \
  "source /ws/install/setup.bash && ros2 topic hz /scan"
```

Expected: ~10 Hz.

- [ ] **Step 7: Test external path override**

While `sim` is running, publish a two-waypoint override path:
```bash
docker compose exec sim bash -c "
  source /ws/install/setup.bash &&
  ros2 topic pub --once /override_path nav_msgs/msg/Path '{
    header: {frame_id: map},
    poses: [
      {header: {frame_id: map}, pose: {position: {x: 3.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}},
      {header: {frame_id: map}, pose: {position: {x: 6.0, y: 3.0, z: 0.0}, orientation: {w: 1.0}}}
    ]
  }'
"
```

Expected: controller log shows `Switched to override path`, robot moves toward (3.0, 3.0), then (6.0, 3.0), then logs `Override path exhausted` and reverts to checkpoint planning.

- [ ] **Step 8: Commit final state**

```bash
git add -A
git commit -m "feat: complete Gazebo Classic 11 rewrite — CPU lidar, stable rendering, path override"
```

---

## Self-Review Checklist

- [x] **Spec § Architecture** — static_transform_publisher included (Task 10)
- [x] **Spec § Robot URDF** — CPU ray sensor, diff_drive plugin (Task 4)
- [x] **Spec § astar_planner** — /scan dynamic layer, /checkpoint_reached trigger, /replan_request trigger (Task 8)
- [x] **Spec § controller** — /override_path, /override_exhausted, /checkpoint_reached publisher (Task 9)
- [x] **Spec § map_manager** — OccupancyGrid, PoseArray, MarkerArray with latched QoS (Task 7)
- [x] **Spec § Docker** — Gazebo Classic packages, LIBGL_ALWAYS_SOFTWARE kept for Mesa stability (Task 2)
- [x] **Spec § Testing** — test_map_math.py (Task 5), astar tests updated (Task 6), control_math kept (unchanged)
- [x] **Type consistency** — `_on_checkpoints` in controller uses `msg.poses` (PoseArray) ✓; astar_planner `_on_map` reads OccupancyGrid ✓
- [x] **No placeholders** — all steps contain actual code
