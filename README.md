# Autonomous Vehicle Simulation

A ROS2 Humble simulation of a differential-drive robot that autonomously navigates through ordered checkpoints on a 20 × 20 grid while detecting and avoiding obstacles in real time. The physics and sensor simulation run in Gazebo Fortress (Ignition Gazebo 6); navigation, planning, and control run as pure ROS2 nodes.

---

## Feature overview

| Feature | Details |
|---|---|
| Path planning | 8-directional A\* with obstacle inflation (configurable radius) and waypoint pruning |
| Steering | Pure Pursuit controller — smooth arcs, no oscillation |
| Reactive replanning | Re-plans from current robot position whenever obstacles or checkpoints change, or when stuck |
| Lidar | 360° GPU lidar at 10 Hz; bridged to `/scan`; used for emergency stop and scan-obstacle layer |
| Adaptive speed | Linearly scales velocity from full speed at 2 m clearance down to zero at 0.6 m |
| Emergency stop | Independent of planner; zeroes velocity if any scan return < 0.6 m in the forward ±45° arc |
| Stuck recovery | After 6 s without movement, forces a replan from current position |
| Checkpoint events | Publishes 1-indexed `Int32` on `/current_checkpoint` as each is reached |
| Mission complete | Publishes `Bool(True)` on `/mission_complete` when all checkpoints are done |
| RViz2 | Live view: planned path, obstacles (static + scan), robot model, checkpoint markers, lookahead point |
| CSV substitution | Hot-swap obstacle/checkpoint layouts via launch arguments — no rebuild needed |
| Tests | 85 pytest unit tests; run with `colcon test` |

---

## Docker — deploy anywhere

> **This is the recommended way to run the simulation.** No ROS2 or Gazebo installation needed on the host — works on Windows, macOS, and Linux.

---

### Windows

> Requires: [Docker Desktop for Windows](https://docs.docker.com/desktop/install/windows-install/) with WSL2 backend enabled (the default). Git for Windows or WSL2 terminal.

**Step 1 — Clone the repo**

Open a WSL2 terminal (Ubuntu from the Start menu) or PowerShell:

```bash
git clone https://github.com/yashpatel1314/autonomous_vehicle.git
cd autonomous_vehicle
```

**Step 2 — Build the image** (~10–15 min on first run)

```bash
docker compose build
```

**Step 3 — Run (with GUI via WSLg)**

WSLg is built into Windows 11 and Windows 10 22H2+ — it provides an X server automatically. Run from a WSL2 terminal:

```bash
docker compose up sim
```

Gazebo and RViz2 open as native windows on your desktop. The robot starts driving within ~5 seconds.

**No GUI / headless (works on all Windows versions)**

```bash
docker compose up sim-headless
```

---

### macOS

> Requires: [Docker Desktop for Mac](https://docs.docker.com/desktop/install/mac-install/) (Apple Silicon or Intel). [XQuartz](https://www.xquartz.org) for the GUI.

**Step 1 — Install XQuartz and allow network connections**

1. Download and install [XQuartz](https://www.xquartz.org).
2. Open XQuartz, go to **Preferences → Security**, and tick **"Allow connections from network clients"**.
3. Log out and back in (or reboot) so the setting takes effect.

**Step 2 — Clone the repo**

```bash
git clone https://github.com/yashpatel1314/autonomous_vehicle.git
cd autonomous_vehicle
```

**Step 3 — Build the image** (~10–15 min on first run)

```bash
docker compose build
```

**Step 4 — Run (with GUI via XQuartz)**

```bash
xhost +localhost
DISPLAY=host.docker.internal:0 docker compose up sim
```

Gazebo and RViz2 open inside XQuartz. The robot starts driving within ~5 seconds.

**No GUI / headless**

```bash
docker compose up sim-headless
```

---

### Linux

> Requires: [Docker Engine](https://docs.docker.com/engine/install/) 24+ and the Compose plugin (`sudo apt install docker-compose-plugin`).

**Step 1 — Clone the repo**

```bash
git clone https://github.com/yashpatel1314/autonomous_vehicle.git
cd autonomous_vehicle
```

**Step 2 — Build the image** (~10–15 min on first run)

```bash
docker compose build
```

**Step 3 — Run (with GUI)**

```bash
xhost +local:docker
docker compose up sim
```

**No GUI / headless**

```bash
docker compose up sim-headless
```

---

### Run tests (any OS)

```bash
docker compose up test
```

Expected output: `85 tests, 0 errors, 0 failures, 0 skipped`.

---

### Run with custom CSV maps

Mount your local maps directory and pass the paths as launch arguments:

```bash
docker compose run --rm sim-headless \
  ros2 launch av_sim sim.launch.py \
    obstacles_csv:=/maps/obstacles.csv \
    checkpoints_csv:=/maps/checkpoints.csv
```

Uncomment the volume mount in `docker-compose.yml` to make this automatic:

```yaml
# docker-compose.yml → sim service → volumes
- ./my_maps:/maps:ro
```

---

### Monitor topics inside a running container

```bash
# Open a shell in the running container
docker compose exec sim bash

# Inside the container
source /opt/ros/humble/setup.bash
source /ws/install/setup.bash
ros2 topic echo /current_checkpoint
ros2 topic echo /mission_complete
```

---

### GPU hardware acceleration (Linux only)

By default the container uses software rendering (`LIBGL_ALWAYS_SOFTWARE=1`). To pass through a GPU:

```bash
# Uncomment in docker-compose.yml:
#   devices:
#     - /dev/dri:/dev/dri
LIBGL_ALWAYS_SOFTWARE=0 docker compose up sim
```

> **Note:** The GPU lidar sensor (`/scan`) requires the `ogre2` render engine. It is silently skipped on pure software rendering — the rest of the simulation (planning, control, RViz2) continues normally.

---

## System requirements (native install)

> **Skip this section if you are using Docker** — the container includes everything.

| Component | Version |
|---|---|
| OS | Ubuntu 22.04 LTS (native or WSL2) |
| ROS2 | Humble Hawksbill |
| Simulator | Gazebo Fortress (Ignition Gazebo 6) |
| Python | 3.10+ (ships with Ubuntu 22.04) |

### Required apt packages

```bash
sudo apt update && sudo apt install -y \
  ros-humble-robot-state-publisher \
  ros-humble-ros-ign-gazebo \
  ros-humble-ros-ign-bridge \
  ros-humble-rviz2 \
  ros-humble-tf2-ros \
  ros-humble-visualization-msgs
```

---

## Build

```bash
# 1. Source ROS2 once per shell session
source /opt/ros/humble/setup.bash

# 2. From the workspace root (the repo root)
cd ~/autonomous_vehicle

# 3. Build with symlink install so CSV / config edits are live immediately
colcon build --symlink-install

# 4. Source the install overlay
source install/setup.bash
```

> **WSL2 note:** Gazebo Fortress requires a GPU-capable X server or a software renderer.
> Set `export LIBGL_ALWAYS_SOFTWARE=1` before launching if you have no GPU pass-through.

---

## Quick start

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch av_sim sim.launch.py
```

The simulation starts, Gazebo opens, and the robot begins driving autonomously within ~5 seconds.

---

## Launch options

| Argument | Default | Description |
|---|---|---|
| `obstacles_csv` | `<pkg>/config/obstacles.csv` | Absolute path to obstacles CSV |
| `checkpoints_csv` | `<pkg>/config/checkpoints.csv` | Absolute path to checkpoints CSV |
| `no_rviz` | `true` | Set to `false` to launch RViz2 alongside Gazebo |

### Examples

```bash
# Default (uses bundled CSV files)
ros2 launch av_sim sim.launch.py

# Custom map layout — no rebuild needed
ros2 launch av_sim sim.launch.py \
  obstacles_csv:=/home/user/maps/my_obstacles.csv \
  checkpoints_csv:=/home/user/maps/my_checkpoints.csv

# Headless — no RViz2 window
ros2 launch av_sim sim.launch.py no_rviz:=true

# Custom map, headless
ros2 launch av_sim sim.launch.py \
  obstacles_csv:=/home/user/maps/my_obstacles.csv \
  checkpoints_csv:=/home/user/maps/my_checkpoints.csv \
  no_rviz:=true
```

---

## CSV format

### `obstacles.csv`

One row per impassable grid cell. Coordinates are integers in `[0, 19]`.

```
grid_x,grid_y
3,2
3,3
5,7
```

### `checkpoints.csv`

One row per checkpoint. The robot visits them in ascending `order`. Coordinates must not coincide with any obstacle cell.

```
order,grid_x,grid_y
1,5,3
2,12,4
3,15,10
4,18,17
```

### Editing the bundled files

The bundled CSVs live in `src/av_sim/config/`. Because the workspace was built with `--symlink-install`, edits are reflected on the next launch without rebuilding:

```bash
# Edit the files
nano src/av_sim/config/obstacles.csv
nano src/av_sim/config/checkpoints.csv

# Relaunch — no build step required
ros2 launch av_sim sim.launch.py
```

---

## Grid and world

| Parameter | Value |
|---|---|
| Grid size | 20 × 20 cells |
| Cell size | 1.0 m |
| World footprint | 20 m × 20 m |
| Cell centre formula | `world_x = (grid_x + 0.5) * cell_size` |
| Robot spawn | world (0, 0) — corner of grid (0, 0) |

---

## ROS2 topic reference

| Topic | Type | Publisher | Description |
|---|---|---|---|
| `/map/obstacles` | `nav_msgs/GridCells` | `map_manager` | Static obstacle grid (1 Hz) |
| `/map/checkpoints` | `geometry_msgs/PoseArray` | `map_manager` | Ordered checkpoint poses (1 Hz) |
| `/map/scan_obstacles` | `nav_msgs/GridCells` | `scan_occupancy` | Live lidar-detected obstacles (5 Hz, 3 s decay) |
| `/planned_path` | `nav_msgs/Path` | `astar_planner` | Current planned path |
| `/cmd_vel` | `geometry_msgs/Twist` | `controller` | Drive commands to robot |
| `/odom` | `nav_msgs/Odometry` | Gazebo bridge | Robot odometry |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo bridge | 360° lidar scan |
| `/current_checkpoint` | `std_msgs/Int32` | `controller` | 1-indexed checkpoint just reached |
| `/mission_complete` | `std_msgs/Bool` | `controller` | `True` when all checkpoints done |
| `/replan_request` | `std_msgs/Empty` | `controller` | Forces immediate replan |
| `/checkpoint_markers` | `visualization_msgs/MarkerArray` | `controller` | Green (pending) / grey (reached) spheres |
| `/lookahead_marker` | `visualization_msgs/Marker` | `controller` | Orange sphere at Pure Pursuit target |

### TF tree

```
map (fixed)
 └── odom          ← static identity transform (sim.launch.py)
      └── base_link ← broadcast from /odom by controller
           ├── laser_link
           ├── left_wheel
           ├── right_wheel
           └── caster_wheel
```

---

## Running tests

```bash
# Using colcon (recommended — mirrors CI)
colcon test --packages-select av_sim
colcon test-result --all --verbose   # show pass/fail summary

# Or directly with pytest
source install/setup.bash
python3 -m pytest src/av_sim/test/ -v
```

**85 tests** across four files:

| File | Coverage |
|---|---|
| `test_astar.py` | A\* correctness, obstacle avoidance, path continuity, grid boundaries |
| `test_planning_utils.py` | `inflate_obstacles`, `prune_path` |
| `test_controller_math.py` | `_normalise`, `_yaw_from_quat`, `heading_error`, `compute_cmd`, Pure Pursuit, `speed_scale_from_scan` |
| `test_scan_math.py` | `scan_hits_to_grid` — coordinate transform, range filtering, grid clipping |

All tests are pure Python with no ROS2 runtime required.

---

## Package structure

```
autonomous_vehicle/              ← workspace root (repo root)
├── src/
│   └── av_sim/                  ← ROS2 package (ament_python)
│       ├── av_sim/
│       │   ├── planning.py          pure A* + inflate_obstacles + prune_path
│       │   ├── control_math.py      pure controller math (Pure Pursuit, speed scaling)
│       │   ├── map_manager.py       loads CSVs → /map/obstacles, /map/checkpoints
│       │   ├── astar_planner.py     reactive planner → /planned_path
│       │   ├── controller.py        Pure Pursuit driver + safety + TF broadcaster
│       │   └── scan_occupancy.py    /scan → /map/scan_obstacles
│       ├── config/
│       │   ├── obstacles.csv
│       │   └── checkpoints.csv
│       ├── launch/
│       │   └── sim.launch.py
│       ├── rviz/
│       │   └── av_sim.rviz
│       ├── test/
│       │   ├── conftest.py
│       │   ├── test_astar.py
│       │   ├── test_planning_utils.py
│       │   ├── test_controller_math.py
│       │   └── test_scan_math.py
│       ├── urdf/
│       │   └── robot.urdf
│       ├── pytest.ini
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── README.md
```

---

## Tuning reference

Key constants that control navigation behaviour (all in source — no parameter server required for now):

| Constant | File | Default | Effect |
|---|---|---|---|
| `LOOKAHEAD_DIST` | `control_math.py` | `1.5` m | Pure Pursuit lookahead — increase for smoother paths, decrease for tighter tracking |
| `MAX_LINEAR` | `control_math.py` | `0.5` m/s | Top speed |
| `STOP_DIST` | `control_math.py` | `0.6` m | Emergency stop threshold (scan) |
| `FULL_SPEED_DIST` | `control_math.py` | `2.0` m | Distance at which full speed is restored |
| `WAYPOINT_RADIUS` | `controller.py` | `0.35` m | Distance at which a waypoint is considered reached |
| `STUCK_TIMEOUT` | `controller.py` | `6.0` s | Time without movement before replan |
| `inflation_radius` | `sim.launch.py` (planner param) | `1` cell | Obstacle padding; set to `0` to disable |

---

## Troubleshooting

**Robot does not move**
- Wait ~5 s after Gazebo opens; application nodes start on a timer.
- Check `ros2 topic echo /planned_path` — if empty, the planner is waiting for obstacles + checkpoints + first odom.
- Check `ros2 topic echo /map/obstacles` — if empty, `map_manager` cannot read its CSV; verify the path.

**RViz2 shows no robot model**
- Confirm `ros2 topic echo /odom` is publishing.
- Run `ros2 run tf2_tools view_frames` and verify `odom → base_link` exists.

**Path never published / "No path" error**
- Inflation radius may be blocking the goal cell. Try `inflation_radius:=0` in the launch or move the checkpoint away from obstacles.
- Verify checkpoint coordinates are inside the grid `[0, 19]` and do not coincide with an obstacle.

**Gazebo blank / crash on WSL2**
```bash
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
ros2 launch av_sim sim.launch.py
```

**GPU lidar not working / no `/scan`**
- Confirm `ignition-gazebo-sensors-system` is available: `ign plugin --list | grep sensors`
- The sensor requires `ogre2`; on pure software rendering it may be unavailable. The rest of the simulation continues without it.