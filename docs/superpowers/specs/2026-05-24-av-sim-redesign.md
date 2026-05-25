# AV Sim Redesign — Self-Driving Car Simulation

**Date:** 2026-05-24
**Status:** Approved for implementation

## Problem Statement

The previous simulation (Ignition Gazebo Fortress + Mesa software rendering in Docker/WSL2) suffered from persistent GPU rendering instability. The GzSceneManager + SceneBroadcaster interaction resets Ogre's scene at every physics step under Mesa, causing unrecoverable viewport flickering. Five separate fix attempts failed. This document specifies a complete rewrite using Gazebo Classic 11, which eliminates the rendering issue architecturally.

## Goals

1. A differential-drive robot navigates through configurable checkpoints in an open field.
2. Obstacles are detected in real time via a CPU lidar sensor.
3. The robot auto-plans an A\* route through checkpoints; an external node can override the path via a ROS 2 topic.
4. Gazebo Classic 11 renders without flickering in Docker on WSL2 with Mesa software rendering.
5. All pure math is unit-testable without ROS or Gazebo.

## Non-Goals

- Ackermann/car-like steering
- SLAM or map building
- Dynamic (moving) obstacles
- Camera-based perception
- GPU rendering / AMD GPU passthrough

---

## System Architecture

```
Docker container (osrf/ros:humble-desktop + gazebo11)
│
├── gzserver / gzclient        — Gazebo Classic 11 physics + GUI
│   └── robot.urdf             — diff-drive plugin + CPU ray sensor
│
├── static_transform_publisher  — map → odom identity TF (simulation has no drift)
│
├── map_manager (ROS 2 node)
│   in:  obstacles.csv, checkpoints.csv (params)
│   out: /map/obstacles (nav_msgs/OccupancyGrid)
│        /checkpoints  (geometry_msgs/PoseArray)
│        /checkpoint_markers (visualization_msgs/MarkerArray)
│
├── astar_planner (ROS 2 node)
│   in:  /map/obstacles, /odom, /checkpoints, /scan
│   out: /planned_path (nav_msgs/Path)
│        /map/inflated (nav_msgs/OccupancyGrid, debug)
│
└── controller (ROS 2 node)
    in:  /planned_path, /override_path, /odom
    out: /cmd_vel (geometry_msgs/Twist)
         /lookahead_marker (visualization_msgs/Marker)
```

---

## Component Specifications

### Gazebo World

- **Field:** 30×30 m flat plane
- **Obstacles:** 1×1×1 m boxes; positions loaded from `config/obstacles.csv` (columns: `grid_x`, `grid_y`; cell size 1 m)
- **Checkpoints:** 0.3 m radius, 0.1 m tall cylinders; positions + order from `config/checkpoints.csv` (columns: `order`, `grid_x`, `grid_y`)
- **Lighting:** Directional light, `cast_shadows false`
- **Physics:** 500 Hz, real-time factor 1.0
- **Rendering:** Ogre 1.x via Gazebo Classic; no GzSceneManager plugin; stable under Mesa GLX software rasterizer

### Robot URDF

- **Chassis:** 0.4 × 0.3 × 0.1 m box, mass 5 kg
- **Drive wheels:** two 0.1 m radius wheels at ±0.17 m from centre, mass 0.5 kg each
- **Caster:** 0.04 m radius sphere, rear, frictionless
- **Gazebo plugin:** `libgazebo_ros_diff_drive.so`
  - Topics: `/cmd_vel` (in), `/odom` (out)
  - Wheel separation: 0.34 m; wheel radius: 0.1 m
  - Publish TF: `odom → base_link`
- **Lidar:** `<sensor type="ray">` (CPU, not GPU)
  - 360° sweep, 180 beams, 8 m range, 10 Hz
  - Plugin: `libgazebo_ros_ray_sensor.so` → topic `/scan` (`sensor_msgs/LaserScan`)
  - Frame: `laser_link` (fixed, centred on chassis top)

### map_manager Node

- **Inputs:** `obstacles_csv` and `checkpoints_csv` ROS 2 parameters
- **Outputs:**
  - `/map/obstacles` — `nav_msgs/OccupancyGrid` (100 = obstacle, 0 = free); latched
  - `/checkpoints` — `geometry_msgs/PoseArray` ordered by `order` column; latched
  - `/checkpoint_markers` — `visualization_msgs/MarkerArray` for Gazebo/RViz debug display; latched
- **Behaviour:** Publishes once on startup; no dynamic updates (static obstacle map)

### astar_planner Node

- **Grid:** configurable width/height/cell_size (default 30×30, 1 m/cell)
- **Inflation radius:** 1 cell around each obstacle in the static map
- **Dynamic layer:** `/scan` hits within 8 m are projected to grid cells and treated as temporary obstacles (cleared each replan)
- **Search:** A\* with 8-directional movement (diagonal cost = √2 × cell_size); post-prune collinear waypoints
- **Fallback:** if inflated path fails, retry with zero inflation; if still no path, publish last known path unchanged and log a warning
- **Replan triggers:**
  1. Checkpoint reached (controller publishes `/checkpoint_reached` std_msgs/Int32 with checkpoint index)
  2. Stuck detection signal from controller (`/replan_request` std_msgs/Empty)
  3. New obstacles detected blocking current path (checked each `/scan` callback)
- **Output:** `nav_msgs/Path` on `/planned_path`; replanned in-place (no restart from origin)

### controller Node

- **Pure Pursuit parameters:**
  - Lookahead distance: 1.5 m (param: `lookahead_distance`)
  - Max linear speed: 0.5 m/s (param: `max_linear`)
  - Max angular speed: 1.2 rad/s (param: `max_angular`)
- **Checkpoint detection radius:** 0.75 m (param: `checkpoint_radius`)
- **Stuck detection:** if displacement < 0.1 m in 6 s → publish `/replan_request`
- **Path source priority:**
  1. `/override_path` — followed when a message arrives; controller publishes `/override_exhausted` (`std_msgs/Empty`) when the last waypoint is reached, then reverts to `/planned_path`
  2. `/planned_path` — default auto-planned route
- **Outputs:** `/cmd_vel`, `/lookahead_marker`, `/checkpoint_reached`, `/replan_request`, `/override_exhausted`

---

## Docker / Deployment

**Base image:** `osrf/ros:humble-desktop`

**Additional packages (installed in Dockerfile):**
```
ros-humble-gazebo-ros-pkgs
python3-pytest
```

**Rendering configuration (docker-compose.yml):**
- `DISPLAY: ${DISPLAY:-:0}` — WSLg X11 forwarding
- `QT_X11_NO_MITSHM: "1"` — Qt shared-memory fix
- `QT_QPA_PLATFORM: xcb`
- **No** `LIBGL_ALWAYS_SOFTWARE: "1"` — Gazebo Classic + Ogre 1 renders correctly with the default Mesa GLX driver; forcing software mode is unnecessary and potentially harmful
- Volume: `/mnt/wslg/.X11-unix:/tmp/.X11-unix:rw`

**Services:**
| Service | Command | Purpose |
|---------|---------|---------|
| `sim` | `ros2 launch av_sim sim.launch.py` | Full sim: `gzclient` GUI + all nodes |
| `sim-headless` | `ros2 launch av_sim sim.launch.py headless:=true` | `gzserver` only; CI |
| `test` | `pytest src/av_sim/test/` | Unit test suite |

**Launch args:**
| Arg | Default | Description |
|-----|---------|-------------|
| `obstacles_csv` | `config/obstacles.csv` | Path to obstacle layout |
| `checkpoints_csv` | `config/checkpoints.csv` | Path to checkpoint list |
| `headless` | `false` | `true` = gzserver only, no GUI |

---

## Data Flow

```
obstacles.csv ──► map_manager ──► /map/obstacles ──► astar_planner ──► /planned_path ──►┐
checkpoints.csv ─►             ──► /checkpoints  ──►                                     │
                                                                                          ▼
/scan ────────────────────────────────────────────────► astar_planner           controller ──► /cmd_vel ──► robot
/odom ────────────────────────────────────────────────────────────────────────► controller
/override_path ────────────────────────────────────────────────────────────────►
                                                        /checkpoint_reached ◄──
                                                        /replan_request ◄──────
```

---

## Testing Strategy

### Unit Tests (pytest, no ROS/Gazebo)

All math lives in importable Python modules with no ROS imports:

| Module | Tests |
|--------|-------|
| `av_sim/astar_math.py` | path correctness on simple grids, obstacle inflation, no-path fallback, diagonal cost, collinear pruning |
| `av_sim/control_math.py` | Pure Pursuit geometry, heading error, lookahead selection, velocity clamping, stuck threshold |
| `av_sim/map_math.py` | CSV parsing, grid↔world coordinate conversions, bounds checking |

### Integration Test (ROS 2, no Gazebo)

- Launch `map_manager` + `astar_planner` with a 5×5 test map (inline param, no CSV)
- Publish a `/odom` pose and trigger planning
- Assert `/planned_path` arrives within 2 s and all waypoints are obstacle-free

### Manual Gazebo Validation

1. `docker compose up sim` → Gazebo Classic GUI opens, no flickering
2. Robot drives from start through all checkpoints in order
3. Publish a custom path to `/override_path` → robot follows it, then reverts to checkpoint planning
4. Confirm `/scan` topic is active (`ros2 topic hz /scan` shows ~10 Hz)

---

## File Structure

```
autonomous_vehicle/
├── docker-compose.yml
├── Dockerfile
├── src/av_sim/
│   ├── package.xml
│   ├── setup.py
│   ├── config/
│   │   ├── obstacles.csv
│   │   └── checkpoints.csv
│   ├── urdf/
│   │   └── robot.urdf
│   ├── launch/
│   │   └── sim.launch.py
│   ├── av_sim/
│   │   ├── __init__.py
│   │   ├── map_math.py        ← pure math, no ROS
│   │   ├── astar_math.py      ← pure math, no ROS
│   │   ├── control_math.py    ← pure math, no ROS
│   │   ├── map_manager.py     ← ROS node
│   │   ├── astar_planner.py   ← ROS node
│   │   └── controller.py      ← ROS node
│   └── test/
│       ├── test_map_math.py
│       ├── test_astar.py
│       └── test_control_math.py
└── docs/superpowers/specs/
    └── 2026-05-24-av-sim-redesign.md   ← this file
```
