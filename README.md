# Autonomous Vehicle Simulation

A ROS2 Humble simulation of a differential-drive robot that uses A\* path planning to navigate through a sequence of checkpoints on a 2D grid while avoiding obstacles. The robot and environment run in Gazebo Fortress (Ignition Gazebo 6).

---

## System Requirements

| Component | Version |
|-----------|---------|
| OS | Ubuntu 22.04 LTS (WSL2 supported) |
| ROS2 | Humble Hawksbill |
| Simulator | Gazebo Fortress (Ignition Gazebo 6) |
| Python | 3.10+ (ships with Ubuntu 22.04) |

Required ROS2 packages (installable via `apt`):

```bash
sudo apt install \
  ros-humble-robot-state-publisher \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge
```

---

## Build

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash

# 2. Navigate to the workspace root (the repo root)
cd ~/autonomous_vehicle

# 3. Build
colcon build --symlink-install

# 4. Source the install overlay
source install/setup.bash
```

---

## Launch

```bash
ros2 launch av_sim sim.launch.py
```

The launch file:
1. Reads `obstacles.csv` and `checkpoints.csv` from the installed `config/` directory.
2. Generates a Gazebo world SDF on the fly (written to `/tmp/av_sim_world.sdf`) containing the ground plane, red obstacle boxes, and green checkpoint cylinders.
3. Starts Gazebo Fortress with that world.
4. Spawns the differential-drive robot at grid cell (1, 1).
5. Bridges `/cmd_vel`, `/odom`, and `/clock` between ROS2 and Gazebo.
6. Launches all three application nodes: map manager, A\* planner, and controller.

The robot begins driving autonomously once the planner has received obstacle and checkpoint data from the map manager and an initial odometry reading.

---

## Grid and World Dimensions

| Parameter | Value |
|-----------|-------|
| Grid size | 20 × 20 cells |
| Cell size | 1.0 m × 1.0 m |
| World footprint | 20 m × 20 m |
| Grid origin (0, 0) | world (0, 0) — cell centre at (0.5, 0.5) |
| Robot spawn | grid (1, 1) → world (1.5, 1.5) |

---

## Swapping in Custom Obstacles and Checkpoints

Both data files live in `src/av_sim/config/`. Edit them, then rebuild and relaunch.

### `config/obstacles.csv`

One row per impassable grid cell:

```
grid_x,grid_y
3,2
5,7
```

### `config/checkpoints.csv`

One row per checkpoint. The robot visits them in `order` sequence:

```
order,grid_x,grid_y
1,5,3
2,12,4
3,18,17
```

**Rules:**
- Grid coordinates are integers in `[0, 19]` for both axes.
- A checkpoint must not coincide with an obstacle cell.
- The A\* planner will log an error and skip planning if no path exists between two consecutive waypoints.

After editing, run:

```bash
colcon build --symlink-install   # only needed if you did not use --symlink-install initially
source install/setup.bash
ros2 launch av_sim sim.launch.py
```

If `--symlink-install` was used at build time, changes to the CSV files take effect on the next launch without rebuilding.

---

## Package Structure

```
autonomous_vehicle/          ← workspace root (repo root)
├── src/
│   └── av_sim/              ← single ROS2 package (ament_python)
│       ├── av_sim/
│       │   ├── map_manager.py    ← loads CSVs; publishes /map/obstacles, /map/checkpoints
│       │   ├── astar_planner.py  ← A* over grid; publishes /planned_path
│       │   └── controller.py     ← follows path; publishes /cmd_vel
│       ├── config/
│       │   ├── obstacles.csv
│       │   └── checkpoints.csv
│       ├── launch/
│       │   └── sim.launch.py     ← single launch entry-point
│       ├── urdf/
│       │   └── robot.urdf        ← differential-drive robot with Ignition DiffDrive plugin
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
└── README.md
```

### Node summary

| Node | Publishes | Subscribes |
|------|-----------|------------|
| `map_manager` | `/map/obstacles` (GridCells), `/map/checkpoints` (PoseArray) | — |
| `astar_planner` | `/planned_path` (Path) | `/map/obstacles`, `/map/checkpoints`, `/odom` |
| `controller` | `/cmd_vel` (Twist) | `/planned_path`, `/odom` |
