# ──────────────────────────────────────────────────────────────────────────────
# av_sim — ROS2 Humble + Gazebo Fortress autonomous vehicle simulation
# ──────────────────────────────────────────────────────────────────────────────
# Base: official ROS2 Humble desktop image (Ubuntu 22.04 + RViz2 + common tools)
FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

# ── 1. Add Gazebo Fortress (OSRF) APT repository ─────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
         -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) \
         signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
         http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" \
         > /etc/apt/sources.list.d/gazebo-stable.list \
    && rm -rf /var/lib/apt/lists/*

# ── 2. Install Gazebo Fortress and ROS–Gazebo integration packages ────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
        ignition-fortress \
        ros-humble-ros-ign-gazebo \
        ros-humble-ros-ign-bridge \
        ros-humble-robot-state-publisher \
        ros-humble-tf2-ros \
        ros-humble-visualization-msgs \
        python3-colcon-common-extensions \
        python3-pytest \
    && rm -rf /var/lib/apt/lists/*

# ── 3. Build the ROS2 workspace ───────────────────────────────────────────────
WORKDIR /ws
COPY src/ src/

RUN . /opt/ros/humble/setup.sh \
    && colcon build \
         --cmake-args -DCMAKE_BUILD_TYPE=Release \
         --event-handlers console_direct+ \
    # Remove build intermediates; install/ is self-contained with a regular build
    && rm -rf build/ log/ src/

# ── 4. Entrypoint ─────────────────────────────────────────────────────────────
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# Default: headless simulation (no RViz2 window).
# Override in docker-compose.yml or with `docker run … ros2 launch av_sim sim.launch.py`
CMD ["ros2", "launch", "av_sim", "sim.launch.py", "no_rviz:=true"]