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
