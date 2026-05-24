#!/bin/bash
# Container entrypoint — sources ROS2 and the built workspace, then runs CMD.
set -e

source /opt/ros/humble/setup.bash
source /ws/install/setup.bash

exec "$@"