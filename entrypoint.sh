#!/bin/bash
set -e

# Source the main ROS 2 setup file to activate the ROS environment
source /opt/ros/humble/setup.bash

# Source the local workspace's setup file if it has been built.
# This allows you to use your workspace's packages and executables.
if [ -f /ros2_ws/install/setup.bash ]; then
  source /ros2_ws/install/setup.bash
fi

# Execute the command passed into the docker container (e.g., "bash", "colcon build")
exec "$@"
