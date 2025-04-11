#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/humble/setup.bash

# Build the workspace if the package isn't built yet
if [ ! -d "/ros2_ws/install/ros2_ndi_tracker" ]; then
  echo "Building ROS2 workspace..."
  cd /ros2_ws
  colcon build --symlink-install
  echo "Workspace built successfully!"
fi

# Source the workspace
source /ros2_ws/install/setup.bash

# Execute the command passed to the entrypoint
exec "$@"