#!/bin/bash

# 设置 ROS 2 环境
source /opt/ros/humble/setup.sh
source /ros2_ws/install/setup.sh

# 检查是否需要重新构建工作空间
if [ ! -d "/ros2_ws/install" ]; then
  echo "Building ROS 2 workspace..."
  colcon build --symlink-install
  source /ros2_ws/install/setup.sh
fi

# 执行传入的命令
echo "Starting command: $@"
exec "$@"