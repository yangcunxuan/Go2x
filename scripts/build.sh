#!/usr/bin/env bash
set -euo pipefail

cd /project/ros2_ws
colcon build \
  --symlink-install \
  --parallel-workers 2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
