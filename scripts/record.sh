#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
mkdir -p /project/bags
ros2 bag record --storage mcap \
  -o "/project/bags/mid360_${STAMP}" \
  /livox/lidar /livox/imu /tf /tf_static /Odometry /path
