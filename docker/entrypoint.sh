#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
if [ -f /project/ros2_ws/install/setup.bash ]; then
  source /project/ros2_ws/install/setup.bash
fi

exec "$@"
