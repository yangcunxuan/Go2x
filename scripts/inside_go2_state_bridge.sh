#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
set -u
cd /project/ros2_ws
colcon build --packages-select patrol_bridge --symlink-install
set +u
source install/setup.bash
set -u
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
exec ros2 run patrol_bridge go2_state_bridge
