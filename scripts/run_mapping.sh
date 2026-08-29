#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u

mkdir -p /project/maps
ros2 launch fast_lio mapping.launch.py \
  config_path:=/project/config \
  config_file:="${FASTLIO_CONFIG:-fastlio_mid360.yaml}" \
  rviz:="${RVIZ:-true}"
