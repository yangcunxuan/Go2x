#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export DISPLAY="${DISPLAY:-:0}"
xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || true
docker-compose run --rm \
  -e DISPLAY="$DISPLAY" \
  -e ROS_DOMAIN_ID=0 \
  -e CYCLONEDDS_URI=file:///project/config/cyclonedds_go2.xml \
  ros2 bash -lc 'source /opt/ros/humble/setup.bash; rviz2 -d /project/config/go2_3d.rviz'
