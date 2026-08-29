#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p runtime/logs patrol_data/maps
docker-compose run --rm \
  -e ROS_DOMAIN_ID=0 \
  -e CYCLONEDDS_URI=file:///project/config/cyclonedds_go2.xml \
  ros2 ./scripts/inside_go2_mapping.sh
