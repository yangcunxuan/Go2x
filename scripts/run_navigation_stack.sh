#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p runtime/logs patrol_data/maps
SENSOR=$(python3 -c 'import json;print(json.load(open("patrol_data/config.json")).get("sensor","go2"))' 2>/dev/null || echo go2)
if [ "$SENSOR" = go2 ]; then
  docker-compose run --rm \
    -e ROS_DOMAIN_ID=0 \
    -e CYCLONEDDS_URI=file:///project/config/cyclonedds_go2.xml \
    ros2 ./scripts/inside_navigation.sh
else
  docker-compose run --rm ros2 ./scripts/inside_navigation.sh
fi
