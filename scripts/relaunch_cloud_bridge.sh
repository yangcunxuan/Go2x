#!/usr/bin/env bash
# Relaunch ONLY the cloud bridge (patrol_cloud_bridge) inside a running
# perception container. Never touches the main bridge or FAST-LIO, so the
# supervisor's wait -n is not disturbed and the session frame is preserved.
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
ALIGNMENT_FILE=/project/runtime/localization_alignment.json
read -r MAP_LEVEL_X MAP_LEVEL_Y MAP_LEVEL_Z MAP_LEVEL_YAW < <(python3 -c 'import json,sys
try: d=json.load(open(sys.argv[1]))
except Exception: d={}
print(float(d.get("x",0)),float(d.get("y",0)),float(d.get("z",0)),float(d.get("yaw",0)))' "$ALIGNMENT_FILE")
export MAP_LEVEL_X MAP_LEVEL_Y MAP_LEVEL_Z MAP_LEVEL_YAW
PATROL_RUNTIME=/project/runtime/cloud_bridge ODOM_TOPIC=/__disabled \
  CYCLONEDDS_URI=file:///project/config/cyclonedds.xml \
  ros2 run patrol_bridge bridge --ros-args -r __node:=patrol_cloud_bridge \
  >> /project/runtime/logs/patrol_cloud_bridge.log 2>&1
