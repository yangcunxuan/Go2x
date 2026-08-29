#!/usr/bin/env bash
# Mapping mode entry: build a NEW 3D map with FAST-LIO (map_en=true).
#
#   Livox driver -> FAST-LIO mapping -> static leveling TF -> keyframe_saver
#   -> patrol_bridge (state/cloud)
#
# The localization-mode twin is inside_mid360_localization.sh. The two modes
# are mutually exclusive: only one FAST-LIO may own the lidar at a time.
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"

# Duplicate-start guard MUST run before any driver is launched: after our own
# FAST-LIO boots it would see its own /Odometry and wrongly abort (P0 audit #1).
if ros2 topic list 2>/dev/null | grep -Fxq /Odometry; then
  echo '检测到 /Odometry 已存在：MID360/FAST-LIO 栈已在运行，拒绝重复启动。' >&2
  exit 3
fi

# map_level for a NEW map is anchored at the boot pose with only the fixed
# mechanical mount angles (P0 audit: stale alignment offsets must not leak
# into new maps). localization_alignment.json is legacy/debug only.
MAP_LEVEL_X=0 MAP_LEVEL_Y=0 MAP_LEVEL_Z=0 MAP_LEVEL_YAW=0
export MAP_LEVEL_X MAP_LEVEL_Y MAP_LEVEL_Z MAP_LEVEL_YAW
PIDS=(); NAMES=(); cleanup(){ kill -INT "${PIDS[@]}" 2>/dev/null || true; }; trap cleanup EXIT INT TERM

echo "模式: mapping（map_en=true，累积图用于保存）"
# Mapping mode persists the marker in runtime so the web can show which mode
# the running perception stack is in.
printf '{"mode":"mapping","updated_at":%s}\n' "$(date +%s)" \
  > /project/runtime/perception_mode.json.tmp && \
  mv /project/runtime/perception_mode.json.tmp /project/runtime/perception_mode.json

/project/scripts/run_driver.sh > /project/runtime/logs/livox_driver.log 2>&1 & PIDS+=($!); NAMES+=(livox_driver)
RVIZ=false FASTLIO_CONFIG=fastlio_mapping.yaml /project/scripts/run_mapping.sh > /project/runtime/logs/fastlio.log 2>&1 & PIDS+=($!); NAMES+=(fastlio)
sleep 5
# Level the FAST-LIO initial frame for the mechanically tilted MID360.
# Mapping mode ALWAYS owns the static map_level -> camera_init TF (the
# localization twin gets its TF from the localization manager instead).
ros2 run tf2_ros static_transform_publisher \
  --x "$MAP_LEVEL_X" --y "$MAP_LEVEL_Y" --z "$MAP_LEVEL_Z" \
  --roll -0.030788 --pitch 0.621767 --yaw "$MAP_LEVEL_YAW" \
  --frame-id map_level --child-frame-id camera_init \
  > /project/runtime/logs/mid360_level_tf.log 2>&1 & PIDS+=($!); NAMES+=(level_tf)
# Three-dimensional mapping mode deliberately does not start
# pointcloud_to_laserscan or slam_toolbox. Those 2-D nodes belong to the
# navigation preparation path, not the MID360 FAST-LIO mapping session.
# Record the mapping trajectory for the global-localization database (Plan A).
MAPPING_SESSION_ID=$(python3 -c 'import json
try: print(json.load(open("/project/runtime/mapping_session.json")).get("id","unknown"))[:8]
except Exception: print("unknown")')
PYTHONPATH=/project/ros2_ws/src/patrol_global_localization python3 -m patrol_global_localization.keyframe_saver >> /project/runtime/logs/keyframe_saver.log 2>&1 & PIDS+=($!); NAMES+=(keyframe_saver)
CLOUD_MAP_TOPIC=/__disabled ODOM_TOPIC=/Odometry ros2 run patrol_bridge bridge > /project/runtime/logs/patrol_bridge.log 2>&1 & PIDS+=($!); NAMES+=(patrol_bridge)
mkdir -p /project/runtime/cloud_bridge
PATROL_RUNTIME=/project/runtime/cloud_bridge ODOM_TOPIC=/__disabled \
  CYCLONEDDS_URI=file:///project/config/cyclonedds.xml \
  ros2 run patrol_bridge bridge --ros-args -r __node:=patrol_cloud_bridge \
  > /project/runtime/logs/patrol_cloud_bridge.log 2>&1 & PIDS+=($!); NAMES+=(patrol_cloud_bridge)
set +e
wait -n "${PIDS[@]}"; code=$?
set -e
for index in "${!PIDS[@]}"; do
  if ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
    echo "三维建图子进程退出：${NAMES[$index]}，退出码：$code" >&2
  fi
done
exit "$code"
