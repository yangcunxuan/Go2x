#!/usr/bin/env bash
# Localization mode entry: run FAST-LIO odometry (map_en=false) and anchor
# it to a saved map with the global localization manager (Plan A).
#
#   Livox driver -> FAST-LIO odometry -> localization_manager (dynamic TF)
#   -> patrol_bridge (state/cloud)
#
# Differences from the mapping twin (inside_mid360_mapping.sh):
#   - fastlio_localization.yaml: map_en=false, no accumulated-map publishing
#   - NO static map_level -> camera_init TF: the localization manager is the
#     single publisher of that transform (P0 audit #10 / final plan §8)
#   - localization_manager instead of keyframe_saver
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"

# Duplicate-start guard MUST run before any driver is launched.
if ros2 topic list 2>/dev/null | grep -Fxq /Odometry; then
  echo '检测到 /Odometry 已存在：MID360/FAST-LIO 栈已在运行，拒绝重复启动。' >&2
  exit 3
fi

PIDS=(); NAMES=(); cleanup(){ kill -INT "${PIDS[@]}" 2>/dev/null || true; }; trap cleanup EXIT INT TERM

echo "模式: localization（map_en=false，动态TF由 localization_manager 发布）"
printf '{"mode":"localization","updated_at":%s}\n' "$(date +%s)" \
  > /project/runtime/perception_mode.json.tmp && \
  mv /project/runtime/perception_mode.json.tmp /project/runtime/perception_mode.json

/project/scripts/run_driver.sh > /project/runtime/logs/livox_driver.log 2>&1 & PIDS+=($!); NAMES+=(livox_driver)
RVIZ=false FASTLIO_CONFIG=fastlio_localization.yaml /project/scripts/run_mapping.sh > /project/runtime/logs/fastlio.log 2>&1 & PIDS+=($!); NAMES+=(fastlio)
sleep 5
# No static map_level -> camera_init TF here: localization_manager publishes
# the dynamic transform once Scan Context + GICP converge (UNINITIALIZED
# before that; navigation is blocked by the motion bridge gate).

CLOUD_MAP_TOPIC=/__disabled ODOM_TOPIC=/Odometry ros2 run patrol_bridge bridge > /project/runtime/logs/patrol_bridge.log 2>&1 & PIDS+=($!); NAMES+=(patrol_bridge)
mkdir -p /project/runtime/cloud_bridge
PATROL_RUNTIME=/project/runtime/cloud_bridge ODOM_TOPIC=/__disabled \
  CYCLONEDDS_URI=file:///project/config/cyclonedds.xml \
  ros2 run patrol_bridge bridge --ros-args -r __node:=patrol_cloud_bridge \
  > /project/runtime/logs/patrol_cloud_bridge.log 2>&1 & PIDS+=($!); NAMES+=(patrol_cloud_bridge)
PYTHONPATH=/project/ros2_ws/src/patrol_global_localization \
  python3 -m patrol_global_localization.localization_manager \
  >> /project/runtime/logs/localization_manager.log 2>&1 & PIDS+=($!); NAMES+=(localization_manager)
set +e
wait -n "${PIDS[@]}"; code=$?
set -e
for index in "${!PIDS[@]}"; do
  if ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
    echo "定位栈子进程退出：${NAMES[$index]}，退出码：$code" >&2
  fi
done
exit "$code"
