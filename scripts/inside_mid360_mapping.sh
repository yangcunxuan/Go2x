#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
ALIGNMENT_FILE=/project/runtime/localization_alignment.json
read -r MAP_LEVEL_X MAP_LEVEL_Y MAP_LEVEL_Z MAP_LEVEL_YAW < <(python3 -c 'import json,sys; p=sys.argv[1];
try: d=json.load(open(p))
except Exception: d={}
print(float(d.get("x",0)),float(d.get("y",0)),float(d.get("z",0)),float(d.get("yaw",0)))' "$ALIGNMENT_FILE")
export MAP_LEVEL_X MAP_LEVEL_Y MAP_LEVEL_Z MAP_LEVEL_YAW
PIDS=(); NAMES=(); cleanup(){ kill -INT "${PIDS[@]}" 2>/dev/null || true; }; trap cleanup EXIT INT TERM
/project/scripts/run_driver.sh > /project/runtime/logs/livox_driver.log 2>&1 & PIDS+=($!); NAMES+=(livox_driver)
RVIZ=false /project/scripts/run_mapping.sh > /project/runtime/logs/fastlio.log 2>&1 & PIDS+=($!); NAMES+=(fastlio)
sleep 5
# Level the FAST-LIO initial frame for the current mechanically tilted MID360.
# Recalibrate these angles if the sensor mount changes.
ros2 run tf2_ros static_transform_publisher \
  --x "$MAP_LEVEL_X" --y "$MAP_LEVEL_Y" --z "$MAP_LEVEL_Z" \
  --roll -0.030788 --pitch 0.621767 --yaw "$MAP_LEVEL_YAW" \
  --frame-id map_level --child-frame-id camera_init \
  > /project/runtime/logs/mid360_level_tf.log 2>&1 & PIDS+=($!); NAMES+=(level_tf)
# Three-dimensional mapping mode deliberately does not start
# pointcloud_to_laserscan or slam_toolbox. Those 2-D nodes belong to the
# navigation preparation path, not the MID360 FAST-LIO mapping session.
# Keep low-latency pose/teleoperation independent from the large accumulated
# point cloud.  The cloud process is explicitly pinned to the wired interface,
# so Wi-Fi roaming cannot strand its DDS subscription.

# Re-entry guard: a second FAST-LIO/lidar stack would fight over the lidar
# Ethernet stream and exhaust memory. Refuse if odometry is already flowing.
if ros2 topic list 2>/dev/null | grep -Fxq /Odometry; then
  echo '检测到 /Odometry 已存在：MID360/FAST-LIO 栈已在运行，拒绝重复启动。' >&2
  exit 3
fi

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
