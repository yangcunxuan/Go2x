#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
cleanup(){ kill -INT "${SCAN_PID:-}" "${SLAM_PID:-}" "${BRIDGE_PID:-}" 2>/dev/null || true; }
trap cleanup EXIT INT TERM
CONFIG=/project/patrol_data/config.json
SCAN_TOPIC=/scan
ODOM_TOPIC=/utlidar/robot_odom

echo "等待 GO2 狗载雷达 /utlidar/cloud_base 与里程计 ${ODOM_TOPIC} ..."
timeout 20 bash -c 'until ros2 topic list | grep -qx /utlidar/cloud_base; do sleep 1; done'
timeout 20 bash -c 'until ros2 topic list | grep -qx /utlidar/robot_odom; do sleep 1; done'

ODOM_TOPIC="${ODOM_TOPIC}" ros2 run patrol_bridge bridge > /project/runtime/logs/patrol_bridge.log 2>&1 & BRIDGE_PID=$!
sleep 1
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/go2/cloud_base -r scan:="${SCAN_TOPIC}" \
  -p target_frame:=base_link -p transform_tolerance:=0.05 \
  -p min_height:=-0.10 -p max_height:=0.30 \
  -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.00873 \
  -p scan_time:=0.2 -p range_min:=0.30 -p range_max:=20.0 \
  > /project/runtime/logs/go2_cloud_to_scan.log 2>&1 & SCAN_PID=$!

sleep 2
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file /project/config/slam_toolbox_patrol.yaml -r scan:="${SCAN_TOPIC}" \
  > /project/runtime/logs/slam_toolbox.log 2>&1 & SLAM_PID=$!
wait -n "$SCAN_PID" "$SLAM_PID" "$BRIDGE_PID"
