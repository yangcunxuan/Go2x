#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u

STAMP="$(date +%Y%m%d_%H%M%S)"
DRIVER_LOG="/tmp/mid360_mapping_driver_${STAMP}.log"
MAPPING_LOG="/tmp/mid360_fastlio_${STAMP}.log"
BAG_DIR="/project/bags/mapping_validation_${STAMP}"
DRIVER_PID=""
MAPPING_PID=""

stop_tree() {
  local pid="${1:-}"
  if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
    pkill -INT -P "${pid}" 2>/dev/null || true
    kill -INT "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  stop_tree "${MAPPING_PID}"
  stop_tree "${DRIVER_PID}"
  sleep 1
  pkill -TERM -f fastlio_mapping 2>/dev/null || true
  pkill -TERM -f livox_ros_driver2_node 2>/dev/null || true
}
trap cleanup EXIT INT TERM

mkdir -p /project/bags
./scripts/run_driver.sh >"${DRIVER_LOG}" 2>&1 &
DRIVER_PID=$!
sleep 6

RVIZ=false ./scripts/run_mapping.sh >"${MAPPING_LOG}" 2>&1 &
MAPPING_PID=$!
sleep 10

echo "=== 建图相关话题 ==="
ros2 topic list -t | grep -E 'livox|Odometry|path|cloud|Laser|tf' || true
echo "=== FAST-LIO2 节点 ==="
ros2 node list
echo "=== 里程计频率（10 秒） ==="
timeout --signal=INT 10s ros2 topic hz /Odometry || true
echo "=== 一帧里程计位置 ==="
timeout 5s ros2 topic echo /Odometry --once --field pose.pose.position || true
echo "=== 录制 10 秒 MCAP 验证包 ==="
timeout --signal=INT 10s ros2 bag record --storage mcap \
  -o "${BAG_DIR}" /livox/lidar /livox/imu /Odometry /path \
  /cloud_registered /cloud_registered_body /Laser_map || true
ros2 bag info "${BAG_DIR}" || true
echo "=== FAST-LIO2 日志 ==="
sed -n '1,240p' "${MAPPING_LOG}"
echo "=== Livox 驱动日志 ==="
sed -n '1,160p' "${DRIVER_LOG}"
