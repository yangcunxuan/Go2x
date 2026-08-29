#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u

LOG_FILE="/tmp/mid360_driver_$(date +%Y%m%d_%H%M%S).log"
DRIVER_PID=""

cleanup() {
  if [ -n "${DRIVER_PID}" ] && kill -0 "${DRIVER_PID}" 2>/dev/null; then
    pkill -INT -P "${DRIVER_PID}" 2>/dev/null || true
    kill -INT "${DRIVER_PID}" 2>/dev/null || true
    sleep 1
    pkill -TERM -P "${DRIVER_PID}" 2>/dev/null || true
    kill -TERM "${DRIVER_PID}" 2>/dev/null || true
    wait "${DRIVER_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

./scripts/run_driver.sh >"${LOG_FILE}" 2>&1 &
DRIVER_PID=$!
sleep 6

if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
  echo "MID360 驱动提前退出："
  sed -n '1,200p' "${LOG_FILE}"
  exit 1
fi

echo "=== ROS 节点 ==="
ros2 node list
echo "=== ROS 话题 ==="
ros2 topic list -t
echo "=== 点云发布信息 ==="
ros2 topic info /livox/lidar -v || true
echo "=== IMU 发布信息 ==="
ros2 topic info /livox/imu -v || true
echo "=== 点云频率（10 秒） ==="
timeout --signal=INT 10s ros2 topic hz /livox/lidar || true
echo "=== IMU 频率（10 秒） ==="
timeout --signal=INT 10s ros2 topic hz /livox/imu || true
echo "=== 点云带宽（6 秒） ==="
timeout --signal=INT 6s ros2 topic bw /livox/lidar || true
echo "=== 驱动日志 ==="
sed -n '1,200p' "${LOG_FILE}"
