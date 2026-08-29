#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
PIDS=(); NAMES=(); cleanup(){ kill -INT "${PIDS[@]}" 2>/dev/null || true; }; trap cleanup EXIT INT TERM
/project/scripts/run_driver.sh > /project/runtime/logs/livox_driver.log 2>&1 & PIDS+=($!); NAMES+=(livox_driver)
RVIZ=false /project/scripts/run_mapping.sh > /project/runtime/logs/fastlio.log 2>&1 & PIDS+=($!); NAMES+=(fastlio)
sleep 5
# Level the FAST-LIO initial frame for the current mechanically tilted MID360.
# Recalibrate these angles if the sensor mount changes.
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 \
  --roll -0.030788 --pitch 0.621767 --yaw 0 \
  --frame-id map_level --child-frame-id camera_init \
  > /project/runtime/logs/mid360_level_tf.log 2>&1 & PIDS+=($!); NAMES+=(level_tf)
# Three-dimensional mapping mode deliberately does not start
# pointcloud_to_laserscan or slam_toolbox. Those 2-D nodes belong to the
# navigation preparation path, not the MID360 FAST-LIO mapping session.
ODOM_TOPIC=/Odometry ros2 run patrol_bridge bridge > /project/runtime/logs/patrol_bridge.log 2>&1 & PIDS+=($!); NAMES+=(patrol_bridge)
set +e
wait -n "${PIDS[@]}"; code=$?
set -e
for index in "${!PIDS[@]}"; do
  if ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
    echo "三维建图子进程退出：${NAMES[$index]}，退出码：$code" >&2
  fi
done
exit "$code"
