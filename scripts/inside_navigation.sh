#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u

export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
ENABLE_FILE=/project/runtime/nav_motion_enable.json
PIDS=()
NAMES=()
cleanup() {
  printf '{"enabled":false,"expires_at":0}\n' > "$ENABLE_FILE"
  kill -INT "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

MAP_YAML=/project/runtime/navigation_map.yaml
if [ ! -f "$MAP_YAML" ]; then
  echo '没有PCD生成的导航地图，请先保存MID360三维地图。' >&2
  exit 2
fi
TOPICS=$(ros2 topic list)
if ! grep -Fxq /Odometry <<<"$TOPICS" || ! grep -Fxq /cloud_registered_body <<<"$TOPICS"; then
  echo 'FAST-LIO尚未运行：导航必须保持MID360三维建图/定位服务在线。' >&2
  exit 3
fi

# Every navigation session starts motion-disabled. The web UI must explicitly
# confirm a target before writing a short-lived enable lease.
printf '{"enabled":false,"expires_at":0}\n' > "$ENABLE_FILE"

# Ignore only returns inside the robot's own 0.36 m front footprint.  The
# user's real obstacle at about 0.50 m must remain visible for replanning.
# body 原点位于机身附近；-0.35 m 会把地面投影成约 0.40 m 的假障碍。
# 只保留高于地面、可能碰到机身的点。
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
  -r cloud_in:=/cloud_registered_body -r scan:=/scan \
  -p target_frame:=body -p transform_tolerance:=0.10 \
  -p min_height:=-0.15 -p max_height:=0.75 \
  -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.00873 \
  -p scan_time:=0.10 -p range_min:=0.40 -p range_max:=20.0 \
  > /project/runtime/logs/mid360_nav_cloud_to_scan.log 2>&1 &
PIDS+=($!); NAMES+=(cloud_to_scan)

ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:="$MAP_YAML" -p frame_id:=map_level \
  > /project/runtime/logs/nav_map_server.log 2>&1 &
PIDS+=($!); NAMES+=(map_server)

ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p node_names:="[map_server]" \
  > /project/runtime/logs/nav_map_lifecycle.log 2>&1 &
PIDS+=($!); NAMES+=(map_lifecycle)

python3 /project/scripts/nav_motion_bridge.py \
  > /project/runtime/logs/nav_motion_bridge.log 2>&1 &
PIDS+=($!); NAMES+=(nav_motion_bridge)

# Compatibility alias for bridge versions that published navigation goals in
# `map`. The MID360 navigation stack uses `map_level` as its global frame.
ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 \
  --frame-id map_level --child-frame-id map \
  > /project/runtime/logs/nav_map_frame_alias.log 2>&1 &
PIDS+=($!); NAMES+=(map_frame_alias)

sleep 2
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=false autostart:=true \
  use_composition:=False params_file:=/project/config/nav2_mid360.yaml \
  > /project/runtime/logs/nav2.log 2>&1 &
PIDS+=($!); NAMES+=(nav2)

set +e
wait -n "${PIDS[@]}"
code=$?
set -e
for index in "${!PIDS[@]}"; do
  if ! kill -0 "${PIDS[$index]}" 2>/dev/null; then
    echo "导航子进程退出：${NAMES[$index]}，退出码：$code" >&2
  fi
done
exit "$code"
