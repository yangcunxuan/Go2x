#!/usr/bin/env bash
set -euo pipefail
set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u
export AMENT_PREFIX_PATH="/project/ros2_ws/install/patrol_bridge:${AMENT_PREFIX_PATH:-}"
PIDS=(); cleanup(){ kill -INT "${PIDS[@]}" 2>/dev/null || true; }; trap cleanup EXIT INT TERM
MAP_YAML=$(ls -1t /project/patrol_data/maps/*.yaml 2>/dev/null | head -1 || true)
if [ -z "$MAP_YAML" ]; then echo '没有已保存地图，请先建图并保存地图。'; exit 2; fi
SENSOR=$(python3 -c 'import json;print(json.load(open("/project/patrol_data/config.json")).get("sensor","go2"))' 2>/dev/null || echo go2)
ODOM_TOPIC=$(python3 -c 'import json;print(json.load(open("/project/patrol_data/config.json")).get("odom_topic","/odom"))' 2>/dev/null || echo /odom)
if [ "$SENSOR" = go2 ]; then
  ODOM_TOPIC=/utlidar/robot_odom
  timeout 20 bash -c 'until ros2 topic list | grep -qx /utlidar/cloud_base; do sleep 1; done'
fi
ODOM_TOPIC="$ODOM_TOPIC" ENABLE_NAV_MOTION=1 ros2 run patrol_bridge bridge > /project/runtime/logs/patrol_bridge.log 2>&1 & PIDS+=($!)
sleep 1
if [ "$SENSOR" = go2 ]; then
  timeout 10 bash -c 'until ros2 topic list | grep -qx /go2/cloud_base; do sleep 1; done'
  ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args \
    -r cloud_in:=/go2/cloud_base -r scan:=/scan \
    -p target_frame:=base_link -p transform_tolerance:=0.05 \
    -p min_height:=-0.10 -p max_height:=0.30 \
    -p angle_min:=-3.14159 -p angle_max:=3.14159 -p angle_increment:=0.00873 \
    -p scan_time:=0.2 -p range_min:=0.30 -p range_max:=20.0 \
    > /project/runtime/logs/go2_nav_cloud_to_scan.log 2>&1 & PIDS+=($!)
fi
sleep 2
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=false autostart:=true \
  use_composition:=False map:="$MAP_YAML" params_file:=/project/config/nav2_go2.yaml \
  > /project/runtime/logs/nav2.log 2>&1 & PIDS+=($!)
wait -n "${PIDS[@]}"
