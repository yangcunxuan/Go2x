#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
OUT="${HOME}/桌面/GO2X设备检测_$(date +%Y%m%d_%H%M%S).txt"
{
  echo "GO2X 巡逻系统设备检测"
  echo "时间：$(date '+%F %T')"
  echo
  echo "=== 网络接口 ==="
  ip -br addr
  echo
  echo "=== 路由 ==="
  ip route
  echo
  echo "=== ROS 2 节点（等待 8 秒发现） ==="
  timeout 12 docker-compose run --rm -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI=file:///project/config/cyclonedds_go2.xml ros2 bash -lc 'source /opt/ros/humble/setup.bash; ros2 node list' || true
  echo
  echo "=== ROS 2 话题和类型 ==="
  timeout 12 docker-compose run --rm -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI=file:///project/config/cyclonedds_go2.xml ros2 bash -lc 'source /opt/ros/humble/setup.bash; ros2 topic list -t' || true
  echo
  echo "GO2 重点话题：/utlidar/cloud_base、/utlidar/robot_odom、/utlidar/robot_pose"
} | tee "$OUT"
echo "检测结果已保存：$OUT"
read -r -p "按回车关闭..." _
