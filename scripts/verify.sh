#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/humble/setup.bash
source /project/ros2_ws/install/setup.bash
set -u

echo "ROS_DISTRO=${ROS_DISTRO}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
ros2 pkg executables livox_ros_driver2
ros2 interface show livox_ros_driver2/msg/CustomMsg | sed -n '1,30p'
ros2 launch fast_lio mapping.launch.py --show-args | sed -n '1,120p'
