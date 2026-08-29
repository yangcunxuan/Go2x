#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${ROOT_DIR}/ros2_ws/src"
mkdir -p "${SRC_DIR}"

if [ ! -d "${SRC_DIR}/livox_ros_driver2/.git" ]; then
  git clone https://github.com/Livox-SDK/livox_ros_driver2.git "${SRC_DIR}/livox_ros_driver2"
fi
git -C "${SRC_DIR}/livox_ros_driver2" fetch --tags origin
git -C "${SRC_DIR}/livox_ros_driver2" checkout --detach 13eb05e4e6dd7a765b934d0c5fd6236676a57b49
cp "${SRC_DIR}/livox_ros_driver2/package_ROS2.xml" "${SRC_DIR}/livox_ros_driver2/package.xml"
rm -rf "${SRC_DIR}/livox_ros_driver2/launch"
ln -s launch_ROS2 "${SRC_DIR}/livox_ros_driver2/launch"

if [ ! -d "${SRC_DIR}/FAST_LIO_ROS2/.git" ]; then
  git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git "${SRC_DIR}/FAST_LIO_ROS2"
fi
git -C "${SRC_DIR}/FAST_LIO_ROS2" fetch origin ros2
git -C "${SRC_DIR}/FAST_LIO_ROS2" checkout --detach 2fffc570a25d0df172720bac034fbdb6a13d2162
git -C "${SRC_DIR}/FAST_LIO_ROS2" submodule update --init --recursive

echo "Pinned ROS 2 sources are ready in ${SRC_DIR}"
