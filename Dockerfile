# Docker Hub is not reachable from the validation host. DaoCloud mirrors the
# official multi-arch ROS image; replace this with ros:humble-ros-base-jammy
# when building from a network that can reach Docker Hub directly.
FROM docker.m.daocloud.io/library/ros:humble-ros-base-jammy

ARG USER_UID=1000
ARG USER_GID=1000
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i 's|http://archive.ubuntu.com/ubuntu|https://mirrors.aliyun.com/ubuntu|g; s|http://security.ubuntu.com/ubuntu|https://mirrors.aliyun.com/ubuntu|g' /etc/apt/sources.list

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    cmake \
    git \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rviz2 \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-tf2-ros \
    && rm -rf /var/lib/apt/lists/*

# Global localization stack (Plan A): small_gicp provides multithreaded
# GICP/VGICP refinement for the Scan Context -> GICP relocalization chain.
# Primary install from the aliyun PyPI mirror (consistent with apt above);
# a plain PyPI attempt is the fallback and failure is tolerated so image
# builds never break on network flakiness — the localization manager
# falls back to open3d if small_gicp is absent at runtime.
RUN pip3 install --no-cache-dir -i https://mirrors.aliyun.com/pypi/simple/ small_gicp     || pip3 install --no-cache-dir small_gicp     || echo "WARNING: small_gicp unavailable; localization falls back to open3d"


# Livox SDK2 is vendored from pinned commit 08f523c930b2f0ba1e98a6afaa8d7476bf479908
# so image builds do not depend on a live GitHub connection.
COPY vendor/Livox-SDK2 /tmp/Livox-SDK2
RUN cd /tmp/Livox-SDK2 \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
    && cmake --build build --parallel 2 \
    && cmake --install build \
    && rm -rf /tmp/Livox-SDK2

RUN if ! getent group "${USER_GID}" >/dev/null; then groupadd --gid "${USER_GID}" ros; fi \
    && useradd --uid "${USER_UID}" --gid "${USER_GID}" --create-home --shell /bin/bash ros

ENV ROS_DISTRO=humble \
    RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ROS_DOMAIN_ID=42 \
    LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh && ldconfig

USER ros
WORKDIR /project
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
