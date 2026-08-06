# syntax=docker/dockerfile:1.7
FROM --platform=linux/arm64 ros:humble-ros-base-jammy AS validator

ARG DEBIAN_FRONTEND=noninteractive
ARG FULL_VENDOR=0
ENV ROS_DISTRO=humble \
    CAR_HUMBLE_BUILD_ROOT=/tmp/huichuan_agv_humble_ws \
    CAR_VALIDATION_ROS_DOMAIN_ID=187 \
    PYTHONNOUSERSITE=1

RUN apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      file \
      git \
      binutils \
      python3-colcon-common-extensions \
      python3-empy \
      python3-lark \
      python3-pip \
      python3-rosdep \
      python3-yaml \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

RUN test "$(uname -m)" = aarch64 \
    && test "$(. /etc/os-release && printf '%s' "$VERSION_ID")" = 22.04

RUN (rosdep init 2>/dev/null || true) \
    && rosdep update

# Keep dependency installation independent from ordinary source edits. Only a
# package.xml change should invalidate rosdep; Python/C++ changes reuse it.
COPY lidar/chapt1_ws/src/lidar_py/package.xml /tmp/rosdep_src/lidar_py/package.xml
COPY lidar/chapt1_ws/src/local_depth_cloud_cpp/package.xml /tmp/rosdep_src/local_depth_cloud_cpp/package.xml
COPY lidar/chapt1_ws/src/short_goal_bt/package.xml /tmp/rosdep_src/short_goal_bt/package.xml
COPY lidar/chapt1_ws/src/reloc_rviz_panel/package.xml /tmp/rosdep_src/reloc_rviz_panel/package.xml
COPY lidar/chapt1_ws/src/frontier_exploration_ros2/package.xml /tmp/rosdep_src/frontier_exploration_ros2/package.xml
COPY lidar/chapt1_ws/src/frontier_exploration_ros2/plugin/frontier_exploration_ros2_rviz/package.xml /tmp/rosdep_src/frontier_exploration_ros2_rviz/package.xml
COPY lidar/chapt1_ws/src/OrbbecSDK_ROS2/orbbec_camera/package.xml /tmp/rosdep_src/orbbec_camera/package.xml
COPY lidar/chapt1_ws/src/OrbbecSDK_ROS2/orbbec_camera_msgs/package.xml /tmp/rosdep_src/orbbec_camera_msgs/package.xml
COPY lidar/chapt1_ws/src/OrbbecSDK_ROS2/orbbec_description/package.xml /tmp/rosdep_src/orbbec_description/package.xml

RUN apt-get update \
    && rosdep install --from-paths /tmp/rosdep_src \
         --ignore-src --rosdistro humble -r -y \
         --skip-keys "orbbec_camera cartographer_ros libgoogle-glog-dev" \
    && if [ "$FULL_VENDOR" = 1 ]; then \
         apt-get install -y --no-install-recommends ros-humble-cartographer-ros; \
       fi \
    && rm -rf /var/lib/apt/lists/*

COPY . /workspace

# Persist colcon's build/install/log trees outside the image layer cache. A
# changed source file then recompiles only its affected package and dependants.
RUN --mount=type=cache,id=huichuan-arm64-humble-colcon,target=/tmp/huichuan_agv_humble_ws \
    if [ "$FULL_VENDOR" = 1 ]; then \
      file lidar/chapt1_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/libOrbbecSDK.so.2.9.3 \
        | grep -Eq 'ARM aarch64|ARM64'; \
      readelf -h lidar/chapt1_ws/src/OrbbecSDK_ROS2/orbbec_camera/SDK/lib/arm64/libOrbbecSDK.so.2.9.3 \
        | grep -Eq 'Machine:[[:space:]]+AArch64'; \
      CAR_VALIDATION_SKIP_EXTERNAL=0 CAR_VALIDATION_BUILD_VENDOR=1 \
        ./validate_auto_mapping_humble.sh --build; \
    else \
      CAR_VALIDATION_SKIP_EXTERNAL=1 ./validate_auto_mapping_humble.sh --build; \
    fi

CMD ["bash"]
