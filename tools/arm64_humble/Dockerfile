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
COPY . /workspace

RUN test "$(uname -m)" = aarch64 \
    && test "$(. /etc/os-release && printf '%s' "$VERSION_ID")" = 22.04

RUN (rosdep init 2>/dev/null || true) \
    && rosdep update

RUN apt-get update \
    && rosdep install --from-paths lidar/chapt1_ws/src \
         --ignore-src --rosdistro humble -r -y \
         --skip-keys "orbbec_camera cartographer_ros libgoogle-glog-dev" \
    && if [ "$FULL_VENDOR" = 1 ]; then \
         apt-get install -y --no-install-recommends ros-humble-cartographer-ros; \
       fi \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "$FULL_VENDOR" = 1 ]; then \
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
