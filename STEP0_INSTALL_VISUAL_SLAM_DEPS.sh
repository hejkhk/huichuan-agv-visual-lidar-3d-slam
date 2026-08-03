#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_SETUP=/opt/ros/humble/setup.bash

[ -f "$ROS_SETUP" ] || {
  echo "[ERROR] ROS 2 Humble is missing: $ROS_SETUP" >&2
  echo "Install ROS 2 Humble Desktop on Ubuntu 22.04 first." >&2
  exit 1
}
# shellcheck disable=SC1091
source "$ROS_SETUP"
[ "${ROS_DISTRO:-}" = humble ] || {
  echo "[ERROR] Expected ROS_DISTRO=humble, got ${ROS_DISTRO:-unset}" >&2
  exit 1
}

echo "[1/3] Installing Ubuntu 22.04 / ROS 2 Humble dependencies..."
sudo apt update
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-octomap-server \
  ros-humble-depth-image-proc \
  ros-humble-sensor-msgs-py \
  ros-humble-robot-localization \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-nav2-regulated-pure-pursuit-controller \
  ros-humble-spatio-temporal-voxel-layer \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-tf2-tools \
  ros-humble-rviz2 \
  ros-humble-robot-state-publisher \
  ros-humble-xacro \
  ros-humble-laser-filters \
  ros-humble-cartographer-ros \
  ros-humble-rosbridge-server \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-empy \
  python3-lark \
  python3-yaml \
  python3-serial \
  python3-numpy \
  psmisc procps

echo "[2/3] Checking the Gemini2 driver..."
if ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
  echo "  Found: $(ros2 pkg prefix orbbec_camera)"
else
  cat <<MSG
[WARNING] orbbec_camera is not visible in the Humble environment.
Build OrbbecSDK_ROS2 against ROS 2 Humble, then set its setup file in:
  $ROOT_DIR/visual_laser_slam/dual_resolution_3d.env
Example:
  ORBBEC_SETUP=/home/<user>/OrbbecSDK_ROS2/install/setup.bash
Do not reuse a driver binary built against Jazzy.
MSG
fi

echo "[3/3] Dependency installation complete."
echo "Run: ./validate_auto_mapping_humble.sh"
echo "Then: ./START_DUAL_2D_3D_MAPPING.sh or ./START_DUAL_2D_3D_NAVIGATION.sh"
