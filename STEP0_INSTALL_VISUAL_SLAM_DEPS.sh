#!/usr/bin/env bash
set -Eeo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ -f /opt/ros/jazzy/setup.bash ] || { echo "未找到 ROS 2 Jazzy。" >&2; exit 1; }
source /opt/ros/jazzy/setup.bash

echo "[1/3] 安装 RTAB-Map、robot_localization 与调试工具……"
sudo apt update
sudo apt install -y \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-octomap-server \
  ros-jazzy-depth-image-proc \
  ros-jazzy-sensor-msgs-py \
  ros-jazzy-robot-localization \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-regulated-pure-pursuit-controller \
  ros-jazzy-spatio-temporal-voxel-layer \
  ros-jazzy-rmw-cyclonedds-cpp \
  ros-jazzy-tf2-tools \
  ros-jazzy-rviz2 \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-xacro \
  ros-jazzy-laser-filters \
  ros-jazzy-cartographer-ros \
  python3-colcon-common-extensions

echo "[2/3] 检查 Gemini2 驱动……"
if ros2 pkg prefix orbbec_camera >/dev/null 2>&1; then
  echo "  已找到 orbbec_camera: $(ros2 pkg prefix orbbec_camera)"
else
  cat <<'MSG'
  未在当前环境发现 orbbec_camera。
  你原项目的 RGB-D 功能若已能运行，请把 Orbbec 工作空间的 setup.bash 写入：
    visual_laser_slam/visual_laser_slam.env
  例如：
    ORBBEC_SETUP=/home/peter/OrbbecSDK_ROS2/install/setup.bash
MSG
fi

echo "[3/3] 完成。接下来运行："
echo "  ./STEP1_RGBD_CAMERA_TEST.sh"
echo "配置文件：$ROOT_DIR/visual_laser_slam/visual_laser_slam.env"
