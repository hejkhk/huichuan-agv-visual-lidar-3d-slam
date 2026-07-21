#!/usr/bin/env bash
set -e

echo "=========================================="
echo "  Raspberry Pi ROS2 RGB-D Bridge 依赖安装"
echo "=========================================="

if [ ! -d "/opt/ros/jazzy" ]; then
    echo "❌ 没找到 /opt/ros/jazzy"
    echo "如果你不是 Jazzy，请把脚本里的 jazzy 改成你的 ROS2 版本。"
    exit 1
fi

source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install -y \
    ros-jazzy-rclpy \
    ros-jazzy-sensor-msgs \
    ros-jazzy-std-msgs \
    ros-jazzy-image-transport \
    ros-jazzy-compressed-image-transport \
    ros-jazzy-camera-info-manager \
    ros-jazzy-rqt-image-view \
    ros-jazzy-image-tools \
    python3-pip \
    python3-numpy \
    python3-opencv \
    python3-serial \
    python3-yaml \
    python3-colcon-common-extensions \
    unzip git htop \
    libgl1 libglib2.0-0 libgtk-3-0

if ! grep -q "source /opt/ros/jazzy/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
fi

echo "✅ 完成。测试："
echo "source /opt/ros/jazzy/setup.bash"
echo "python3 -c \"import rclpy, cv2, numpy; from sensor_msgs.msg import Image; print('OK')\""
