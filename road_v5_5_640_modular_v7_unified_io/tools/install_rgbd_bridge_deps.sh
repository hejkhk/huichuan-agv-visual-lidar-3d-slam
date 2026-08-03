#!/usr/bin/env bash
set -e

echo "=========================================="
echo "  Raspberry Pi ROS2 RGB-D Bridge 依赖安装"
echo "=========================================="

if [ ! -d "/opt/ros/humble" ]; then
    echo "❌ 没找到 /opt/ros/humble"
    echo "请先在 Ubuntu 22.04 安装 ROS 2 Humble。"
    exit 1
fi

source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
    ros-humble-rclpy \
    ros-humble-sensor-msgs \
    ros-humble-std-msgs \
    ros-humble-image-transport \
    ros-humble-compressed-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-rqt-image-view \
    ros-humble-image-tools \
    python3-pip \
    python3-numpy \
    python3-opencv \
    python3-serial \
    python3-yaml \
    python3-colcon-common-extensions \
    unzip git htop \
    libgl1 libglib2.0-0 libgtk-3-0

if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

echo "✅ 完成。测试："
echo "source /opt/ros/humble/setup.bash"
echo "python3 -c \"import rclpy, cv2, numpy; from sensor_msgs.msg import Image; print('OK')\""
