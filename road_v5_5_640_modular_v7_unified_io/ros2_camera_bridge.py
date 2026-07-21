"""
兼容旧入口：真正的 ROS2 相机桥接已经并入 camera_input.py。
你仍然可以直接运行本文件测试 ROS2 RGB-D 输入。
"""

import cv2 as cv
from camera_input import Ros2TopicCameraManager


def main():
    cam = Ros2TopicCameraManager()
    if not cam.start():
        return
    try:
        while True:
            color, depth = cam.get_frames()
            if color is None or depth is None:
                continue
            cv.imshow("ROS2 RGB", color)
            d8 = cv.convertScaleAbs(depth, alpha=255.0/3000.0)
            cv.imshow("ROS2 Depth(mm)", d8)
            if (cv.waitKey(1) & 0xFF) == 27:
                break
    finally:
        cam.stop()


if __name__ == "__main__":
    main()
