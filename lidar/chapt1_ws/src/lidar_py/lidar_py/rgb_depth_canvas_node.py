#!/usr/bin/env python3
"""Match the registered RGB canvas to the native depth image geometry.

Gemini2 publishes registered color at 640x480 while native depth is 640x400
when alignment targets DEPTH. RTAB-Map expects both images to share one image
geometry. This node center-crops the registered color canvas and republishes
the depth CameraInfo with the color timestamp.
"""

from copy import deepcopy

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class RgbDepthCanvasNode(Node):
    def __init__(self):
        super().__init__("rgb_depth_canvas")
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("output_rgb_topic", "/camera/rtabmap/rgb/image_raw")
        self.declare_parameter("output_info_topic", "/camera/rtabmap/rgb/camera_info")
        self.declare_parameter("target_width", 640)
        self.declare_parameter("target_height", 400)

        self.target_width = int(self.get_parameter("target_width").value)
        self.target_height = int(self.get_parameter("target_height").value)
        self.depth_info = None
        self.warned_encoding = False

        self.rgb_pub = self.create_publisher(
            Image, self.get_parameter("output_rgb_topic").value, qos_profile_sensor_data)
        self.info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("output_info_topic").value, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo,
            self.get_parameter("depth_info_topic").value,
            self._info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("rgb_topic").value,
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"RTAB RGB canvas target={self.target_width}x{self.target_height}; "
            "native depth CameraInfo is authoritative"
        )

    def _info_callback(self, msg):
        self.depth_info = msg

    def _rgb_callback(self, msg):
        if self.depth_info is None:
            return
        if msg.width < self.target_width or msg.height < self.target_height:
            self.get_logger().warn(
                f"RGB {msg.width}x{msg.height} is smaller than target "
                f"{self.target_width}x{self.target_height}",
                throttle_duration_sec=5.0,
            )
            return
        if msg.width != self.target_width:
            self.get_logger().warn(
                f"Unsupported RGB width crop {msg.width}->{self.target_width}; "
                "only equal widths are accepted",
                throttle_duration_sec=5.0,
            )
            return

        bytes_per_pixel = msg.step // msg.width if msg.width else 0
        if bytes_per_pixel <= 0 or msg.step != msg.width * bytes_per_pixel:
            if not self.warned_encoding:
                self.get_logger().warn(
                    f"Unsupported packed image layout encoding={msg.encoding} step={msg.step}")
                self.warned_encoding = True
            return

        top = (msg.height - self.target_height) // 2
        start = top * msg.step
        end = start + self.target_height * msg.step

        output = Image()
        output.header = msg.header
        output.height = self.target_height
        output.width = self.target_width
        output.encoding = msg.encoding
        output.is_bigendian = msg.is_bigendian
        output.step = msg.step
        output.data = msg.data[start:end]

        info = deepcopy(self.depth_info)
        info.header = msg.header
        info.width = self.target_width
        info.height = self.target_height
        self.rgb_pub.publish(output)
        self.info_pub.publish(info)


def main(args=None):
    rclpy.init(args=args)
    node = RgbDepthCanvasNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
