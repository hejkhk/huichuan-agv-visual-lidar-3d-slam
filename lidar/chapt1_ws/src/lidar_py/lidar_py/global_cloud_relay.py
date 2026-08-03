#!/usr/bin/env python3
"""Rate-limit the live depth cloud and bridge sensor QoS to OctoMap QoS."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class GlobalCloudRelay(Node):
    def __init__(self):
        super().__init__("global_cloud_relay")
        self.declare_parameter("input_topic", "/local_highres_cloud_v21")
        self.declare_parameter("output_topic", "/global_3d/cloud_in")
        self.declare_parameter("publish_rate_hz", 5.0)

        rate = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(
            PointCloud2, self.get_parameter("output_topic").value, output_qos)
        self._subscription = self.create_subscription(
            PointCloud2,
            self.get_parameter("input_topic").value,
            self._cloud_callback,
            qos_profile_sensor_data,
        )
        self._latest = None
        self._last_stamp = None
        self._timer = self.create_timer(1.0 / rate, self._publish_latest)
        self.get_logger().info(
            f"Global geometry cloud relay: {rate:.1f} Hz, sensor QoS -> reliable QoS")

    def _cloud_callback(self, msg):
        self._latest = msg

    def _publish_latest(self):
        if self._latest is None:
            return
        stamp = (self._latest.header.stamp.sec, self._latest.header.stamp.nanosec)
        if stamp == self._last_stamp:
            return
        self._last_stamp = stamp
        self._publisher.publish(self._latest)


def main(args=None):
    rclpy.init(args=args)
    node = GlobalCloudRelay()
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
