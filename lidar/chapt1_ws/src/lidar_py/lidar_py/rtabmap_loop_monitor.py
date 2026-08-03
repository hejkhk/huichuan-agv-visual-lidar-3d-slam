#!/usr/bin/env python3
"""Report accepted RTAB-Map visual loop closures without publishing TF."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from std_msgs.msg import Bool, String


class RtabmapLoopMonitor(Node):
    def __init__(self):
        super().__init__("rtabmap_loop_monitor")
        self.declare_parameter("info_topic", "/rtabmap_3d/info")
        self.declare_parameter(
            "event_topic", "/rtabmap_3d/visual_loop_event")
        self.declare_parameter(
            "detected_topic", "/rtabmap_3d/visual_loop_detected")
        self.declare_parameter("status_period_sec", 10.0)

        self.accepted_count = 0
        self.proximity_count = 0
        self.last_pair = None
        self.last_info_time = 0.0
        self.event_pub = self.create_publisher(
            String, self.get_parameter("event_topic").value, 10)
        self.detected_pub = self.create_publisher(
            Bool, self.get_parameter("detected_topic").value, 10)
        self.create_subscription(
            Info,
            self.get_parameter("info_topic").value,
            self._on_info,
            10,
        )
        period = max(
            2.0, float(self.get_parameter("status_period_sec").value))
        self.create_timer(period, self._report)
        self.get_logger().info(
            "RTAB-Map visual loop monitor active; this node never publishes TF")

    def _on_info(self, msg: Info) -> None:
        self.last_info_time = time.monotonic()
        ref_id = int(getattr(msg, "ref_id", 0))
        loop_id = int(getattr(msg, "loop_closure_id", 0))
        proximity_id = int(getattr(msg, "proximity_detection_id", 0))
        event = None
        if loop_id > 0:
            pair = (ref_id, loop_id, "global")
            if pair != self.last_pair:
                self.accepted_count += 1
                event = (
                    f"VISUAL_LOOP_ACCEPTED ref={ref_id} match={loop_id} "
                    f"count={self.accepted_count}")
                self.last_pair = pair
        elif proximity_id > 0:
            pair = (ref_id, proximity_id, "proximity")
            if pair != self.last_pair:
                self.proximity_count += 1
                event = (
                    f"VISUAL_PROXIMITY_ACCEPTED ref={ref_id} "
                    f"match={proximity_id} count={self.proximity_count}")
                self.last_pair = pair

        detected = Bool()
        detected.data = event is not None
        self.detected_pub.publish(detected)
        if event is not None:
            self.get_logger().warn(event)
            self.event_pub.publish(String(data=event))

    def _report(self) -> None:
        age = (
            time.monotonic() - self.last_info_time
            if self.last_info_time > 0.0 else -1.0)
        self.get_logger().info(
            "VISUAL_LOOP_STATUS "
            f"global={self.accepted_count} proximity={self.proximity_count} "
            f"info_age={age:.2f}s")


def main(args=None):
    rclpy.init(args=args)
    node = RtabmapLoopMonitor()
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
