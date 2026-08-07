"""Pause motion briefly while Cartographer applies a global correction."""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

from .slam_correction_logic import Pose2D, SlamCorrectionDetector


def quaternion_yaw(rotation) -> float:
    """Return planar yaw from a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


class SlamCorrectionGuard(Node):
    """Publish a temporary velocity hold for map-to-odom discontinuities."""

    def __init__(self) -> None:
        super().__init__("slam_correction_guard")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("hold_topic", "/slam_correction_hold")
        self.declare_parameter("status_topic", "/slam_correction/status")
        self.declare_parameter("sample_rate_hz", 30.0)
        self.declare_parameter("translation_threshold_m", 0.10)
        self.declare_parameter("yaw_threshold_deg", 0.50)
        self.declare_parameter("window_sec", 0.50)
        self.declare_parameter("window_translation_threshold_m", 0.15)
        self.declare_parameter("window_yaw_threshold_deg", 1.00)
        self.declare_parameter("max_sample_gap_sec", 1.0)
        self.declare_parameter("hold_sec", 1.50)
        self.declare_parameter("startup_grace_sec", 2.0)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.odom_frame = str(self.get_parameter("odom_frame").value)
        self.hold_sec = max(
            0.2, float(self.get_parameter("hold_sec").value))
        self.startup_grace_sec = max(
            0.0, float(self.get_parameter("startup_grace_sec").value))
        self.started_at = time.monotonic()
        self.hold_until = 0.0
        self.hold_active = False
        self.event_count = 0
        self.last_transform_stamp_ns = None

        self.detector = SlamCorrectionDetector(
            translation_threshold=float(
                self.get_parameter("translation_threshold_m").value),
            yaw_threshold=math.radians(float(
                self.get_parameter("yaw_threshold_deg").value)),
            window_sec=float(self.get_parameter("window_sec").value),
            window_translation_threshold=float(self.get_parameter(
                "window_translation_threshold_m").value),
            window_yaw_threshold=math.radians(float(self.get_parameter(
                "window_yaw_threshold_deg").value)),
            max_sample_gap_sec=float(
                self.get_parameter("max_sample_gap_sec").value),
        )

        hold_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.hold_pub = self.create_publisher(
            Bool, str(self.get_parameter("hold_topic").value), hold_qos)
        self.status_pub = self.create_publisher(
            String, str(self.get_parameter("status_topic").value), 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        sample_rate = max(
            5.0, float(self.get_parameter("sample_rate_hz").value))
        self.timer = self.create_timer(1.0 / sample_rate, self._sample)
        self.get_logger().info(
            "SLAM correction guard active: "
            f"{self.map_frame}->{self.odom_frame}, "
            f"instant={self.detector.translation_threshold:.2f}m/"
            f"{math.degrees(self.detector.yaw_threshold):.2f}deg, "
            f"hold={self.hold_sec:.2f}s"
        )

    def _publish_state(self, now: float) -> None:
        active = now < self.hold_until
        self.hold_pub.publish(Bool(data=active))
        if self.hold_active and not active:
            message = f"SLAM_CORRECTION_RELEASE event={self.event_count}"
            self.get_logger().info(message)
            self.status_pub.publish(String(data=message))
        self.hold_active = active

    def _sample(self) -> None:
        now = time.monotonic()
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.odom_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"Waiting for TF {self.map_frame}->{self.odom_frame}: {exc}",
                throttle_duration_sec=5.0,
            )
            self._publish_state(now)
            return

        stamp = transform.header.stamp
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        if stamp_ns > 0 and stamp_ns == self.last_transform_stamp_ns:
            self._publish_state(now)
            return
        if stamp_ns > 0:
            self.last_transform_stamp_ns = stamp_ns

        translation = transform.transform.translation
        sample = Pose2D(
            time_sec=now,
            x=float(translation.x),
            y=float(translation.y),
            yaw=quaternion_yaw(transform.transform.rotation),
        )
        if now - self.started_at < self.startup_grace_sec:
            self.detector.reset(sample)
            self._publish_state(now)
            return

        correction = self.detector.update(sample)
        if correction is not None:
            self.event_count += 1
            self.hold_until = max(self.hold_until, now + self.hold_sec)
            message = (
                f"SLAM_CORRECTION_HOLD event={self.event_count} "
                f"instant={correction.instant_translation:.3f}m/"
                f"{math.degrees(correction.instant_yaw):.2f}deg "
                f"window={correction.window_translation:.3f}m/"
                f"{math.degrees(correction.window_yaw):.2f}deg "
                f"hold={self.hold_sec:.2f}s"
            )
            self.get_logger().warn(message)
            self.status_pub.publish(String(data=message))
        self._publish_state(now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SlamCorrectionGuard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.hold_pub.publish(Bool(data=False))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
