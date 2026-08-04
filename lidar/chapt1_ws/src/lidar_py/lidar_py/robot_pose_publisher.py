#!/usr/bin/env python3
"""Publish map-frame robot pose for the web console.

The web page should not integrate odom by itself. It subscribes to /robot_pose,
which is generated here from the current TF chain. Position is normally taken
from map -> base_link. For map display, heading should also come from
map -> base_link so the arrow stays aligned with the map and RViz LaserScan
after SLAM/localization changes map -> odom.

When orientation_source=map, a yaw smoothing filter reduces the 2-3°
jitter from Cartographer's continuous map->odom corrections.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener
import math


def quat_multiply(q1, q2):
    """Multiply two quaternions: q1 * q2"""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


def quat_to_yaw(q):
    """Extract yaw (radians) from quaternion (x,y,z,w)."""
    siny_cosp = 2.0 * (q[3] * q[2] + q[0] * q[1])
    cosy_cosp = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quat(yaw):
    """Convert yaw angle to quaternion (x,y,z,w)."""
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class RobotPosePublisher(Node):
    def __init__(self):
        super().__init__("robot_pose_publisher")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("orientation_source", "map")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("topic", "/robot_pose")
        self.declare_parameter("corrected_odom_topic", "/cartographer_pose_odom")
        self.declare_parameter("yaw_offset_deg", 0.0)
        self.declare_parameter("yaw_smooth_alpha", 0.3)

        self.map_frame = self.get_parameter("map_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.orientation_source = str(self.get_parameter("orientation_source").value).lower()
        topic = self.get_parameter("topic").value
        corrected_odom_topic = self.get_parameter("corrected_odom_topic").value
        rate = float(self.get_parameter("publish_rate").value)
        yaw_offset_deg = float(self.get_parameter("yaw_offset_deg").value)
        half_yaw = math.radians(yaw_offset_deg) * 0.5
        self.yaw_offset_q = (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))
        self.yaw_smooth_alpha = float(self.get_parameter("yaw_smooth_alpha").value)
        self.yaw_smooth_alpha = max(0.01, min(1.0, self.yaw_smooth_alpha))

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(PoseStamped, topic, 10)
        self.corrected_odom_pub = self.create_publisher(
            Odometry, corrected_odom_topic, 10)
        self.warn_count = 0
        self.odom_warn_count = 0
        self._last_orientation = None
        self._smoothed_yaw = None  # EMA-smoothed yaw for map orientation
        self._last_map_pose = None
        self._last_map_stamp_ns = None
        self.create_timer(max(0.02, 1.0 / rate), self.publish_pose)
        self.get_logger().info(
            f"Publishing {topic}: position={self.map_frame}->{self.base_frame}, "
            f"orientation={self.orientation_source}->{self.base_frame}, rate={rate:.1f}Hz, "
            f"yaw_smooth_alpha={self.yaw_smooth_alpha:.2f}; "
            f"Cartographer-corrected odom={corrected_odom_topic}"
        )

    def publish_pose(self):
        if not rclpy.ok():
            return
        try:
            map_transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self.warn_count += 1
            if self.warn_count <= 5 or self.warn_count % 100 == 0:
                self.get_logger().warn(f"Waiting for TF {self.map_frame}->{self.base_frame}: {exc}")
            return

        stamp_ns = (
            map_transform.header.stamp.sec * 1000000000
            + map_transform.header.stamp.nanosec
        )
        translation = map_transform.transform.translation
        rotation = map_transform.transform.rotation
        raw_map_yaw = quat_to_yaw((rotation.x, rotation.y, rotation.z, rotation.w))
        if self._last_map_pose is not None and self._last_map_stamp_ns is not None:
            dt = (stamp_ns - self._last_map_stamp_ns) / 1e9
            if dt > 0.0:
                dx = translation.x - self._last_map_pose[0]
                dy = translation.y - self._last_map_pose[1]
                distance = math.hypot(dx, dy)
                yaw_delta = math.atan2(
                    math.sin(raw_map_yaw - self._last_map_pose[2]),
                    math.cos(raw_map_yaw - self._last_map_pose[2]),
                )
                if distance > 0.20 or abs(math.degrees(yaw_delta)) > 5.0:
                    self.get_logger().warn(
                        "CARTOGRAPHER_POSE_JUMP "
                        f"dt={dt:.3f}s translation={distance:.3f}m "
                        f"yaw={math.degrees(yaw_delta):+.2f}deg "
                        f"pose=({translation.x:.3f},{translation.y:.3f},"
                        f"{math.degrees(raw_map_yaw):+.2f}deg)"
                    )
        self._last_map_pose = (translation.x, translation.y, raw_map_yaw)
        self._last_map_stamp_ns = stamp_ns

        orientation = None
        if self.orientation_source == "odom":
            try:
                odom_transform = self.tf_buffer.lookup_transform(
                    self.odom_frame,
                    self.base_frame,
                    rclpy.time.Time(),
                )
                self._last_orientation = odom_transform.transform.rotation
            except TransformException as exc:
                self.odom_warn_count += 1
                if self.odom_warn_count <= 5 or self.odom_warn_count % 100 == 0:
                    self.get_logger().warn(
                        f"TF {self.odom_frame}->{self.base_frame} failed: {exc}"
                    )
        elif self.orientation_source != "map":
            self.get_logger().warn(
                f"Unknown orientation_source={self.orientation_source}, using map",
                throttle_duration_sec=5.0,
            )

        if self.orientation_source == "odom" and self._last_orientation is not None:
            orientation = self._last_orientation
        else:
            o = map_transform.transform.rotation
            raw_yaw = quat_to_yaw((o.x, o.y, o.z, o.w))
            if self._smoothed_yaw is None:
                self._smoothed_yaw = raw_yaw
            else:
                # Handle yaw wrap-around for EMA
                diff = raw_yaw - self._smoothed_yaw
                if diff > math.pi:
                    raw_yaw -= 2.0 * math.pi
                elif diff < -math.pi:
                    raw_yaw += 2.0 * math.pi
                self._smoothed_yaw += self.yaw_smooth_alpha * (raw_yaw - self._smoothed_yaw)
                # Normalize back to [-pi, pi]
                self._smoothed_yaw = math.atan2(
                    math.sin(self._smoothed_yaw), math.cos(self._smoothed_yaw))
            orientation = type(o)()
            q = yaw_to_quat(self._smoothed_yaw)
            orientation.x = q[0]
            orientation.y = q[1]
            orientation.z = q[2]
            orientation.w = q[3]

        o = orientation
        corrected = quat_multiply(
            (o.x, o.y, o.z, o.w),
            self.yaw_offset_q,
        )

        msg = PoseStamped()
        msg.header = map_transform.header
        msg.pose.position.x = map_transform.transform.translation.x
        msg.pose.position.y = map_transform.transform.translation.y
        msg.pose.position.z = map_transform.transform.translation.z
        msg.pose.orientation.x = corrected[0]
        msg.pose.orientation.y = corrected[1]
        msg.pose.orientation.z = corrected[2]
        msg.pose.orientation.w = corrected[3]
        self.pub.publish(msg)

        # RTAB-Map must use the same corrected pose authority as the 2D map.
        # Publish the raw map->base transform here; the web-only yaw smoother
        # above must never feed mapping or localization.
        corrected_odom = Odometry()
        corrected_odom.header = map_transform.header
        corrected_odom.header.frame_id = self.map_frame
        corrected_odom.child_frame_id = self.base_frame
        corrected_odom.pose.pose.position.x = map_transform.transform.translation.x
        corrected_odom.pose.pose.position.y = map_transform.transform.translation.y
        corrected_odom.pose.pose.position.z = map_transform.transform.translation.z
        corrected_odom.pose.pose.orientation = map_transform.transform.rotation
        corrected_odom.pose.covariance[0] = 0.0025
        corrected_odom.pose.covariance[7] = 0.0025
        corrected_odom.pose.covariance[14] = 0.01
        corrected_odom.pose.covariance[21] = 0.01
        corrected_odom.pose.covariance[28] = 0.01
        corrected_odom.pose.covariance[35] = 0.001
        self.corrected_odom_pub.publish(corrected_odom)


def main(args=None):
    rclpy.init(args=args)
    node = RobotPosePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # ros2 launch can shut down the context before the executor returns.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
