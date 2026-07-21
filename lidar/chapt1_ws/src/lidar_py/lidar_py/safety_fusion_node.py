#!/usr/bin/env python3
"""Coordinate Nav2, web teleoperation and near-ground depth avoidance."""

from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int32MultiArray

from lidar_py.fusion_control import (
    DepthSample,
    FusionConfig,
    FusionController,
    build_virtual_scan_ranges,
    clamp,
)


class SafetyFusionNode(Node):
    """Single velocity arbiter and depth-to-costmap adapter."""

    def __init__(self):
        super().__init__("safety_fusion_node")

        self.declare_parameter("nav_cmd_topic", "/cmd_vel_nav")
        self.declare_parameter("web_cmd_topic", "/cmd_vel_web")
        self.declare_parameter("depth_topic", "/depth_obstacle")
        self.declare_parameter("safe_cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("wheel_topic", "/wheel_speed_cmd")
        self.declare_parameter("virtual_scan_topic", "/depth_obstacle_scan")
        self.declare_parameter("virtual_scan_frame", "base_link")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("depth_timeout_sec", 0.5)
        self.declare_parameter("nav_timeout_sec", 0.7)
        self.declare_parameter("web_timeout_sec", 0.35)
        self.declare_parameter("require_depth_alive", False)
        self.declare_parameter("require_depth_baseline", False)
        self.declare_parameter(
            "software_estop_topic", "/robot/emergency_stop_state")
        self.declare_parameter("status_log_period_sec", 1.0)

        self.declare_parameter("max_v", 0.23)
        self.declare_parameter("max_w", 0.80)
        self.declare_parameter("nav_arc_outer_wheel_mps", 0.16)
        self.declare_parameter("level_release_hold_sec", 0.40)
        self.declare_parameter("direction_switch_margin", 80)
        self.declare_parameter("direction_switch_frames", 6)
        self.declare_parameter("forward_threshold", 0.015)
        self.declare_parameter("warning_w_min", 0.05)
        self.declare_parameter("warning_w_max", 0.20)
        self.declare_parameter("danger_w_min", 0.12)
        self.declare_parameter("danger_w_max", 0.32)
        self.declare_parameter("return_w_slew_rate", 1.4)
        self.declare_parameter("avoid_w_slew_rate", 2.2)
        self.declare_parameter("distance_config_dir", "")
        self.declare_parameter("danger_distance_mm", 450)
        self.declare_parameter("warning_distance_mm", 950)
        self.declare_parameter("critical_distance_mm", 300)
        self.declare_parameter("virtual_scan_min_range_m", 0.18)
        self.declare_parameter("virtual_scan_max_range_m", 1.30)
        self.declare_parameter("virtual_scan_origin_x_m", 0.30)

        self.declare_parameter("pulse_per_rev", 8388608.0)
        self.declare_parameter("gear_ratio", 25.0)
        self.declare_parameter("wheel_radius", 0.0755)
        # This project historically names the center-to-wheel distance
        # wheel_track_w. It is the differential kinematic turn radius.
        self.declare_parameter("wheel_track_w", 0.2825)
        self.declare_parameter("max_wheel_cnt", 100000000)

        self.nav_cmd = Twist()
        self.nav_time = 0.0
        self.web_cmd = Twist()
        self.web_time = 0.0
        self.active_command_source = "none"
        self.depth_time = 0.0
        self.depth_seq = -1
        self.depth_sample = DepthSample()
        self.baseline_seen = True
        self.baseline_ready = False
        self.software_estop = False
        self.last_fuse_time = time.perf_counter()
        self.last_status_log = 0.0
        self.last_safe_cmd = Twist()

        self._load_distance_thresholds()
        self.controller = FusionController(self._fusion_config())

        self.safe_pub = self.create_publisher(
            Twist, self.get_parameter("safe_cmd_topic").value, 10)
        self.wheel_pub = self.create_publisher(
            Int32MultiArray, self.get_parameter("wheel_topic").value, 10)
        self.virtual_scan_pub = self.create_publisher(
            LaserScan,
            self.get_parameter("virtual_scan_topic").value,
            qos_profile_sensor_data,
        )

        self.create_subscription(
            Twist, self.get_parameter("nav_cmd_topic").value,
            self._on_nav_cmd, 10)
        self.create_subscription(
            Twist, self.get_parameter("web_cmd_topic").value,
            self._on_web_cmd, 10)
        self.create_subscription(
            Int32MultiArray, self.get_parameter("depth_topic").value,
            self._on_depth, 10)
        self.create_subscription(
            Bool, "/depth/baseline_ready", self._on_baseline_ready, 10)
        self.create_subscription(
            Bool, self.get_parameter("software_estop_topic").value,
            self._on_software_estop, 10)

        self.publish_rate = max(
            1.0, float(self.get_parameter("publish_rate").value))
        self.create_timer(1.0 / self.publish_rate, self._publish_safe_cmd)
        self.get_logger().info(
            "Safety fusion started: one velocity arbiter + depth virtual scan")

    def _fusion_config(self) -> FusionConfig:
        return FusionConfig(
            max_v=float(self.get_parameter("max_v").value),
            max_w=float(self.get_parameter("max_w").value),
            warning_distance_mm=self.warning_distance_mm,
            danger_distance_mm=self.danger_distance_mm,
            critical_distance_mm=self.critical_distance_mm,
            level_release_hold_sec=float(
                self.get_parameter("level_release_hold_sec").value),
            direction_switch_margin=int(
                self.get_parameter("direction_switch_margin").value),
            direction_switch_frames=int(
                self.get_parameter("direction_switch_frames").value),
            forward_threshold=float(
                self.get_parameter("forward_threshold").value),
            warning_w_min=float(self.get_parameter("warning_w_min").value),
            warning_w_max=float(self.get_parameter("warning_w_max").value),
            danger_w_min=float(self.get_parameter("danger_w_min").value),
            danger_w_max=float(self.get_parameter("danger_w_max").value),
            avoid_w_slew_rate=float(
                self.get_parameter("avoid_w_slew_rate").value),
            return_w_slew_rate=float(
                self.get_parameter("return_w_slew_rate").value),
        )

    def _load_distance_thresholds(self) -> None:
        config_dir = str(
            self.get_parameter("distance_config_dir").value or "").strip()
        config_dir = config_dir or os.environ.get("VISION_CODE_DIR", "").strip()
        if config_dir and os.path.isdir(config_dir) and config_dir not in sys.path:
            sys.path.insert(0, config_dir)

        self.danger_distance_mm = int(
            self.get_parameter("danger_distance_mm").value)
        self.warning_distance_mm = int(
            self.get_parameter("warning_distance_mm").value)
        self.critical_distance_mm = int(
            self.get_parameter("critical_distance_mm").value)
        try:
            from calibration_640 import FAR_VALUE, STOP_VALUE, WARN_VALUE
            self.danger_distance_mm = int(STOP_VALUE)
            self.warning_distance_mm = int(max(WARN_VALUE, FAR_VALUE))
            self.critical_distance_mm = int(
                max(220, min(STOP_VALUE - 120, 330)))
            self.get_logger().info(
                "Loaded depth thresholds from calibration_640: "
                f"stop={STOP_VALUE}mm warn={WARN_VALUE}mm far={FAR_VALUE}mm "
                f"critical={self.critical_distance_mm}mm")
        except Exception as exc:
            self.get_logger().warn(
                "Using launch/default depth thresholds: "
                f"danger={self.danger_distance_mm}mm "
                f"warning={self.warning_distance_mm}mm "
                f"critical={self.critical_distance_mm}mm ({exc})")

    def _on_nav_cmd(self, msg: Twist) -> None:
        self.nav_cmd = msg
        self.nav_time = time.perf_counter()

    def _on_web_cmd(self, msg: Twist) -> None:
        self.web_cmd = msg
        self.web_time = time.perf_counter()

    def _on_baseline_ready(self, msg: Bool) -> None:
        self.baseline_seen = True
        self.baseline_ready = bool(msg.data)

    def _on_software_estop(self, msg: Bool) -> None:
        self.software_estop = bool(msg.data)

    def _on_depth(self, msg: Int32MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 8:
            self.get_logger().warn(
                "Ignoring malformed /depth_obstacle message",
                throttle_duration_sec=2.0)
            return
        self.depth_sample = DepthSample(
            level=int(data[0]),
            preferred_dir=int(data[1]),
            nearest_mm=int(data[2]),
            center_danger=bool(data[3]),
            center_far=bool(data[4]),
            left_blocked=bool(data[5]),
            right_blocked=bool(data[6]),
            center_area_x1000=int(data[8]) if len(data) >= 14 else 0,
            total_area_x1000=int(data[9]) if len(data) >= 14 else 0,
            left_score_x1000=int(data[10]) if len(data) >= 14 else 0,
            right_score_x1000=int(data[11]) if len(data) >= 14 else 0,
            center_min_mm=int(data[13]) if len(data) >= 14 else 9999,
        )
        self.depth_seq = int(data[7])
        self.depth_time = time.perf_counter()

    def _alive(self, timestamp: float, parameter: str) -> bool:
        return (
            time.perf_counter() - timestamp
            <= float(self.get_parameter(parameter).value)
        )

    def _depth_alive(self) -> bool:
        return self._alive(self.depth_time, "depth_timeout_sec")

    def _nav_alive(self) -> bool:
        return self._alive(self.nav_time, "nav_timeout_sec")

    def _web_alive(self) -> bool:
        return self._alive(self.web_time, "web_timeout_sec")

    @staticmethod
    def _limit_side_speed(v: float, w: float, radius: float, limit: float):
        if limit <= 0.0:
            return v, w
        peak = max(abs(v - w * radius), abs(v + w * radius))
        if peak <= limit or peak <= 1e-9:
            return v, w
        scale = limit / peak
        return v * scale, w * scale

    def _select_source(self):
        if self._web_alive():
            self.active_command_source = "web"
            return self.web_cmd
        if self._nav_alive():
            self.active_command_source = "nav2"
            return self.nav_cmd
        self.active_command_source = "none"
        return Twist()

    def _fuse(self):
        now = time.perf_counter()
        dt = clamp(now - self.last_fuse_time, 0.0, 0.2)
        self.last_fuse_time = now
        source = self._select_source()
        nav_v = float(source.linear.x)
        nav_w = float(source.angular.z)
        radius = float(self.get_parameter("wheel_track_w").value)

        if (
            self.active_command_source == "nav2"
            and abs(nav_v) > 0.02
            and abs(nav_w) > 0.02
        ):
            nav_v, nav_w = self._limit_side_speed(
                nav_v,
                nav_w,
                radius,
                float(self.get_parameter("nav_arc_outer_wheel_mps").value),
            )

        baseline_blocked = bool(
            self.get_parameter("require_depth_baseline").value
        ) and self.baseline_seen and not self.baseline_ready
        depth_alive = self._depth_alive()
        depth_required_missing = bool(
            self.get_parameter("require_depth_alive").value
        ) and not depth_alive

        if self.software_estop or baseline_blocked or depth_required_missing:
            self.controller.reset()
            result = self.controller.update(
                0.0, 0.0, DepthSample(), now, dt, depth_alive=False)
            if self.software_estop:
                self.active_command_source = "software_estop"
            elif baseline_blocked:
                self.active_command_source = "baseline_lock"
            else:
                self.active_command_source = "depth_timeout_lock"
            return result

        return self.controller.update(
            nav_v,
            nav_w,
            self.depth_sample,
            now,
            dt,
            depth_alive=depth_alive,
        )

    def _twist_to_lr_counts(self, msg: Twist):
        wheel_radius = float(self.get_parameter("wheel_radius").value)
        gear_ratio = float(self.get_parameter("gear_ratio").value)
        pulse_per_rev = float(self.get_parameter("pulse_per_rev").value)
        radius = float(self.get_parameter("wheel_track_w").value)
        max_wheel_cnt = int(self.get_parameter("max_wheel_cnt").value)
        left_mps = float(msg.linear.x) - float(msg.angular.z) * radius
        right_mps = float(msg.linear.x) + float(msg.angular.z) * radius
        factor = (pulse_per_rev * gear_ratio) / (
            2.0 * math.pi * wheel_radius)
        left = int(round(left_mps * factor))
        right = int(round(right_mps * factor))
        return (
            int(clamp(left, -max_wheel_cnt, max_wheel_cnt)),
            int(clamp(right, -max_wheel_cnt, max_wheel_cnt)),
        )

    def _publish_virtual_scan(self) -> None:
        depth_alive = self._depth_alive()
        sample = (
            self.controller.last_obstacle_sample
            if depth_alive
            else DepthSample()
        )
        stable_level = self.controller.stable_level if depth_alive else 0
        min_range = float(
            self.get_parameter("virtual_scan_min_range_m").value)
        max_range = float(
            self.get_parameter("virtual_scan_max_range_m").value)
        origin_offset = float(
            self.get_parameter("virtual_scan_origin_x_m").value)
        angle_min = -math.pi / 3.0
        angle_max = math.pi / 3.0
        ray_count = 61

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = str(
            self.get_parameter("virtual_scan_frame").value)
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = (angle_max - angle_min) / (ray_count - 1)
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / self.publish_rate
        scan.range_min = min_range
        scan.range_max = max_range
        scan.ranges = build_virtual_scan_ranges(
            sample,
            stable_level,
            ray_count=ray_count,
            angle_min=angle_min,
            angle_max=angle_max,
            min_range_m=min_range,
            max_range_m=max_range,
            origin_offset_m=origin_offset,
        )
        self.virtual_scan_pub.publish(scan)

    def _publish_safe_cmd(self) -> None:
        result = self._fuse()
        safe = Twist()
        safe.linear.x = result.linear_x
        safe.angular.z = result.angular_z
        self.last_safe_cmd = safe
        self.safe_pub.publish(safe)

        left, right = self._twist_to_lr_counts(safe)
        wheel_msg = Int32MultiArray()
        wheel_msg.data = [left, right]
        self.wheel_pub.publish(wheel_msg)
        self._publish_virtual_scan()

        now = time.perf_counter()
        period = float(self.get_parameter("status_log_period_sec").value)
        if period > 0.0 and now - self.last_status_log >= period:
            self.last_status_log = now
            self.get_logger().info(
                "FUSION_STATUS "
                f"source={self.active_command_source} mode={result.mode} "
                f"nav=({self.nav_cmd.linear.x:+.3f},"
                f"{self.nav_cmd.angular.z:+.3f}) "
                f"safe=({safe.linear.x:+.3f},{safe.angular.z:+.3f}) "
                f"depth_alive={self._depth_alive()} "
                f"level={self.depth_sample.level}->{result.level} "
                f"risk={result.risk:.2f} dir={result.direction:+d} "
                f"nearest={self.depth_sample.effective_nearest()} "
                f"score={self.depth_sample.left_score_x1000}/"
                f"{self.depth_sample.right_score_x1000} "
                f"baseline_ready={self.baseline_ready} "
                f"software_estop={self.software_estop}")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
