#!/usr/bin/env python3
"""Coordinate Nav2, web teleoperation and near-ground depth avoidance."""

from __future__ import annotations

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Bool, Int32MultiArray

from lidar_py.fusion_control import (
    AdaptiveArcGain,
    ArcGainConfig,
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
        self.declare_parameter("nav_raw_cmd_topic", "/cmd_vel_nav_raw")
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
            "local_cloud_topic", "/local_highres_cloud_v21/sensor")
        self.declare_parameter("local_cloud_timeout_sec", 0.35)
        self.declare_parameter("require_local_cloud_alive", False)
        self.declare_parameter(
            "collision_stop_topic", "/local_cloud_collision_stop")
        self.declare_parameter(
            "collision_status_topic", "/local_cloud_collision_status")
        self.declare_parameter(
            "navigation_sensor_health_topic",
            "/robot/navigation_sensor_healthy",
        )
        self.declare_parameter(
            "software_estop_topic", "/robot/emergency_stop_state")
        self.declare_parameter("status_log_period_sec", 1.0)

        self.declare_parameter("max_v", 0.23)
        self.declare_parameter("max_w", 0.80)
        self.declare_parameter("nav_arc_outer_wheel_mps", 0.18)
        self.declare_parameter("nav_arc_yaw_gain", 1.45)
        self.declare_parameter("nav_arc_max_w", 0.32)
        self.declare_parameter("nav_arc_gain_min_v", 0.10)
        self.declare_parameter("nav_arc_gain_min_radius", 0.70)
        self.declare_parameter("nav_pure_turn_min_w", 0.08)
        self.declare_parameter("nav_arc_adaptive_enabled", True)
        self.declare_parameter("nav_arc_gain_min", 1.10)
        self.declare_parameter("nav_arc_gain_max", 1.80)
        self.declare_parameter("nav_arc_gain_learning_rate", 0.08)
        self.declare_parameter("nav_arc_gain_settle_sec", 0.60)
        self.declare_parameter("nav_arc_gain_update_period_sec", 0.25)
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("approach_slow_distance_m", 1.20)
        self.declare_parameter("approach_stop_distance_m", 0.62)
        self.declare_parameter("approach_min_scale", 0.30)
        self.declare_parameter("approach_min_linear_speed_mps", 0.06)
        self.declare_parameter("collision_status_timeout_sec", 0.50)
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
        self.nav_raw_cmd = Twist()
        self.nav_raw_time = 0.0
        self.web_cmd = Twist()
        self.web_time = 0.0
        self.active_command_source = "none"
        self.depth_time = 0.0
        self.local_cloud_time = 0.0
        self.collision_stop = False
        self.collision_rotation_stop = False
        self.collision_rear_stop = False
        self.collision_scan_reported_alive = False
        self.collision_status_time = 0.0
        self.collision_nearest_x = math.inf
        self.collision_point_count = 0
        self.collision_rotation_point_count = 0
        self.collision_rear_point_count = 0
        self.collision_self_filtered_point_count = 0
        self.collision_approach_point_count = 0
        self.collision_approach_nearest_x = math.inf
        self.depth_seq = -1
        self.depth_sample = DepthSample()
        self.baseline_seen = True
        self.baseline_ready = False
        self.software_estop = False
        self.last_fuse_time = time.perf_counter()
        self.last_status_log = 0.0
        self.last_safe_cmd = Twist()
        self.last_arc_gain_applied = False
        self.last_pure_turn_lift_applied = False
        self.last_approach_scale = 1.0
        self.arc_learning_active = False
        self.arc_learning_since = 0.0
        self.arc_learning_desired_v = 0.0
        self.arc_learning_desired_w = 0.0
        self.last_arc_gain_update = 0.0
        self.arc_gain_estimator = AdaptiveArcGain(ArcGainConfig(
            initial_gain=float(self.get_parameter("nav_arc_yaw_gain").value),
            min_gain=float(self.get_parameter("nav_arc_gain_min").value),
            max_gain=float(self.get_parameter("nav_arc_gain_max").value),
            learning_rate=float(
                self.get_parameter("nav_arc_gain_learning_rate").value),
        ))

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
        self.navigation_sensor_health_pub = self.create_publisher(
            Bool,
            self.get_parameter("navigation_sensor_health_topic").value,
            10,
        )

        self.create_subscription(
            Twist, self.get_parameter("nav_cmd_topic").value,
            self._on_nav_cmd, 10)
        self.create_subscription(
            Twist, self.get_parameter("nav_raw_cmd_topic").value,
            self._on_nav_raw_cmd, 10)
        self.create_subscription(
            Twist, self.get_parameter("web_cmd_topic").value,
            self._on_web_cmd, 10)
        self.create_subscription(
            Int32MultiArray, self.get_parameter("depth_topic").value,
            self._on_depth, 10)
        self.create_subscription(
            PointCloud2,
            self.get_parameter("local_cloud_topic").value,
            self._on_local_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            self.get_parameter("collision_stop_topic").value,
            self._on_collision_stop,
            10,
        )
        self.create_subscription(
            Int32MultiArray,
            self.get_parameter("collision_status_topic").value,
            self._on_collision_status,
            10,
        )
        self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            qos_profile_sensor_data,
        )
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

    def _on_nav_raw_cmd(self, msg: Twist) -> None:
        self.nav_raw_cmd = msg
        self.nav_raw_time = time.perf_counter()

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

    def _on_local_cloud(self, _msg: PointCloud2) -> None:
        self.local_cloud_time = time.perf_counter()

    def _on_collision_stop(self, msg: Bool) -> None:
        self.collision_stop = bool(msg.data)

    def _on_collision_status(self, msg: Int32MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 3:
            return
        self.collision_status_time = time.perf_counter()
        self.collision_stop = bool(data[0])
        self.collision_point_count = int(data[1])
        self.collision_nearest_x = (
            float(data[2]) / 1000.0 if data[2] < 9999 else math.inf)
        if len(data) >= 8:
            self.collision_rotation_stop = bool(data[3])
            self.collision_rear_stop = bool(data[4])
            self.collision_rotation_point_count = int(data[5])
            self.collision_rear_point_count = int(data[6])
            self.collision_scan_reported_alive = bool(data[7])
        if len(data) >= 10:
            self.collision_approach_point_count = int(data[8])
            self.collision_approach_nearest_x = (
                float(data[9]) / 1000.0 if data[9] < 9999 else math.inf)
        if len(data) >= 11:
            self.collision_self_filtered_point_count = int(data[10])

    def _on_odom(self, msg: Odometry) -> None:
        now = time.perf_counter()
        if (
            not bool(self.get_parameter("nav_arc_adaptive_enabled").value)
            or not self.arc_learning_active
            or now - self.arc_learning_since
            < float(self.get_parameter("nav_arc_gain_settle_sec").value)
            or now - self.last_arc_gain_update
            < float(
                self.get_parameter("nav_arc_gain_update_period_sec").value)
        ):
            return
        self.arc_gain_estimator.observe(
            self.arc_learning_desired_v,
            self.arc_learning_desired_w,
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.angular.z),
        )
        self.last_arc_gain_update = now

    def _collision_blocked(self) -> bool:
        return self.collision_stop

    def _collision_scan_alive(self) -> bool:
        return (
            self.collision_scan_reported_alive
            and self._alive(
                self.collision_status_time,
                "collision_status_timeout_sec",
            )
        )

    def _alive(self, timestamp: float, parameter: str) -> bool:
        return (
            time.perf_counter() - timestamp
            <= float(self.get_parameter(parameter).value)
        )

    def _depth_alive(self) -> bool:
        return self._alive(self.depth_time, "depth_timeout_sec")

    def _local_cloud_alive(self) -> bool:
        return self._alive(
            self.local_cloud_time, "local_cloud_timeout_sec")

    def _navigation_sensor_healthy(self) -> bool:
        depth_ok = (
            not bool(self.get_parameter("require_depth_alive").value)
            or self._depth_alive()
        )
        cloud_ok = (
            not bool(self.get_parameter("require_local_cloud_alive").value)
            or self._local_cloud_alive()
        )
        return depth_ok and cloud_ok and self._collision_scan_alive()

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
        raw_nav_v = nav_v
        raw_nav_w = nav_w
        radius = float(self.get_parameter("wheel_track_w").value)
        self.last_arc_gain_applied = False
        self.last_pure_turn_lift_applied = False
        self.last_approach_scale = 1.0
        arc_compensation_candidate = (
            self.active_command_source == "nav2"
            and abs(nav_v)
            >= float(self.get_parameter("nav_arc_gain_min_v").value)
            and abs(nav_w) > 0.02
            and abs(nav_v / nav_w)
            >= float(self.get_parameter("nav_arc_gain_min_radius").value)
        )
        if arc_compensation_candidate:
            if not self.arc_learning_active:
                self.arc_learning_since = now
            self.arc_learning_active = True
            self.arc_learning_desired_v = raw_nav_v
            self.arc_learning_desired_w = raw_nav_w
        else:
            self.arc_learning_active = False

        if (
            self.active_command_source == "nav2"
            and abs(nav_v) <= 0.02
            and 0.02 < abs(nav_w)
            < float(self.get_parameter("nav_pure_turn_min_w").value)
        ):
            nav_w = math.copysign(
                float(self.get_parameter("nav_pure_turn_min_w").value),
                nav_w,
            )
            self.last_pure_turn_lift_applied = True

        if arc_compensation_candidate:
            nav_w = clamp(
                nav_w * self.arc_gain_estimator.gain,
                -float(self.get_parameter("nav_arc_max_w").value),
                float(self.get_parameter("nav_arc_max_w").value),
            )
            nav_v, nav_w = self._limit_side_speed(
                nav_v,
                nav_w,
                radius,
                float(self.get_parameter("nav_arc_outer_wheel_mps").value),
            )
            self.last_arc_gain_applied = True

        if (
            self.active_command_source == "nav2"
            and nav_v > 0.02
            and math.isfinite(self.collision_approach_nearest_x)
        ):
            stop_distance = float(
                self.get_parameter("approach_stop_distance_m").value)
            slow_distance = max(
                stop_distance + 0.05,
                float(self.get_parameter("approach_slow_distance_m").value),
            )
            if self.collision_approach_nearest_x < slow_distance:
                scale = clamp(
                    (
                        self.collision_approach_nearest_x - stop_distance
                    ) / (slow_distance - stop_distance),
                    float(self.get_parameter("approach_min_scale").value),
                    1.0,
                )
                # Preserve curvature while preventing the approach gate from
                # pushing an already regulated command below the chassis'
                # effective navigation speed. A true collision still takes
                # the hard-stop branch below.
                min_linear_speed = max(
                    0.0,
                    float(self.get_parameter(
                        "approach_min_linear_speed_mps").value),
                )
                if nav_v > 0.0 and min_linear_speed > 0.0:
                    scale = max(
                        scale,
                        min(1.0, min_linear_speed / nav_v),
                    )
                nav_v *= scale
                nav_w *= scale
                self.last_approach_scale = scale

        baseline_blocked = bool(
            self.get_parameter("require_depth_baseline").value
        ) and self.baseline_seen and not self.baseline_ready
        depth_alive = self._depth_alive()
        depth_required_missing = bool(
            self.get_parameter("require_depth_alive").value
        ) and not depth_alive
        local_cloud_required_missing = bool(
            self.get_parameter("require_local_cloud_alive").value
        ) and not self._local_cloud_alive()
        scan_alive = self._collision_scan_alive()
        front_collision = nav_v > 0.02 and self._collision_blocked()
        rear_collision = nav_v < -0.02 and (
            self.collision_rear_stop or not scan_alive)
        turn_needs_swept_circle = (
            abs(nav_w) > 0.02
            and (
                # Only a true stop-and-turn uses the full swept circle.
                # The former 0.04 m/s boundary made low-speed MPPI arcs
                # alternate between moving and collision-locked each cycle.
                abs(nav_v) <= 0.015
                or (
                    abs(nav_w) >= 0.14
                    and abs(nav_v / nav_w) < 0.65
                )
            )
        )
        rotation_collision = turn_needs_swept_circle and (
            self.collision_rotation_stop or not scan_alive)
        collision_blocked = (
            front_collision or rear_collision or rotation_collision)

        if (
            self.software_estop
            or baseline_blocked
            or depth_required_missing
            or local_cloud_required_missing
            or collision_blocked
        ):
            self.controller.reset()
            result = self.controller.update(
                0.0, 0.0, DepthSample(), now, dt, depth_alive=False)
            if self.software_estop:
                self.active_command_source = "software_estop"
            elif baseline_blocked:
                self.active_command_source = "baseline_lock"
            elif local_cloud_required_missing:
                self.active_command_source = "local_cloud_timeout_lock"
            elif collision_blocked:
                if rear_collision:
                    self.active_command_source = "rear_scan_collision_lock"
                elif rotation_collision:
                    self.active_command_source = "rotation_scan_collision_lock"
                else:
                    self.active_command_source = "front_collision_lock"
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
            allow_steering_bias=self.active_command_source != "nav2",
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
        if not rclpy.ok():
            return
        result = self._fuse()
        safe = Twist()
        safe.linear.x = result.linear_x
        safe.angular.z = result.angular_z
        self.last_safe_cmd = safe
        self.safe_pub.publish(safe)
        self.navigation_sensor_health_pub.publish(
            Bool(data=self._navigation_sensor_healthy()))

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
                f"raw=({self.nav_raw_cmd.linear.x:+.3f},"
                f"{self.nav_raw_cmd.angular.z:+.3f}) "
                f"raw_alive={self._alive(self.nav_raw_time, 'nav_timeout_sec')} "
                f"nav=({self.nav_cmd.linear.x:+.3f},"
                f"{self.nav_cmd.angular.z:+.3f}) "
                f"safe=({safe.linear.x:+.3f},{safe.angular.z:+.3f}) "
                f"depth_alive={self._depth_alive()} "
                f"local_cloud_alive={self._local_cloud_alive()} "
                f"collision={self._collision_blocked()} "
                f"collision_points={self.collision_point_count} "
                f"collision_x={self.collision_nearest_x:.3f} "
                f"rotation_collision={self.collision_rotation_stop} "
                f"rotation_points={self.collision_rotation_point_count} "
                f"rear_collision={self.collision_rear_stop} "
                f"rear_points={self.collision_rear_point_count} "
                f"self_filtered_points="
                f"{self.collision_self_filtered_point_count} "
                f"scan_alive={self._collision_scan_alive()} "
                f"nav_sensor_healthy={self._navigation_sensor_healthy()} "
                f"arc_gain={self.last_arc_gain_applied} "
                f"arc_gain_value={self.arc_gain_estimator.gain:.3f} "
                f"pure_turn_lift={self.last_pure_turn_lift_applied} "
                f"approach_points={self.collision_approach_point_count} "
                f"approach_x={self.collision_approach_nearest_x:.3f} "
                f"approach_scale={self.last_approach_scale:.2f} "
                f"require_depth={bool(self.get_parameter('require_depth_alive').value)} "
                f"require_cloud={bool(self.get_parameter('require_local_cloud_alive').value)} "
                f"require_baseline={bool(self.get_parameter('require_depth_baseline').value)} "
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
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        # Jazzy may invalidate the launch context before spin() returns.
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
