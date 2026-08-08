from __future__ import annotations

import base64
import binascii
import json
import logging
import math
import os
import struct
import threading
import time
import zlib
from typing import Any, Callable

try:
    import rclpy
    from action_msgs.msg import GoalStatus
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from nav_msgs.msg import OccupancyGrid, Odometry, Path
    from rclpy.action.client import ActionClient
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from sensor_msgs.msg import Imu, LaserScan
    from std_msgs.msg import Bool, Float32, String
    from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
    _HAS_RCLPY = True
except ImportError:
    _HAS_RCLPY = False


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a normalized ROS quaternion to map-frame yaw in radians."""

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


def occupancy_grid_png(data: list[int], width: int, height: int) -> str:
    """Encode a ROS occupancy grid as a vertically corrected grayscale PNG data URL."""
    if width <= 0 or height <= 0 or len(data) != width * height:
        return ""

    def shade(value: int) -> int:
        if value < 0:
            return 205
        if value >= 65:
            return 35
        if value <= 10:
            return 247
        return max(45, 247 - round(value * 2.1))

    raw = bytearray()
    for row in range(height - 1, -1, -1):
        raw.append(0)
        start = row * width
        raw.extend(shade(value) for value in data[start : start + width])

    def chunk(kind: bytes, payload: bytes) -> bytes:
        checksum = binascii.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + kind + payload + struct.pack(">I", checksum)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")


class Ros2Client:
    """Own an isolated ROS context and cache callbacks without blocking Qt.

    Topic names are configurable through ``ROBOT_UI_*_TOPIC`` variables while
    documented defaults remain stable. Callbacks forward normalized values to
    ``TeamRobotApi``; they never touch QObject/QML state directly. ``close``
    must be called during application shutdown to stop the executor thread.
    """

    def __init__(self, owner: Any):
        self.owner = owner
        self.log = logging.getLogger("ROS")
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("robot_touch_ui", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._closed = False
        self._last_map_signature: tuple[Any, ...] | None = None

        # --- Map: /map (latched, from map_server) ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.node.create_subscription(
            OccupancyGrid,
            os.getenv("ROBOT_UI_MAP_TOPIC", "/map"),
            self._map_callback,
            map_qos,
        )

        # --- Pose: /amcl_pose (from AMCL) ---
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            os.getenv("ROBOT_UI_AMCL_TOPIC", "/amcl_pose"),
            self._amcl_pose_callback,
            10,
        )
        # --- Pose fallback: /localization_pose (from rtabmap) ---
        self.node.create_subscription(
            PoseWithCovarianceStamped,
            os.getenv("ROBOT_UI_LOCALIZATION_TOPIC", "/localization_pose"),
            self._amcl_pose_callback,
            10,
        )
        # Cartographer is the only map->odom authority in the Huichuan stack.
        self.node.create_subscription(
            PoseStamped,
            os.getenv("ROBOT_UI_ROBOT_POSE_TOPIC", "/robot_pose"),
            self._robot_pose_callback,
            10,
        )

        # --- Scan: /scan (from rplidar) ---
        self._last_scan_time = 0.0
        self.node.create_subscription(
            LaserScan,
            os.getenv("ROBOT_UI_SCAN_TOPIC", "/scan_timed_v2_filtered"),
            self._scan_callback,
            qos_profile_sensor_data,
        )

        # --- Path: /plan (from Nav2 planner) ---
        self.node.create_subscription(
            Path,
            os.getenv("ROBOT_UI_PATH_TOPIC", "/plan"),
            self._path_callback,
            10,
        )

        # --- Status topics (from nav_status_bridge) ---
        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.node.create_subscription(
            String,
            os.getenv("ROBOT_UI_STATUS_TOPIC", "/robot/status"),
            self._status_callback,
            10,
        )
        self.node.create_subscription(
            String,
            os.getenv("ROBOT_UI_NAV_STATUS_TOPIC", "/web/navigation_status"),
            self._navigation_status_callback,
            status_qos,
        )
        self.node.create_subscription(
            String,
            os.getenv("ROBOT_UI_MAPPING_STATUS_TOPIC", "/web/mapping_status"),
            self._mapping_status_callback,
            status_qos,
        )
        self.node.create_subscription(
            String,
            os.getenv("ROBOT_UI_RECOVERY_STATUS_TOPIC", "/navigation/recovery_status"),
            self._recovery_status_callback,
            status_qos,
        )
        self.node.create_subscription(
            String,
            os.getenv("ROBOT_UI_RELOCALIZATION_STATE_TOPIC", "/cartographer_reloc/state"),
            self._relocalization_state_callback,
            10,
        )
        self.node.create_subscription(
            String,
            os.getenv("ROBOT_UI_LOCALIZATION_BRINGUP_TOPIC", "/localization_bringup/state"),
            self._localization_bringup_callback,
            10,
        )
        self.node.create_subscription(
            Bool,
            os.getenv("ROBOT_UI_LOCALIZATION_READY_TOPIC", "/localization_ready"),
            self._localization_ready_callback,
            status_qos,
        )

        # --- Charging: /charging_status (from base_node) ---
        self.node.create_subscription(
            Bool,
            os.getenv("ROBOT_UI_CHARGING_STATUS_TOPIC", "/charging_status"),
            self._charging_status_callback,
            10,
        )

        # --- IMU: /imu/data (from base_node) ---
        self.node.create_subscription(
            Imu,
            os.getenv("ROBOT_UI_IMU_TOPIC", "/imu/data"),
            self._imu_callback,
            qos_profile_sensor_data,
        )

        # --- Odometry: /odom (from EKF) ---
        self.node.create_subscription(
            Odometry,
            os.getenv("ROBOT_UI_ODOM_TOPIC", "/odom"),
            self._odom_callback,
            qos_profile_sensor_data,
        )

        # --- Battery: /battery_voltage (from base_node or bridge) ---
        self.node.create_subscription(
            Float32,
            os.getenv("ROBOT_UI_BATTERY_TOPIC", "/battery_voltage"),
            self._battery_callback,
            10,
        )

        # --- Nav2 Action Client (replaces /web/nav_goal publisher) ---
        self._nav_client = ActionClient(
            self.node, NavigateToPose, '/navigate_to_pose'
        )
        self._route_client = ActionClient(
            self.node, NavigateThroughPoses, '/navigate_through_poses'
        )
        self._current_goal_handle = None
        self._active_goal_spec: tuple[str, Any] | None = None
        self._goal_serial = 0
        self._pause_requested = False
        self._cancel_requested = False
        self._web_control_pub = self.node.create_publisher(String, "/robot/web_control", 10)

        # --- Lidar status timer ---
        self._lidar_check_timer = self.node.create_timer(2.0, self._check_lidar_status)

        self.thread = threading.Thread(target=self._spin, name="robot-ui-ros2", daemon=True)
        self.thread.start()
        self.log.info("ROS 2 订阅已启动")

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception:
            if not self._closed:
                self.log.exception("ROS 2 executor 异常退出")

    def _map_callback(self, message: OccupancyGrid) -> None:
        info = message.info
        width = int(info.width)
        height = int(info.height)
        # Some map servers republish the same latched OccupancyGrid. Hash the
        # raw signed-byte payload before the expensive Python PNG conversion
        # so an unchanged map never rebuilds or reuploads a texture.
        if hasattr(message.data, "tobytes"):
            raw_bytes = message.data.tobytes()
        else:
            raw_bytes = bytes((int(value) & 0xFF) for value in message.data)
        signature = (
            width,
            height,
            float(info.resolution),
            float(info.origin.position.x),
            float(info.origin.position.y),
            zlib.crc32(raw_bytes) & 0xFFFFFFFF,
        )
        if signature == self._last_map_signature:
            return

        image = occupancy_grid_png(message.data, width, height)
        if not image:
            return
        self._last_map_signature = signature
        self.owner.update_map(
            {
                "map_image": image,
                "map_width": width,
                "map_height": height,
                "map_resolution": float(info.resolution),
                "map_origin_x": float(info.origin.position.x),
                "map_origin_y": float(info.origin.position.y),
                "map_revision": self.owner.snapshot.map_revision + 1,
            }
        )

    def _update_pose(self, pose: Any) -> None:
        position = pose.position
        orientation = pose.orientation
        yaw = quaternion_to_yaw(
            float(orientation.x),
            float(orientation.y),
            float(orientation.z),
            float(orientation.w),
        )
        self.owner.update_pose(float(position.x), float(position.y), yaw)

    def _amcl_pose_callback(self, message: PoseWithCovarianceStamped) -> None:
        self._update_pose(message.pose.pose)

    def _robot_pose_callback(self, message: PoseStamped) -> None:
        self._update_pose(message.pose)

    def _scan_callback(self, message: LaserScan) -> None:
        self._last_scan_time = time.time()
        with self.owner._snapshot_lock:
            self.owner.snapshot.lidar_status = "NORMAL"
        pose = self.owner.snapshot.current_pose
        points: list[list[float]] = []
        step = max(1, len(message.ranges) // 720)
        for index in range(0, len(message.ranges), step):
            distance = float(message.ranges[index])
            if not math.isfinite(distance) or distance < message.range_min or distance > message.range_max:
                continue
            angle = pose.yaw + message.angle_min + index * message.angle_increment
            points.append([pose.x + math.cos(angle) * distance, pose.y + math.sin(angle) * distance])
        self.owner.update_scan(points)

    def _check_lidar_status(self) -> None:
        if self._last_scan_time > 0 and time.time() - self._last_scan_time > 5.0:
            with self.owner._snapshot_lock:
                self.owner.snapshot.lidar_status = "DISCONNECTED"

    def _path_callback(self, message: Path) -> None:
        self.owner.update_path(
            [[float(item.pose.position.x), float(item.pose.position.y)] for item in message.poses]
        )

    def _status_callback(self, message: String) -> None:
        self.owner.update_status(message.data)

    def _navigation_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.owner.update_navigation_status(payload)

    def _mapping_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.owner.update_mapping_status(payload)

    def _recovery_status_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        if isinstance(payload, dict):
            self.owner.update_recovery_status(payload)

    def _relocalization_state_callback(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            parts = str(message.data).split("|", 1)
            payload = {
                "state": parts[0].strip() or "unknown",
                "detail": parts[1].strip() if len(parts) > 1 else "",
            }
        if isinstance(payload, dict):
            self.owner.update_localization_status(payload)

    def _localization_bringup_callback(self, message: String) -> None:
        self.owner.update_localization_bringup(str(message.data))

    def _localization_ready_callback(self, message: Bool) -> None:
        self.owner.update_localization_ready(bool(message.data))

    def _charging_status_callback(self, message: Bool) -> None:
        self.owner.update_charging_status(bool(message.data))

    def _imu_callback(self, message: Imu) -> None:
        with self.owner._snapshot_lock:
            self.owner.snapshot.ax = round(float(message.linear_acceleration.x) / 9.8, 3)
            self.owner.snapshot.ay = round(float(message.linear_acceleration.y) / 9.8, 3)
            self.owner.snapshot.az = round(float(message.linear_acceleration.z) / 9.8, 3)
            self.owner.snapshot.gx = round(float(message.angular_velocity.x) * 57.2958, 3)
            self.owner.snapshot.gy = round(float(message.angular_velocity.y) * 57.2958, 3)
            self.owner.snapshot.gz = round(float(message.angular_velocity.z) * 57.2958, 3)

    def _odom_callback(self, message: Odometry) -> None:
        with self.owner._snapshot_lock:
            self.owner.snapshot.vx = round(float(message.twist.twist.linear.x), 3)
            self.owner.snapshot.vy = round(float(message.twist.twist.linear.y), 3)
            self.owner.snapshot.wz = round(float(message.twist.twist.angular.z), 3)

    def _battery_callback(self, message: Float32) -> None:
        voltage = float(message.data)
        if voltage > 0:
            min_v, max_v = 21.0, 25.2
            percent = max(0, min(100, round((voltage - min_v) / (max_v - min_v) * 100)))
            with self.owner._snapshot_lock:
                self.owner.snapshot.battery_voltage = round(voltage, 1)
                self.owner.snapshot.battery_percent = percent

    def send_navigation_goal(self, point: dict[str, Any]) -> bool:
        if not self._nav_client.wait_for_server(timeout_sec=0.5):
            self.log.warning("Nav2 action server 不可用")
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = os.getenv("ROBOT_UI_MAP_FRAME", "map")
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(point["x"])
        goal.pose.pose.position.y = float(point["y"])
        yaw = float(point.get("yaw", 0.0))
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self._active_goal_spec = ("single", dict(point))
        self._goal_serial += 1
        goal_serial = self._goal_serial
        self._pause_requested = False
        self._cancel_requested = False
        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self._on_navigation_feedback)
        future.add_done_callback(
            lambda done: self._on_goal_response(done, "single", goal_serial))
        return True

    def send_route_goals(
        self, points: list[dict[str, Any]], ordered: bool = True
    ) -> bool:
        del ordered  # NavigateThroughPoses preserves the supplied route order.
        if not points or not self._route_client.wait_for_server(timeout_sec=0.5):
            self.log.warning("Nav2 navigate_through_poses action server 不可用")
            return False
        goal = NavigateThroughPoses.Goal()
        for point in points:
            pose = PoseStamped()
            pose.header.frame_id = os.getenv("ROBOT_UI_MAP_FRAME", "map")
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = float(point["x"])
            pose.pose.position.y = float(point["y"])
            yaw = float(point.get("yaw", 0.0))
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            goal.poses.append(pose)
        self._active_goal_spec = ("route", [dict(point) for point in points])
        self._goal_serial += 1
        goal_serial = self._goal_serial
        self._pause_requested = False
        self._cancel_requested = False
        future = self._route_client.send_goal_async(
            goal, feedback_callback=self._on_navigation_feedback)
        future.add_done_callback(
            lambda done: self._on_goal_response(done, "route", goal_serial))
        return True

    def _on_goal_response(self, future, action_kind: str, goal_serial: int) -> None:
        goal_handle = future.result()
        if goal_serial != self._goal_serial:
            if goal_handle and goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle or not goal_handle.accepted:
            self.log.warning("导航目标被拒绝")
            with self.owner._snapshot_lock:
                self.owner.snapshot.navigation_state = "FAILED"
                self.owner.snapshot.navigation_message = "导航目标被拒绝"
            return
        self._current_goal_handle = goal_handle
        self.owner.update_navigation_goal_accepted(action_kind)
        self.log.info("导航目标已接受")
        # Monitor result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done: self._on_goal_result(done, goal_handle, goal_serial))

    def _on_navigation_feedback(self, feedback_message) -> None:
        feedback = feedback_message.feedback
        payload = {
            "distance_remaining": float(getattr(feedback, "distance_remaining", 0.0)),
            "number_of_recoveries": int(getattr(feedback, "number_of_recoveries", 0)),
        }
        self.owner.update_navigation_feedback(payload)

    def _on_goal_result(self, future, goal_handle, goal_serial: int) -> None:
        result = future.result()
        if result is None or goal_serial != self._goal_serial:
            return
        status = result.status
        with self.owner._snapshot_lock:
            if status == GoalStatus.STATUS_SUCCEEDED:
                self.owner.snapshot.navigation_state = "ARRIVED"
                self.owner.snapshot.navigation_message = "已到达目标点"
                self.owner.snapshot.navigation_progress = 100
                self._active_goal_spec = None
            elif status == GoalStatus.STATUS_CANCELED and self._pause_requested:
                self.owner.snapshot.navigation_state = "PAUSED"
                self.owner.snapshot.navigation_message = "导航已暂停"
            elif status == GoalStatus.STATUS_CANCELED:
                self.owner.snapshot.navigation_state = "CANCELLED"
                self.owner.snapshot.navigation_message = "导航已取消"
                self._active_goal_spec = None
            else:
                self.owner.snapshot.navigation_state = "FAILED"
                stage = self.owner.snapshot.recovery_stage
                reason = self.owner.snapshot.recovery_reason
                suffix = f"（{stage}: {reason}）" if stage and reason else ""
                self.owner.snapshot.navigation_message = "导航失败" + suffix
                self._active_goal_spec = None
        if self._current_goal_handle is goal_handle:
            self._current_goal_handle = None

    def pause_navigation(self) -> bool:
        if self._current_goal_handle is None or self._active_goal_spec is None:
            return False
        self._pause_requested = True
        self._cancel_requested = False
        self._current_goal_handle.cancel_goal_async()
        with self.owner._snapshot_lock:
            self.owner.snapshot.navigation_state = "PAUSED"
            self.owner.snapshot.navigation_message = "正在暂停导航"
        return True

    def resume_navigation(self) -> bool:
        if self._active_goal_spec is None:
            return False
        action_kind, payload = self._active_goal_spec
        self._pause_requested = False
        if action_kind == "route":
            return self.send_route_goals(payload)
        return self.send_navigation_goal(payload)

    def cancel_navigation(self) -> bool:
        if self._current_goal_handle is None:
            return False
        self._pause_requested = False
        self._cancel_requested = True
        self._current_goal_handle.cancel_goal_async()
        self._current_goal_handle = None
        return True

    def release_control_to_gamepad(self) -> bool:
        message = String()
        message.data = json.dumps(
            {"command": "serial_command", "action": "ps2", "source": "car_ui"},
            separators=(",", ":"),
        )
        self._web_control_pub.publish(message)
        return True

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()
        self.context.try_shutdown()
        self.thread.join(timeout=1.0)
