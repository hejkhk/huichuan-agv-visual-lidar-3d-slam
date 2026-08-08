from __future__ import annotations

import json
import logging
import math
import os
import threading
import time
from typing import Any, Callable

from backend.map_preview import map_signature, occupancy_grid_png

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
    _RCLPY_IMPORT_ERROR = None
except ImportError as exc:
    _HAS_RCLPY = False
    _RCLPY_IMPORT_ERROR = exc


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Convert a normalized ROS quaternion to map-frame yaw in radians."""

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(sin_yaw, cos_yaw)


class Ros2Client:
    """Own an isolated ROS context and cache callbacks without blocking Qt.

    Topic names are configurable through ``ROBOT_UI_*_TOPIC`` variables while
    documented defaults remain stable. Callbacks forward normalized values to
    ``TeamRobotApi``; they never touch QObject/QML state directly. ``close``
    must be called during application shutdown to stop the executor thread.
    """

    def __init__(self, owner: Any):
        if not _HAS_RCLPY:
            raise RuntimeError(
                "ROS 2 Python imports are unavailable: "
                f"{_RCLPY_IMPORT_ERROR}"
            )
        self.owner = owner
        self.log = logging.getLogger("ROS")
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = Node("robot_touch_ui", context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self._closed = False
        self._last_map_signature: tuple[Any, ...] | None = None
        self._map_topic = os.getenv("ROBOT_UI_MAP_TOPIC", "/map")
        self._map_message_count = 0
        self._last_map_diagnostic_state: tuple[int, int] | None = None
        self._last_graph_diagnostic_state: tuple[int, int, int] | None = None
        self._empty_graph_checks = 0
        self._started_at = time.monotonic()

        # --- Map: /map (latched, from map_server) ---
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._map_subscription = self.node.create_subscription(
            OccupancyGrid,
            self._map_topic,
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
        self._map_diagnostic_timer = self.node.create_timer(
            5.0, self._report_map_subscription
        )
        self._graph_diagnostic_timer = self.node.create_timer(
            2.0, self._report_ros_graph
        )

        self.thread = threading.Thread(target=self._spin, name="robot-ui-ros2", daemon=True)
        self.thread.start()
        self.log.info("ROS 2 订阅已启动")

    def _report_ros_graph(self) -> None:
        map_publishers = self.node.count_publishers(self._map_topic)
        scan_topic = os.getenv(
            "ROBOT_UI_SCAN_TOPIC", "/scan_timed_v2_filtered"
        )
        odom_topic = os.getenv("ROBOT_UI_ODOM_TOPIC", "/odom")
        scan_publishers = self.node.count_publishers(scan_topic)
        odom_publishers = self.node.count_publishers(odom_topic)
        state = (
            int(map_publishers),
            int(scan_publishers),
            int(odom_publishers),
        )
        graph_connected = any(state)
        if graph_connected:
            self._empty_graph_checks = 0
            self.owner.update_ros_connection(True)
        else:
            self._empty_graph_checks += 1
            if (
                self._empty_graph_checks >= 3
                and time.monotonic() - self._started_at >= 8.0
            ):
                self.owner.update_ros_connection(
                    False,
                    "DDS 未发现 /map、/scan_timed_v2_filtered 或 /odom 发布者",
                )
        if state == self._last_graph_diagnostic_state:
            return
        self._last_graph_diagnostic_state = state
        log_method = self.log.info if graph_connected else self.log.warning
        log_method(
            "ROS_GRAPH_HEALTH map=%d scan=%d odom=%d domain=%s rmw=%s dds=%s",
            map_publishers,
            scan_publishers,
            odom_publishers,
            os.getenv("ROS_DOMAIN_ID", "unset"),
            os.getenv("RMW_IMPLEMENTATION", "default"),
            os.getenv("CYCLONEDDS_URI", "unset"),
        )

    def _spin(self) -> None:
        try:
            self.executor.spin()
        except Exception:
            if not self._closed:
                self.log.exception("ROS 2 executor 异常退出")

    def _report_map_subscription(self) -> None:
        primary_publishers = self.node.count_publishers(self._map_topic)
        state = (
            int(primary_publishers),
            int(self._map_message_count),
        )
        if state == self._last_map_diagnostic_state:
            return
        self._last_map_diagnostic_state = state
        environment = (
            f"domain={os.getenv('ROS_DOMAIN_ID', 'unset')} "
            f"rmw={os.getenv('RMW_IMPLEMENTATION', 'default')}"
        )
        if self._map_message_count:
            self.log.info(
                "地图订阅正常：immutable %s publishers=%d，messages=%d，%s",
                self._map_topic,
                primary_publishers,
                self._map_message_count,
                environment,
            )
        elif primary_publishers:
            self.log.warning(
                "已发现不可变地图发布者但尚未收到兼容数据：%s publishers=%d，%s",
                self._map_topic,
                primary_publishers,
                environment,
            )
        else:
            self.log.warning(
                "尚未发现不可变地图发布者：%s，当前继续显示 Loc_MAP 文件预览，%s",
                self._map_topic,
                environment,
            )

    def _map_callback(
        self,
        message: OccupancyGrid,
        *,
        topic: str | None = None,
    ) -> None:
        logger = getattr(self, "log", logging.getLogger("ROS"))
        topic = topic or getattr(self, "_map_topic", "/map")
        info = message.info
        width = int(info.width)
        height = int(info.height)
        if width <= 0 or height <= 0 or len(message.data) != width * height:
            logger.error(
                "拒绝无效地图：topic=%s size=%dx%d data=%d",
                topic,
                width,
                height,
                len(message.data),
            )
            return
        try:
            signature = map_signature(
                message.data,
                width,
                height,
                float(info.resolution),
                float(info.origin.position.x),
                float(info.origin.position.y),
            )
        except (TypeError, ValueError, OverflowError):
            logger.exception("地图数据转换失败：topic=%s", topic)
            return
        expected = getattr(self.owner, "reference_map_signature", None)
        if expected is not None and signature != expected:
            logger.error(
                "拒绝与所选 Loc_MAP 不一致的 /map：expected_crc=0x%08x "
                "incoming_crc=0x%08x size=%dx%d",
                expected[-1],
                signature[-1],
                width,
                height,
            )
            return
        if signature == self._last_map_signature:
            return

        image = occupancy_grid_png(message.data, width, height)
        if not image:
            logger.error("地图 PNG 编码失败：topic=%s size=%dx%d", topic, width, height)
            return
        self._last_map_signature = signature
        self._map_message_count = getattr(self, "_map_message_count", 0) + 1
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
        logger.info(
            "地图帧已接收：source=%s immutable=true size=%dx%d resolution=%.3fm "
            "origin=(%.3f,%.3f) crc=0x%08x revision=%d",
            topic,
            width,
            height,
            float(info.resolution),
            float(info.origin.position.x),
            float(info.origin.position.y),
            signature[-1],
            self.owner.snapshot.map_revision,
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
