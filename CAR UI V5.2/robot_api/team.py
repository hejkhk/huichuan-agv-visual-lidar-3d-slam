from __future__ import annotations

import json
import logging
import math
import threading
import time
from pathlib import Path
from typing import Any

from .mock import MockRobotApi
from .stack_manager import SlamStackManager
from .types import ApiResult, NavigationState, Pose, RobotSnapshot


class TeamRobotApi(MockRobotApi):
    """Default real-device adapter with ROS 2 and serial cache integration.

    The class deliberately inherits the complete Mock implementation so
    unavailable team services remain demonstrable. Real navigation and
    telemetry methods override that behavior. ROS callbacks update
    ``RobotSnapshot`` under ``_snapshot_lock``; Qt only reads the cache through
    ``get_robot_snapshot``.

    Add real voice, vision, Wi-Fi, or OTA integrations by overriding the
    corresponding ``RobotApiBase`` method here (or in a subclass), not by
    calling device code from QML.
    """

    def __init__(self, data_dir: str | Path | None = None):
        super().__init__(data_dir)
        self.log = logging.getLogger("ROS")
        self._snapshot_lock = threading.RLock()
        self.snapshot.navigation_pause_supported = True
        self._stack = SlamStackManager(Path(__file__).resolve().parents[1])
        self.map_directory = (
            self._stack.project_root / "Loc_MAP"
            if self._stack.project_root is not None
            else Path(__file__).resolve().parents[1] / "map"
        )
        self.selected_map_id = str(self._stack.status().get("map_name", ""))
        self._ros = None
        try:
            from .ros2_client import Ros2Client

            self._ros = Ros2Client(self)
        except Exception as exc:
            self.snapshot.ros_error = str(exc)
            self.log.exception("ROS 2 初始化失败")

        # chassis_node is the sole STM32 serial owner. The UI only uses ROS.

    def get_robot_snapshot(self) -> ApiResult[RobotSnapshot]:
        with self._snapshot_lock:
            stack = self._stack.status()
            selected_map = str(stack.get("map_name", ""))
            if selected_map:
                self.selected_map_id = selected_map
            self.snapshot.slam_running = bool(stack["running"])
            self.snapshot.slam_mode = str(stack["mode"])
            self.snapshot.slam_message = (
                f"SLAM {stack['mode']} 运行中" if stack["running"] else "SLAM 系统未运行"
            )
            if stack["running"]:
                self.snapshot.mapping_active = stack["mode"] == "mapping"
                if self.snapshot.mapping_active:
                    self.snapshot.mapping_state = "MAPPING"
                    self.snapshot.mapping_message = "建图中"
            elif self.snapshot.mapping_active:
                self.snapshot.mapping_active = False
                self.snapshot.mapping_state = "IDLE"
                self.snapshot.mapping_message = "建图已停止"
            self.snapshot.timestamp = time.time()
            self._read_system_stats()
            return ApiResult.ok(self.snapshot)

    def get_current_pose(self) -> ApiResult[dict[str, float]]:
        with self._snapshot_lock:
            if not self.snapshot.pose_available:
                return ApiResult.fail("尚未收到机器人定位", "POSE_UNAVAILABLE")
            pose = self.snapshot.current_pose
            return ApiResult.ok({"x": pose.x, "y": pose.y, "yaw": pose.yaw})

    def start_single_navigation(self, point_id: str) -> ApiResult[None]:
        point = next((item for item in self.list_points().data or [] if item["id"] == point_id), None)
        if point is None:
            return ApiResult.fail("目标点不存在", "NOT_FOUND")
        return self._send_navigation_goal(point)

    def start_pose_navigation(self, x: float, y: float, yaw: float = 0.0) -> ApiResult[None]:
        return self._send_navigation_goal(
            {
                "id": "temporary-map-goal",
                "name": f"地图目标 ({x:.2f}, {y:.2f})",
                "x": x,
                "y": y,
                "yaw": yaw,
            }
        )

    def start_route_navigation(self, point_ids: list[str], ordered: bool = True) -> ApiResult[None]:
        if not point_ids:
            return ApiResult.fail("路径为空", "EMPTY_ROUTE")
        if self._ros is None:
            return ApiResult.fail(self.snapshot.ros_error or "ROS 2 未连接", "ROS_UNAVAILABLE")
        points_by_id = {item["id"]: item for item in self.list_points().data or []}
        missing = [point_id for point_id in point_ids if point_id not in points_by_id]
        if missing:
            return ApiResult.fail(f"目标点不存在: {', '.join(missing)}", "NOT_FOUND")
        points = [points_by_id[point_id] for point_id in point_ids]
        if not self._ros.send_route_goals(points, ordered=ordered):
            return ApiResult.fail("Nav2 多点导航 action server 不可用", "NAVIGATION_BRIDGE_UNAVAILABLE")
        with self._snapshot_lock:
            self.snapshot.navigation_state = NavigationState.STARTING.value
            self.snapshot.navigation_target = " → ".join(str(point["name"]) for point in points)
            self.snapshot.navigation_message = f"正在发送 {len(points)} 个导航点"
            self.snapshot.navigation_progress = 0
        return ApiResult.ok(message="多点导航已发送")

    def _send_navigation_goal(self, point: dict[str, Any]) -> ApiResult[None]:
        if self._ros is None:
            return ApiResult.fail(self.snapshot.ros_error or "ROS 2 未连接", "ROS_UNAVAILABLE")
        with self._snapshot_lock:
            if self.snapshot.map_available:
                minimum_x = self.snapshot.map_origin_x
                minimum_y = self.snapshot.map_origin_y
                maximum_x = minimum_x + self.snapshot.map_width * self.snapshot.map_resolution
                maximum_y = minimum_y + self.snapshot.map_height * self.snapshot.map_resolution
                point_x = float(point["x"])
                point_y = float(point["y"])
                if not (minimum_x <= point_x <= maximum_x and minimum_y <= point_y <= maximum_y):
                    return ApiResult.fail("目标点不在当前地图范围，请重新保存当前位置", "POINT_OUTSIDE_MAP")
        if not self._ros.send_navigation_goal(point):
            return ApiResult.fail("Nav2 action server 不可用", "NAVIGATION_BRIDGE_UNAVAILABLE")
        with self._snapshot_lock:
            self.snapshot.navigation_state = NavigationState.STARTING.value
            self.snapshot.navigation_target = str(point["name"])
            self.snapshot.navigation_message = "正在发送导航目标"
            self.snapshot.navigation_progress = 0
        return ApiResult.ok(message="导航目标已发送")

    def pause_navigation(self) -> ApiResult[None]:
        if self._ros is None or not self._ros.pause_navigation():
            return ApiResult.fail("当前没有可暂停的导航", "NO_ACTIVE_GOAL")
        with self._snapshot_lock:
            self.snapshot.navigation_state = NavigationState.PAUSED.value
            self.snapshot.navigation_message = "导航已暂停，目标已保留"
        return ApiResult.ok(message="导航已暂停")

    def resume_navigation(self) -> ApiResult[None]:
        if self._ros is None or not self._ros.resume_navigation():
            return ApiResult.fail("没有可继续的导航目标", "NO_PAUSED_GOAL")
        with self._snapshot_lock:
            self.snapshot.navigation_state = NavigationState.STARTING.value
            self.snapshot.navigation_message = "正在恢复导航"
        return ApiResult.ok(message="导航已继续")

    def cancel_navigation(self) -> ApiResult[None]:
        if self._ros is None or not self._ros.cancel_navigation():
            return ApiResult.fail("当前没有可取消的 Nav2 导航", "NO_ACTIVE_GOAL")
        with self._snapshot_lock:
            self.snapshot.navigation_message = "正在取消导航"
        return ApiResult.ok(message="已请求取消导航")

    def start_charging(self) -> ApiResult[None]:
        point = self.get_charging_point()
        if not point.success or point.data is None:
            return ApiResult.fail(point.message, point.error_code)
        result = self._send_navigation_goal(point.data)
        if result.success:
            with self._snapshot_lock:
                self.snapshot.charging = True
                self.snapshot.navigation_message = "正在返回充电点"
        return result

    def cancel_charging(self) -> ApiResult[None]:
        with self._snapshot_lock:
            self.snapshot.charging = False
        return self.cancel_navigation()

    def start_mapping(self) -> ApiResult[None]:
        result = self._stack.start("mapping")
        if not result.success:
            return result
        with self._snapshot_lock:
            self.snapshot.mapping_active = True
            self.snapshot.mapping_state = "STARTING"
            self.snapshot.mapping_message = "正在启动建图"
        return result

    def stop_mapping(self) -> ApiResult[None]:
        result = self._stack.stop()
        if result.success:
            with self._snapshot_lock:
                self.snapshot.mapping_active = False
                self.snapshot.mapping_state = "IDLE"
                self.snapshot.mapping_message = "建图已停止"
        return result

    def save_map(self, name: str) -> ApiResult[dict[str, Any]]:
        return self._stack.save_map(name)

    def load_map(self, yaml_path: str) -> ApiResult[dict[str, Any]]:
        prepared = self._stack.prepare_localization_map(yaml_path)
        if not prepared.success:
            return prepared
        started = self._stack.start("localization", str(prepared.data))
        if not started.success:
            return ApiResult.fail(started.message, started.error_code)
        self.selected_map_id = str(prepared.data)
        with self._snapshot_lock:
            self.snapshot.localization_ready = False
            self.snapshot.localization_state = "STARTING"
            self.snapshot.localization_detail = f"正在加载地图 {Path(yaml_path).stem}"
        return ApiResult.ok(prepared.data, message="地图已切换，正在自动重定位")

    def get_selected_map_id(self) -> str:
        """Return the persisted map selected by UI or terminal launchers."""

        selected = str(self._stack.status().get("map_name", ""))
        if selected:
            self.selected_map_id = selected
        return self.selected_map_id

    def start_slam_navigation(self) -> ApiResult[None]:
        return self._stack.start("navigation")

    def stop_slam_system(self) -> ApiResult[None]:
        return self._stack.stop()

    def update_map(self, payload: dict[str, Any]) -> None:
        with self._snapshot_lock:
            for key, value in payload.items():
                setattr(self.snapshot, key, value)
            self.snapshot.map_available = True
            self.snapshot.ros_connected = True
            self.snapshot.ros_error = ""
            self.snapshot.system_status = "ROS 已连接"

    def update_pose(self, x: float, y: float, yaw: float) -> None:
        with self._snapshot_lock:
            self.snapshot.current_pose = Pose(x=x, y=y, yaw=yaw)
            self.snapshot.pose_available = True
            self.snapshot.ros_connected = True
            self.snapshot.ros_error = ""

    def update_scan(self, points: list[list[float]]) -> None:
        with self._snapshot_lock:
            self.snapshot.laser_points = points
            self.snapshot.lidar_status = "NORMAL"
            self.snapshot.ros_connected = True

    def update_path(self, points: list[list[float]]) -> None:
        with self._snapshot_lock:
            self.snapshot.path_points = points

    def update_status(self, raw_status: str) -> None:
        try:
            status = json.loads(raw_status)
        except (json.JSONDecodeError, TypeError):
            return
        with self._snapshot_lock:
            if "battery_voltage" in status:
                self.snapshot.battery_voltage = float(status["battery_voltage"])
            state = str(status.get("state", ""))
            if state:
                self.snapshot.system_status = f"ROS {state}"
            self.snapshot.ros_connected = True

    def update_navigation_status(self, status: dict[str, Any]) -> None:
        state = str(status.get("state", "")).lower()
        active = bool(status.get("navigation_active", False))
        distance = status.get("distance_remaining")
        messages = {
            "idle": "请选择目标点",
            "goal_sending": "正在发送导航目标",
            "pausing_exploration": "正在停止自动建图",
            "navigating": "导航中",
            "canceling": "正在取消导航",
            "canceled": "导航已取消",
            "succeeded": "已到达目标点",
            "goal_rejected": "导航目标被拒绝",
            "failed": "导航失败",
            "cancel_failed": "取消导航失败",
        }
        with self._snapshot_lock:
            if state in {"goal_sending", "pausing_exploration"}:
                self.snapshot.navigation_state = NavigationState.STARTING.value
            elif state == "navigating" or active:
                self.snapshot.navigation_state = NavigationState.NAVIGATING.value
            elif state == "succeeded":
                self.snapshot.navigation_state = NavigationState.ARRIVED.value
                self.snapshot.navigation_progress = 100
                self.snapshot.charging = False
            elif state == "canceled":
                if self.snapshot.navigation_state != NavigationState.PAUSED.value:
                    self.snapshot.navigation_state = NavigationState.CANCELLED.value
                    self.snapshot.navigation_progress = 0
                    self.snapshot.charging = False
            elif state in {"failed", "goal_rejected", "cancel_failed"}:
                self.snapshot.navigation_state = NavigationState.FAILED.value
                self.snapshot.navigation_progress = 0
                self.snapshot.charging = False
            elif state == "idle" and not active:
                self.snapshot.navigation_state = NavigationState.IDLE.value
                self.snapshot.navigation_progress = 0
            message = messages.get(state, str(status.get("message", "导航状态已更新")))
            if state == "canceled" and self.snapshot.navigation_state == NavigationState.PAUSED.value:
                message = "导航已暂停，目标已保留"
            if state == "navigating" and isinstance(distance, (int, float)) and math.isfinite(distance):
                message += f" · 剩余 {distance:.1f} m"
            self.snapshot.navigation_message = message

    def update_navigation_goal_accepted(self, action_kind: str) -> None:
        with self._snapshot_lock:
            self.snapshot.navigation_state = NavigationState.NAVIGATING.value
            self.snapshot.navigation_message = (
                "多点导航中" if action_kind == "route" else "导航中"
            )

    def update_navigation_feedback(self, feedback: dict[str, Any]) -> None:
        distance = float(feedback.get("distance_remaining", 0.0))
        recoveries = int(feedback.get("number_of_recoveries", 0))
        with self._snapshot_lock:
            self.snapshot.navigation_state = NavigationState.NAVIGATING.value
            self.snapshot.navigation_message = f"导航中 · 剩余 {max(0.0, distance):.1f} m"
            self.snapshot.recovery_count = max(self.snapshot.recovery_count, recoveries)

    def update_recovery_status(self, status: dict[str, Any]) -> None:
        stage = str(status.get("stage", "tracking"))
        reason = str(status.get("reason", ""))
        with self._snapshot_lock:
            if stage != "tracking" and stage != self.snapshot.recovery_stage:
                self.snapshot.recovery_count += 1
            self.snapshot.recovery_stage = stage
            self.snapshot.recovery_reason = reason

    def update_localization_status(self, status: dict[str, Any]) -> None:
        with self._snapshot_lock:
            self.snapshot.localization_state = str(status.get("state", "unknown"))
            self.snapshot.localization_detail = str(
                status.get("detail", status.get("message", ""))
            )

    def update_localization_ready(self, ready: bool) -> None:
        with self._snapshot_lock:
            self.snapshot.localization_ready = ready
            if ready:
                self.snapshot.localization_state = "ready"
                if not self.snapshot.localization_detail:
                    self.snapshot.localization_detail = "重定位完成"

    def update_localization_bringup(self, detail: str) -> None:
        if not detail:
            return
        with self._snapshot_lock:
            self.snapshot.localization_detail = detail

    def update_mapping_status(self, status: dict[str, Any]) -> None:
        state = str(status.get("state", "")).lower()
        active = bool(status.get("mapping_active", False))
        with self._snapshot_lock:
            self.snapshot.mapping_active = active
            self.snapshot.mapping_state = state.upper() if state else ("MAPPING" if active else "IDLE")
            self.snapshot.mapping_message = "建图中" if active else ("建图完成" if state == "completed" else "")

    def update_charging_status(self, charging: bool) -> None:
        with self._snapshot_lock:
            self.snapshot.charging = charging
            self.snapshot.charging_status = "充电中" if charging else "未在充电"

    def release_control_to_gamepad(self) -> ApiResult[None]:
        if self._ros is None or not self._ros.release_control_to_gamepad():
            return ApiResult.fail("ROS 2 未连接，无法归还 PS2 控制权", "ROS_UNAVAILABLE")
        return ApiResult.ok(message="已通过 chassis_node 归还 PS2 控制权")

    def close(self) -> None:
        if self._ros is not None:
            self._ros.close()
