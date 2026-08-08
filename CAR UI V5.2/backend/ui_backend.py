from __future__ import annotations

import logging
import os
import platform
import shutil
import time
from collections import deque
from pathlib import Path
from typing import Any, Callable, Protocol

from PySide6.QtCore import Property, QCoreApplication, QObject, QThreadPool, QTimer, Signal, Slot

from robot_api.base import RobotApiBase
from robot_api.types import ApiResult, NavigationState
from .app_state import AppState
from .map_manager import MapManager
from .map_sync_manager import MapSyncManager
from .storage import JsonStorage
from .worker import ApiWorker


class MapImageSink(Protocol):
    def update_data_url(self, data_url: str) -> bool: ...

    def clear(self) -> None: ...


class UiBackend(QObject):
    """The only QObject exposed to QML as ``backend``.

    Public Qt Signal/Property/Slot names form a compatibility contract with
    all QML pages. Device calls are routed through ``RobotApiBase`` and
    ``ApiWorker`` so the GUI thread never performs ROS, serial, filesystem, or
    network work. Do not rename public Qt members without updating every QML
    binding and the contract tests.
    """

    dataChanged = Signal()
    snapshotChanged = Signal()
    busyChanged = Signal()
    notificationChanged = Signal()
    currentPoseReady = Signal("QVariantMap")
    recordingStateChanged = Signal()
    languageChanged = Signal()
    mapsChanged = Signal()
    mapOperationFinished = Signal(str, bool, str)
    mapImageChanged = Signal()
    mapOverlayChanged = Signal()
    navigationControlsChanged = Signal()
    systemInfoChanged = Signal()
    developerDataChanged = Signal()
    historyChanged = Signal()
    alertsChanged = Signal()

    def __init__(
        self,
        api: RobotApiBase,
        data_dir: str | Path,
        project_root: str | Path | None = None,
        map_image_sink: MapImageSink | None = None,
    ):
        super().__init__()
        self.api = api
        self.storage = getattr(api, "storage", JsonStorage(data_dir))
        self.state = AppState(self.storage)
        self.pool = QThreadPool.globalInstance()
        self.log = logging.getLogger("UI")
        self._workers: set[ApiWorker] = set()
        self._points: list[dict[str, Any]] = []
        self._voiceprints: list[dict[str, Any]] = []
        self._route_ids: list[str] = []
        self._map_goal: dict[str, float] = {}
        self._snapshot: dict[str, Any] = {}
        self._settings: dict[str, Any] = {}
        self._busy = False
        self._notification = ""
        self._recording_state = "NOT_STARTED"
        self._snapshot: dict[str, Any] = {}
        self._map_image_sink = map_image_sink
        self._map_image_source = ""
        self._map_image_revision = -1
        self._map_decode_worker: ApiWorker | None = None
        self._pending_map_decode: tuple[int, str] | None = None
        self._maps: list[dict[str, Any]] = []
        self._map_errors: list[dict[str, str]] = []
        self._map_operation_state: dict[str, Any] = {
            "status": "SYNCING",
            "message": "正在同步地图",
            "error_code": "",
            "map_id": "",
        }
        root = (
            Path(project_root).resolve()
            if project_root is not None
            else Path(data_dir).resolve()
        )
        self._project_root = root
        self._system_info: dict[str, Any] = {}
        self._developer_ui_log = ""
        self._developer_ros_log = ""
        self._developer_refreshing = False
        self._history: dict[str, deque[float]] = {
            "timestamps": deque(maxlen=1200),
            "battery_percent": deque(maxlen=1200),
            "cpu_percent": deque(maxlen=1200),
            "memory_percent": deque(maxlen=1200),
            "cpu_temperature": deque(maxlen=1200),
            "battery_voltage": deque(maxlen=1200),
            "vx": deque(maxlen=1200),
        }
        self._history_timer = QTimer(self)
        self._history_timer.setInterval(3000)
        self._history_timer.timeout.connect(self._sample_history)
        self._history_timer.start()
        self._alerts: deque[dict[str, Any]] = deque(maxlen=500)
        map_dir = Path(getattr(self.api, "map_directory", root / "map")).resolve()
        self.map_sync_manager = MapSyncManager(map_dir, root / "map_cache")
        self.map_manager = MapManager(
            map_dir,
            root / "map_cache",
            self.map_sync_manager,
            state_provider=lambda: dict(self._snapshot),
            map_loader=self.api.load_map,
            current_map_provider=getattr(
                self.api, "get_selected_map_id", None
            ),
        )
        self.map_sync_manager.start()
        self._refresh_system_info()
        self.refresh_data()
        self._language = self._settings.get("language", "zh")
        self._poll_timer = QTimer(self); self._poll_timer.setInterval(750); self._poll_timer.timeout.connect(self._poll_snapshot); self._poll_timer.start()
        self._poll_snapshot()

    def _set_busy(self, value: bool) -> None:
        if self._busy != value: self._busy = value; self.busyChanged.emit()

    def _notify(self, message: str) -> None:
        self._notification = message; self.notificationChanged.emit()

    @Property("QVariantMap", notify=historyChanged)
    def history(self) -> dict[str, list]:
        return {k: list(v) for k, v in self._history.items()}

    def _sample_history(self) -> None:
        snap = self._snapshot
        if not snap:
            return
        now = time.time()
        self._history["timestamps"].append(now)
        for key in ("battery_percent", "cpu_percent", "memory_percent",
                     "cpu_temperature", "battery_voltage", "vx"):
            self._history[key].append(float(snap.get(key, 0)))
        self.historyChanged.emit()

    @Property("QVariantList", notify=alertsChanged)
    def alerts(self) -> list[dict[str, Any]]:
        return list(self._alerts)

    @Slot()
    def clearAlerts(self) -> None:
        self._alerts.clear()
        self.alertsChanged.emit()

    def _detect_alerts(self, old: dict[str, Any], new: dict[str, Any]) -> None:
        now = time.time()
        new_alerts: list[dict[str, Any]] = []

        def add(level: str, category: str, key: str) -> None:
            new_alerts.append({
                "timestamp": now, "level": level, "category": category, "message_key": key,
            })

        batt = float(new.get("battery_percent", 100))
        old_batt = float(old.get("battery_percent", 100))
        if batt < 10 <= old_batt:
            add("ERROR", "battery", "电量严重不足")
        elif batt < 20 <= old_batt:
            add("WARNING", "battery", "电量偏低")

        cpu = float(new.get("cpu_percent", 0))
        old_cpu = float(old.get("cpu_percent", 0))
        if cpu > 85 >= old_cpu:
            add("WARNING", "system", "CPU 占用率过高")

        temp = float(new.get("cpu_temperature", 0))
        old_temp = float(old.get("cpu_temperature", 0))
        if temp > 85 >= old_temp:
            add("ERROR", "system", "核心温度过高")
        elif temp > 75 >= old_temp:
            add("WARNING", "system", "核心温度偏高")

        mem = float(new.get("memory_percent", 0))
        old_mem = float(old.get("memory_percent", 0))
        if mem > 85 >= old_mem:
            add("WARNING", "system", "内存占用率过高")

        nav = new.get("navigation_state", "")
        old_nav = old.get("navigation_state", "")
        if nav == "FAILED" and old_nav != "FAILED":
            add("ERROR", "navigation", "导航失败")
        elif nav == "ARRIVED" and old_nav != "ARRIVED":
            add("INFO", "navigation", "已到达目标点")

        charging = bool(new.get("charging", False))
        old_charging = bool(old.get("charging", False))
        if charging and not old_charging:
            add("INFO", "battery", "开始充电")

        ros = bool(new.get("ros_connected", True))
        old_ros = bool(old.get("ros_connected", True))
        if not ros and old_ros:
            add("ERROR", "connection", "ROS 2 连接断开")

        lidar = new.get("lidar_status", "NORMAL")
        old_lidar = old.get("lidar_status", "NORMAL")
        if lidar != "NORMAL" and old_lidar == "NORMAL":
            add("WARNING", "sensor", "激光雷达异常")

        encoder = new.get("encoder_status", "NORMAL")
        old_encoder = old.get("encoder_status", "NORMAL")
        if encoder != "NORMAL" and old_encoder == "NORMAL":
            add("WARNING", "sensor", "编码器异常")

        mapping = new.get("mapping_state", "")
        old_mapping = old.get("mapping_state", "")
        if mapping != old_mapping and mapping:
            add("INFO", "mapping", "建图状态变化")

        follow = new.get("follow_state", "")
        old_follow = old.get("follow_state", "")
        if follow != old_follow and follow:
            add("INFO", "follow", "跟随状态变化")

        if new_alerts:
            for alert in new_alerts:
                self._alerts.appendleft(alert)
            self.alertsChanged.emit()

    def _async(self, call: Callable[..., ApiResult[Any]], *args: Any, done: Callable[[ApiResult[Any]], None] | None = None) -> None:
        """Run one API operation in the shared pool and normalize UI feedback.

        ``call`` must return ``ApiResult``. Expected failures become a logged
        error plus operator notification; unexpected exceptions are converted
        by ``ApiWorker`` and never cross the Qt event loop.
        """

        self._set_busy(True)
        worker = ApiWorker(call, *args)
        self._workers.add(worker)
        action = getattr(call, "__name__", "api_call")
        self.log.info("调用 %s", action)
        def finish(result: ApiResult[Any]) -> None:
            self._workers.discard(worker)
            self._set_busy(bool(self._workers))
            if not result.success:
                self.log.error("%s 失败：%s (%s)", action, result.message, result.error_code)
                self._notify(result.message)
            elif result.message and result.message != "操作成功": self._notify(result.message)
            if result.success: self.log.info("%s 完成", action)
            if done: done(result)
        def fail(detail: str) -> None:
            self._workers.discard(worker)
            self._set_busy(bool(self._workers))
            self.log.error("%s 异常\n%s", action, detail)
            self._notify("接口异常，操作未完成")
        worker.signals.finished.connect(finish)
        worker.signals.error.connect(fail)
        self.pool.start(worker)

    def _poll_snapshot(self) -> None:
        """Copy the integration snapshot into a QVariant-friendly dictionary."""

        try:
            result = self.api.get_robot_snapshot()
            if result.success and result.data:
                snap = result.data
                map_revision = int(snap.map_revision)
                if map_revision != self._map_image_revision:
                    self._map_image_revision = map_revision
                    if snap.map_image:
                        if self._map_image_sink is None:
                            # Compatibility fallback for tests and embedders
                            # that construct UiBackend without a QML engine.
                            if snap.map_image != self._map_image_source:
                                self._map_image_source = snap.map_image
                                self.mapImageChanged.emit()
                        else:
                            self._queue_map_decode(map_revision, snap.map_image)
                    elif self._map_image_sink is not None:
                        self._pending_map_decode = None
                        self._map_image_sink.clear()
                        if self._map_image_source:
                            self._map_image_source = ""
                            self.mapImageChanged.emit()
                new_dict = {
                    "timestamp": snap.timestamp,
                    "battery_percent": snap.battery_percent,
                    "remaining_range_km": snap.remaining_range_km,
                    "upload_kbps": snap.upload_kbps,
                    "download_kbps": snap.download_kbps,
                    "network_connected": snap.network_connected,
                    "bluetooth_connected": snap.bluetooth_connected,
                    "system_status": snap.system_status,
                    "navigation_state": snap.navigation_state,
                    "navigation_target": snap.navigation_target,
                    "navigation_progress": snap.navigation_progress,
                    "navigation_message": snap.navigation_message,
                    "navigation_pause_supported": snap.navigation_pause_supported,
                    "voice_control_enabled": snap.voice_control_enabled,
                    "visual_follow_enabled": snap.visual_follow_enabled,
                    "charging": snap.charging,
                    "cpu_percent": snap.cpu_percent,
                    "memory_percent": snap.memory_percent,
                    "cpu_temperature": snap.cpu_temperature,
                    "encoder_status": snap.encoder_status,
                    "lidar_status": snap.lidar_status,
                    "voice_module_status": snap.voice_module_status,
                    "battery_voltage": snap.battery_voltage,
                    "charging_status": snap.charging_status,
                    "vx": snap.vx, "vy": snap.vy, "wz": snap.wz,
                    "ax": snap.ax, "ay": snap.ay, "az": snap.az,
                    "gx": snap.gx, "gy": snap.gy, "gz": snap.gz,
                    "current_pose": {"x": snap.current_pose.x, "y": snap.current_pose.y, "yaw": snap.current_pose.yaw},
                    "pose_available": snap.pose_available,
                    "ros_connected": snap.ros_connected,
                    "ros_error": snap.ros_error,
                    "map_available": snap.map_available,
                    # Keep the public field while replacing the expensive
                    # repeated base64 payload with the shared provider URL.
                    "map_image": self._map_image_source,
                    "map_width": snap.map_width,
                    "map_height": snap.map_height,
                    "map_resolution": snap.map_resolution,
                    "map_origin_x": snap.map_origin_x,
                    "map_origin_y": snap.map_origin_y,
                    "map_revision": snap.map_revision,
                    "mapping_state": snap.mapping_state,
                    "mapping_active": snap.mapping_active,
                    "mapping_message": snap.mapping_message,
                    "slam_running": snap.slam_running,
                    "slam_mode": snap.slam_mode,
                    "slam_message": snap.slam_message,
                    "localization_ready": snap.localization_ready,
                    "localization_state": snap.localization_state,
                    "localization_detail": snap.localization_detail,
                    "recovery_stage": snap.recovery_stage,
                    "recovery_reason": snap.recovery_reason,
                    "recovery_count": snap.recovery_count,
                    "laser_points": snap.laser_points,
                    "path_points": snap.path_points,
                    "detected_actors": snap.detected_actors,
                    "follow_state": snap.follow_state,
                    "follow_target": snap.follow_target,
                    "voice_state": snap.voice_state,
                    "speaker_name": snap.speaker_name,
                    "speaker_voiceprint": snap.speaker_voiceprint,
                }
                if new_dict != self._snapshot:
                    overlay_changed = any(
                        self._snapshot.get(key) != new_dict.get(key)
                        for key in ("laser_points", "path_points")
                    )
                    navigation_changed = any(
                        self._snapshot.get(key) != new_dict.get(key)
                        for key in (
                            "navigation_state",
                            "navigation_pause_supported",
                        )
                    )
                    old_snapshot = self._snapshot
                    self._snapshot = new_dict
                    if old_snapshot:
                        self._detect_alerts(old_snapshot, new_dict)
                    self.snapshotChanged.emit()
                    if overlay_changed:
                        self.mapOverlayChanged.emit()
                    if navigation_changed:
                        self.navigationControlsChanged.emit()
                self._update_map_properties()
        except Exception:
            self.log.exception("状态快照读取失败")

    def _queue_map_decode(self, revision: int, data_url: str) -> None:
        """Decode only the newest map image without blocking Qt's GUI thread."""

        self._pending_map_decode = (revision, data_url)
        if self._map_decode_worker is None:
            self._start_pending_map_decode()

    def _start_pending_map_decode(self) -> None:
        if self._pending_map_decode is None or self._map_image_sink is None:
            return
        revision, data_url = self._pending_map_decode
        self._pending_map_decode = None
        worker = ApiWorker(self._map_image_sink.update_data_url, data_url)
        self._map_decode_worker = worker

        def finish(success: object) -> None:
            self._map_decode_worker = None
            # When a newer map arrived during decoding, immediately decode
            # that one and never ask QML to upload the obsolete texture.
            if self._pending_map_decode is not None:
                self._start_pending_map_decode()
                return
            if bool(success) and revision == self._map_image_revision:
                source = f"image://live-map/current?revision={revision}"
                if source != self._map_image_source:
                    self._map_image_source = source
                    self.mapImageChanged.emit()
                    self.log.info("地图图像解码完成：revision=%d", revision)
            elif revision == self._map_image_revision:
                self.log.warning("地图图像解码失败，保留上一版本画面")

        def fail(detail: str) -> None:
            self._map_decode_worker = None
            self.log.error("地图图像后台解码异常\n%s", detail)
            if self._pending_map_decode is not None:
                self._start_pending_map_decode()

        worker.signals.finished.connect(finish)
        worker.signals.error.connect(fail)
        self.pool.start(worker)

    def _update_map_properties(self) -> None:
        maps = self.map_manager.refresh_maps()
        errors = self.map_manager.get_errors()
        operation = self.map_manager.get_map_operation_state()
        if (
            maps != self._maps
            or errors != self._map_errors
            or operation != self._map_operation_state
        ):
            self._maps = maps
            self._map_errors = errors
            self._map_operation_state = operation
            self.mapsChanged.emit()

    def _refresh_system_info(self) -> None:
        usage = shutil.disk_usage("/")
        try:
            os_release = platform.freedesktop_os_release()
            os_name = os_release.get("PRETTY_NAME", platform.platform())
        except OSError:
            os_name = platform.platform()
        self._system_info = {
            "product_name": "智能 AMR 小车",
            "host_model": "NVIDIA Jetson Orin Nano",
            "storage_total_gb": round(usage.total / (1024 ** 3), 1),
            "storage_used_gb": round(usage.used / (1024 ** 3), 1),
            "storage_free_gb": round(usage.free / (1024 ** 3), 1),
            "os_name": os_name,
            "kernel": platform.release(),
            "python_version": platform.python_version(),
            "ros_distro": os.getenv("ROS_DISTRO", "未检测到"),
            "ui_version": "V4.2",
        }
        self.systemInfoChanged.emit()

    def _read_developer_data(self) -> ApiResult[dict[str, str]]:
        def tail(path: Path, lines: int, missing: str) -> str:
            try:
                with path.open("r", encoding="utf-8", errors="replace") as handle:
                    return "".join(deque(handle, maxlen=lines)).rstrip()
            except FileNotFoundError:
                return missing
            except OSError as exc:
                return f"日志读取失败 ({path})：{exc}"

        ui_log_path = self._project_root / "logs" / "ui.log"
        launcher_log_path = self._project_root / "logs" / "slam_launcher.log"
        ui_lines = tail(ui_log_path, 220, "UI 日志尚未生成。请通过 run.sh 启动程序。")
        launcher_lines = tail(
            launcher_log_path,
            220,
            "SLAM 启动器日志尚未生成；尚未从 UI 启动 SLAM，或 SLAM 由外部终端启动。",
        )
        ui_lines = (
            f"=== UI LOG: {ui_log_path} ===\n{ui_lines}\n\n"
            f"=== SLAM LAUNCHER LOG: {launcher_log_path} ===\n{launcher_lines}"
        )

        topic_lines = [
            "ROS 2 公共消息与连接状态",
            f"ROS_DOMAIN_ID={os.getenv('ROS_DOMAIN_ID', '未设置')}",
            f"RMW_IMPLEMENTATION={os.getenv('RMW_IMPLEMENTATION', '默认')}",
            f"CYCLONEDDS_URI={os.getenv('CYCLONEDDS_URI', '未设置')}",
            f"API={type(self.api).__name__}",
        ]
        stack_manager = getattr(self.api, "_stack", None)
        if stack_manager is not None:
            stack = stack_manager.status()
            topic_lines.extend(
                [
                    f"SLAM 工程：{stack.get('project_root') or '未找到'}",
                    f"SLAM 进程：{'运行中' if stack.get('running') else '已停止'}，"
                    f"mode={stack.get('mode')}，pid={stack.get('pid')}，"
                    f"last_exit={stack.get('last_exit_code')}",
                    f"运行目录：{stack.get('run_dir') or '未生成'}",
                    f"启动日志：{stack.get('log_path')}",
                ]
            )
        ros_client = getattr(self.api, "_ros", None)
        if ros_client is not None:
            try:
                topics = ros_client.node.get_topic_names_and_types()
                watched = (
                    "/map",
                    "/robot_pose",
                    "/scan_timed_v2_filtered",
                    "/local_highres_cloud",
                    "/odom",
                    "/imu/data",
                    "/robot/status",
                    "/web/navigation_status",
                    "/web/mapping_status",
                    "/navigation/recovery_status",
                    "/cartographer_reloc/state",
                    "/localization_ready",
                )
                topic_lookup = {name: types for name, types in topics}
                topic_lines.append("")
                topic_lines.append("Topic 发布端（●=有发布者，○=无发布者）：")
                for name in watched:
                    types = topic_lookup.get(name, [])
                    publishers = ros_client.node.count_publishers(name)
                    topic_lines.append(
                        f"{'●' if publishers else '○'} {name}  pubs={publishers}  "
                        + (", ".join(types) if types else "未发现 Topic")
                    )

                services = {
                    name: types
                    for name, types in ros_client.node.get_service_names_and_types()
                }
                topic_lines.append("")
                topic_lines.append("关键服务/Action：")
                for name in (
                    "/navigate_to_pose/_action/send_goal",
                    "/navigate_through_poses/_action/send_goal",
                    "/write_state",
                    "/cartographer_reloc/trigger",
                ):
                    available = name in services
                    topic_lines.append(
                        f"{'●' if available else '○'} {name}  "
                        + (", ".join(services[name]) if available else "未发现服务")
                    )
            except Exception as exc:
                topic_lines.append(f"Topic 图读取失败：{exc}")
        else:
            topic_lines.append("○ ROS 2 客户端尚未连接")

        snapshot_result = self.api.get_robot_snapshot()
        if snapshot_result.success and snapshot_result.data:
            snap = snapshot_result.data
            topic_lines.extend(
                [
                    "",
                    f"/map：{snap.map_width} × {snap.map_height}，"
                    f"版本 {snap.map_revision}",
                    f"定位："
                    f"{'已定位' if snap.pose_available else '等待定位'}",
                    f"/scan_timed_v2_filtered：{snap.lidar_status}",
                    f"/odom：vx {snap.vx:.2f} m/s，wz {snap.wz:.2f} rad/s",
                    f"/robot/status：{snap.system_status}",
                    f"/web/navigation_status：{snap.navigation_state}，"
                    f"{snap.navigation_message}",
                ]
            )
        return ApiResult.ok(
            {
                "ui_log": ui_lines.rstrip(),
                "ros_log": "\n".join(topic_lines),
            }
        )

    @Slot()
    def refresh_data(self) -> None:
        points = self.api.list_points(); voices = self.api.list_voiceprints(); settings = self.api.get_settings()
        self._points = points.data or []; self._voiceprints = voices.data or []; self._settings = settings.data or {}; self.dataChanged.emit()

    @Property("QVariantList", notify=dataChanged)
    def points(self) -> list[dict[str, Any]]: return self._points

    @Property("QVariantList", notify=dataChanged)
    def recentPoints(self) -> list[dict[str, Any]]:
        ids = self._settings.get("recent_point_ids", [])
        lookup = {p["id"]: p for p in self._points}
        return [lookup[item] for item in ids if item in lookup][:3]

    @Property("QVariantList", notify=dataChanged)
    def voiceprints(self) -> list[dict[str, Any]]: return self._voiceprints

    @Property("QVariantList", notify=dataChanged)
    def routePoints(self) -> list[dict[str, Any]]:
        lookup = {p["id"]: p for p in self._points}
        return [lookup[item] for item in self._route_ids if item in lookup]

    @Property(bool, notify=dataChanged)
    def hasChargingPoint(self) -> bool: return any(p.get("is_charging_point") for p in self._points)

    @Property("QVariantMap", notify=snapshotChanged)
    def snapshot(self) -> dict[str, Any]: return self._snapshot

    @Property("QVariantMap", notify=dataChanged)
    def settings(self) -> dict[str, Any]: return self._settings

    @Property("QVariantList", notify=mapsChanged)
    def maps(self) -> list[dict[str, Any]]: return self._maps

    @Property("QVariantList", notify=mapsChanged)
    def mapErrors(self) -> list[dict[str, str]]: return self._map_errors

    @Property("QVariantMap", notify=mapsChanged)
    def currentMap(self) -> dict[str, Any]:
        return next((item for item in self._maps if item.get("is_current")), {})

    @Property("QVariantMap", notify=mapsChanged)
    def mapOperationState(self) -> dict[str, Any]: return self._map_operation_state

    @Property(str, notify=dataChanged)
    def selectedPointId(self) -> str: return self.state.selected_point_id

    @Property("QVariantMap", notify=dataChanged)
    def selectedPoint(self) -> dict[str, Any]:
        return next((point for point in self._points if point.get("id") == self.state.selected_point_id), {})

    @Property("QVariantMap", notify=dataChanged)
    def mapGoal(self) -> dict[str, float]: return self._map_goal

    @Property(bool, notify=dataChanged)
    def hasMapGoal(self) -> bool: return bool(self._map_goal)

    @Property("QVariantMap", notify=navigationControlsChanged)
    def navigationControls(self) -> dict[str, object]:
        state = self._snapshot.get("navigation_state", NavigationState.IDLE.value)
        if state == NavigationState.IDLE.value and (self.state.selected_point_id or self._map_goal): state = NavigationState.TARGET_SELECTED.value
        controls = self.state.navigation_controls(state)
        if not self._snapshot.get("navigation_pause_supported", True):
            controls["pauseEnabled"] = False
        return controls

    @Property(bool, notify=busyChanged)
    def busy(self) -> bool: return self._busy

    @Property(str, notify=notificationChanged)
    def notification(self) -> str: return self._notification

    @Property(str, notify=recordingStateChanged)
    def recordingState(self) -> str: return self._recording_state

    @Property(str, notify=languageChanged)
    def language(self) -> str: return self._language

    @Property(str, notify=mapImageChanged)
    def mapImageSource(self) -> str:
        """Shared, revisioned source used by every QML map Image."""

        return self._map_image_source

    @Property("QVariantMap", notify=systemInfoChanged)
    def systemInfo(self) -> dict[str, Any]:
        return self._system_info

    @Property(str, notify=developerDataChanged)
    def developerUiLog(self) -> str:
        return self._developer_ui_log

    @Property(str, notify=developerDataChanged)
    def developerRosLog(self) -> str:
        return self._developer_ros_log

    @Property(int, notify=dataChanged)
    def rosDomainId(self) -> int:
        """Configured ROS 2 DDS domain; changes take effect on next launch."""

        try:
            value = int(self._settings.get("ros_domain_id", 88))
        except (TypeError, ValueError):
            value = 99
        return max(0, min(232, value))

    @Slot()
    def refreshSystemInfo(self) -> None:
        self._refresh_system_info()

    @Slot()
    def refreshDeveloperData(self) -> None:
        if self._developer_refreshing:
            return
        self._developer_refreshing = True
        worker = ApiWorker(self._read_developer_data)
        self._workers.add(worker)

        def finish(result: ApiResult[Any]) -> None:
            self._workers.discard(worker)
            self._developer_refreshing = False
            if result.success and isinstance(result.data, dict):
                self._developer_ui_log = str(result.data.get("ui_log", ""))
                self._developer_ros_log = str(result.data.get("ros_log", ""))
                self.developerDataChanged.emit()

        def fail(detail: str) -> None:
            self._workers.discard(worker)
            self._developer_refreshing = False
            self.log.error("开发者数据读取失败\n%s", detail)

        worker.signals.finished.connect(finish)
        worker.signals.error.connect(fail)
        self.pool.start(worker)

    @Slot(int)
    def setRosDomainId(self, domain_id: int) -> None:
        """Persist a valid ROS 2 DDS domain without rebuilding live ROS nodes."""

        normalized = int(domain_id)
        if not 0 <= normalized <= 232:
            self._notify("DDS Domain ID 必须在 0 到 232 之间")
            return
        settings = self.storage.read("settings.json")
        settings["ros_domain_id"] = normalized
        self.storage.write("settings.json", settings)
        self._settings = settings
        self.dataChanged.emit()
        self._notify("DDS Domain ID 已保存，重启应用后生效")

    @Slot(bool)
    def setShowHomeTutorialOnStartup(self, enabled: bool) -> None:
        """Persist whether the non-operational home guide is offered at boot."""

        settings = self.storage.read("settings.json")
        settings["show_home_tutorial_on_startup"] = bool(enabled)
        self.storage.write("settings.json", settings)
        self._settings = settings
        self.dataChanged.emit()

    @Slot(int)
    def setPerformanceMode(self, mode: int) -> None:
        """Adjust only UI refresh cadence; robot safety services stay active."""
        intervals = {0: 1500, 1: 750, 2: 400}
        normalized = max(0, min(2, int(mode)))
        if self._settings.get("performance_mode") != normalized:
            settings = self.storage.read("settings.json")
            settings["performance_mode"] = normalized
            self.storage.write("settings.json", settings)
            self._settings = settings
            self.dataChanged.emit()
        self._poll_timer.setInterval(intervals[normalized])
        history_intervals = {0: 6000, 1: 3000, 2: 3000}
        self._history_timer.setInterval(history_intervals[normalized])

    @Slot(str)
    def setLanguage(self, language: str) -> None:
        if language not in {"zh", "en", "ru"} or language == self._language:
            return
        settings = self.storage.read("settings.json")
        settings["language"] = language
        self.storage.write("settings.json", settings)
        self._settings = settings
        self._language = language
        self.languageChanged.emit()
        self.dataChanged.emit()
        self.log.info("界面语言已切换：%s", language)

    @Slot(str)
    def selectPoint(self, point_id: str) -> None:
        self._map_goal = {}
        self.state.select_point(point_id)
        self.dataChanged.emit()
        self.navigationControlsChanged.emit()

    @Slot(float, float)
    def selectMapGoal(self, x: float, y: float) -> None:
        self.state.clear_selection()
        # A newly tapped destination inherits the vehicle's current heading;
        # the operator may then fine-tune it with the shared heading editor.
        pose = self._snapshot.get("current_pose", {})
        yaw = float(pose.get("yaw", 0.0)) if isinstance(pose, dict) else 0.0
        self._map_goal = {
            "x": float(x), "y": float(y), "yaw": yaw % (2.0 * 3.141592653589793)
        }
        self.dataChanged.emit()
        self.navigationControlsChanged.emit()

    @Slot(float)
    def setMapGoalYaw(self, yaw: float) -> None:
        if not self._map_goal:
            self._notify("请先在地图上选择目的地")
            return
        self._map_goal["yaw"] = float(yaw) % (2.0 * 3.141592653589793)
        self.dataChanged.emit()

    def _point_heading_block_reason(self) -> str:
        navigation = str(
            self._snapshot.get("navigation_state", NavigationState.IDLE.value)
        ).upper()
        if navigation in {
            NavigationState.STARTING.value,
            NavigationState.NAVIGATING.value,
            NavigationState.PAUSED.value,
        }:
            return "导航任务进行中，不能修改目标朝向"
        if bool(self._snapshot.get("mapping_active", False)) or str(
            self._snapshot.get("mapping_state", "IDLE")
        ).upper() == "MAPPING":
            return "正在创建地图，不能修改目标朝向"
        if bool(self._snapshot.get("charging", False)):
            return "正在执行回充任务，不能修改目标朝向"
        if str(self._map_operation_state.get("status", "")).upper() in {
            "LOADING", "LOADING_MAP"
        }:
            return "正在加载地图，不能修改目标朝向"
        return ""

    @Slot(str, float)
    def updatePointYaw(self, point_id: str, yaw: float) -> None:
        reason = self._point_heading_block_reason()
        if reason:
            self._notify(reason)
            return
        self._async(
            self.api.update_point_yaw,
            point_id,
            float(yaw),
            done=lambda result: self.refresh_data() if result.success else None,
        )

    @Slot()
    def clearNavigationSelection(self) -> None:
        self.state.clear_selection()
        self._map_goal = {}
        self.dataChanged.emit()
        self.navigationControlsChanged.emit()

    @Slot()
    def startSelectedNavigation(self) -> None:
        if self._map_goal:
            goal = dict(self._map_goal)
            self._async(
                self.api.start_pose_navigation,
                goal["x"],
                goal["y"],
                goal["yaw"],
                done=lambda result: self._poll_snapshot() if result.success else None,
            )
            return
        point_id = self.state.selected_point_id
        if not point_id: self._notify("请先选择目标点"); return
        def done(result: ApiResult[Any]) -> None:
            if result.success: self.state.record_recent([point_id]); self.refresh_data(); self._poll_snapshot()
        self._async(self.api.start_single_navigation, point_id, done=done)

    @Slot()
    def togglePauseNavigation(self) -> None:
        call = self.api.resume_navigation if self._snapshot.get("navigation_state") == "PAUSED" else self.api.pause_navigation
        self._async(call, done=lambda _r: self._poll_snapshot())

    @Slot()
    def cancelNavigation(self) -> None: self._async(self.api.cancel_navigation, done=lambda _r: self._poll_snapshot())

    def _finish_map_operation(
        self, action: str, map_id: str, result: ApiResult[Any]
    ) -> None:
        self._update_map_properties()
        self.mapOperationFinished.emit(action, result.success, map_id)

    def _synchronize_maps(self) -> ApiResult[list[dict[str, Any]]]:
        success = self.map_sync_manager.synchronize()
        maps = self.map_manager.refresh_maps(force=True)
        self.log.info(
            "地图目录同步结果：source=%s cache=%s maps=%d names=%s",
            self.map_sync_manager.map_dir,
            self.map_sync_manager.cache_dir,
            len(maps),
            ",".join(str(item.get("name", "")) for item in maps) or "none",
        )
        if not success:
            state = self.map_sync_manager.snapshot()
            errors = state.get("errors", [])
            message = (
                str(errors[0].get("error", "地图同步失败"))
                if errors
                else "地图同步失败"
            )
            return ApiResult.fail(message, "MAP_SYNC_FAILED")
        return ApiResult.ok(maps)

    @Slot()
    def refreshMaps(self) -> None:
        self._async(
            self._synchronize_maps,
            done=lambda result: self._finish_map_operation(
                "refresh", "", result
            ),
        )

    @Slot(str, str, result="QVariantMap")
    def mapActionAvailability(self, map_id: str, action: str) -> dict[str, Any]:
        return self.map_manager.action_availability(map_id, action)

    @Slot(str, str)
    def showMapBlockedReason(self, map_id: str, action: str) -> None:
        availability = self.map_manager.action_availability(map_id, action)
        if not availability.get("allowed", False):
            self._notify(str(availability.get("reason", "当前无法操作地图")))

    @Slot(str)
    def useMap(self, map_id: str) -> None:
        self._async(
            self.map_manager.select_map,
            map_id,
            done=lambda result: self._finish_map_operation(
                "use", map_id, result
            ),
        )

    @Slot(str, str)
    def renameMap(self, map_id: str, new_name: str) -> None:
        self._async(
            self.map_manager.rename_map,
            map_id,
            new_name,
            done=lambda result: self._finish_map_operation(
                "rename", map_id, result
            ),
        )

    @Slot(str)
    def deleteMap(self, map_id: str) -> None:
        self._async(
            self.map_manager.delete_map,
            map_id,
            done=lambda result: self._finish_map_operation(
                "delete", map_id, result
            ),
        )

    @Slot()
    def startMapping(self) -> None: self._async(self.api.start_mapping, done=lambda _r: self._poll_snapshot())

    @Slot()
    def stopMapping(self) -> None: self._async(self.api.stop_mapping, done=lambda _r: self._poll_snapshot())

    @Slot()
    def startSlamNavigation(self) -> None:
        self._async(self.api.start_slam_navigation, done=lambda _r: self._poll_snapshot())

    @Slot()
    def stopSlamSystem(self) -> None:
        self._async(self.api.stop_slam_system, done=lambda _r: self._poll_snapshot())

    @Slot(str)
    def saveMap(self, name: str) -> None: self._async(self.api.save_map, name, done=lambda r: self._poll_snapshot() if r.success else None)

    @Slot(str)
    def rvizAction(self, action: str) -> None:
        calls = {"in": self.api.rviz_zoom_in, "out": self.api.rviz_zoom_out, "reset": self.api.rviz_reset_view, "fullscreen": self.api.open_rviz_fullscreen}
        if action in calls: self._async(calls[action])

    @Slot()
    def requestCurrentPose(self) -> None:
        self._async(self.api.get_current_pose, done=lambda r: self.currentPoseReady.emit(r.data or {}) if r.success else None)

    @Slot(str, float, float, float, bool)
    def savePoint(self, name: str, x: float, y: float, yaw: float, charging: bool) -> None:
        self._async(self.api.save_point, name, x, y, yaw, charging, done=lambda r: self.refresh_data() if r.success else None)

    @Slot(str, str)
    def renamePoint(self, point_id: str, name: str) -> None: self._async(self.api.rename_point, point_id, name, done=lambda r: self.refresh_data() if r.success else None)

    @Slot(str)
    def deletePoint(self, point_id: str) -> None: self._async(self.api.delete_point, point_id, done=lambda r: self.refresh_data() if r.success else None)

    @Slot(str)
    def addRoutePoint(self, point_id: str) -> None:
        if point_id not in self._route_ids: self._route_ids.append(point_id); self.dataChanged.emit()

    @Slot(str)
    def removeRoutePoint(self, point_id: str) -> None:
        self._route_ids = [p for p in self._route_ids if p != point_id]; self.dataChanged.emit()

    @Slot()
    def clearRoute(self) -> None: self._route_ids.clear(); self.dataChanged.emit()

    @Slot(bool)
    def startRoute(self, ordered: bool) -> None:
        ids = list(self._route_ids)
        if not ids: self._notify("请先添加路径点"); return
        def done(result: ApiResult[Any]) -> None:
            if result.success: self.state.record_recent(ids); self.refresh_data(); self._poll_snapshot()
        self._async(self.api.start_route_navigation, ids, ordered, done=done)

    @Slot(bool)
    def setVoiceEnabled(self, enabled: bool) -> None: self._async(self.api.set_voice_control_enabled, enabled, done=lambda _r: self._poll_snapshot())

    @Slot(bool)
    def setFollowEnabled(self, enabled: bool) -> None: self._async(self.api.set_visual_follow_enabled, enabled, done=lambda _r: self._poll_snapshot())

    @Slot(bool)
    def setUnknownVoiceAllowed(self, enabled: bool) -> None:
        self._async(self.api.set_unknown_voice_control_allowed, enabled, done=lambda r: self.refresh_data() if r.success else None)

    @Slot()
    def startCharging(self) -> None: self._async(self.api.start_charging, done=lambda _r: self._poll_snapshot())

    @Slot(str)
    def selectActor(self, actor_id: str) -> None: self._async(self.api.select_follow_target, actor_id, done=lambda _r: self._poll_snapshot())

    @Slot(str)
    def startFollowing(self, actor_id: str) -> None: self._async(self.api.start_following, actor_id, done=lambda _r: self._poll_snapshot())

    @Slot()
    def stopFollowing(self) -> None: self._async(self.api.stop_following, done=lambda _r: self._poll_snapshot())

    @Slot()
    def releaseControlToGamepad(self) -> None:
        """Transmit the one-way lower-controller hand-off command.

        The HMI deliberately does not wait for or synthesize controller
        ownership feedback. HomePage owns the temporary display state.
        """
        self._async(self.api.release_control_to_gamepad)

    @Slot(str)
    def beginVoiceprint(self, name: str) -> None:
        self._recording_state = "RECORDING"; self.recordingStateChanged.emit()
        def done(result: ApiResult[Any]) -> None:
            if result.success:
                QTimer.singleShot(1600, self._voiceprint_ready)
            else: self._recording_state = "FAILED"; self.recordingStateChanged.emit()
        self._async(self.api.begin_voiceprint_recording, name, done=done)

    def _voiceprint_ready(self) -> None: self._recording_state = "READY"; self.recordingStateChanged.emit()

    @Slot(str)
    def saveVoiceprint(self, name: str) -> None:
        self._async(self.api.save_voiceprint, name, done=lambda r: self._voice_saved(r))

    def _voice_saved(self, result: ApiResult[Any]) -> None:
        if result.success: self._recording_state = "NOT_STARTED"; self.recordingStateChanged.emit(); self.refresh_data()

    @Slot()
    def cancelVoiceprintRecording(self) -> None: self._recording_state = "NOT_STARTED"; self.recordingStateChanged.emit(); self._async(self.api.cancel_voiceprint_recording)

    @Slot(str, str)
    def renameVoiceprint(self, voice_id: str, name: str) -> None: self._async(self.api.rename_voiceprint, voice_id, name, done=lambda r: self.refresh_data() if r.success else None)

    @Slot(str)
    def deleteVoiceprint(self, voice_id: str) -> None: self._async(self.api.delete_voiceprint, voice_id, done=lambda r: self.refresh_data() if r.success else None)

    @Slot(str, int)
    def moveVoiceprint(self, voice_id: str, direction: int) -> None:
        self._async(self.api.move_voiceprint, voice_id, direction, done=lambda r: self.refresh_data() if r.success else None)

    @Slot()
    def notifyVoiceprintLimit(self) -> None:
        self._notify("声纹已满，请先删除一个声纹")

    @Slot(int)
    def setVolume(self, value: int) -> None: self._async(self.api.set_volume, value, done=lambda r: self.refresh_data() if r.success else None)

    @Slot()
    def refreshSystemVolume(self) -> None:
        """Refresh settings so the volume page reflects the OS mixer."""

        def apply_settings(result: ApiResult[Any]) -> None:
            if result.success:
                self._settings = result.data or {}
                self.dataChanged.emit()

        self._async(self.api.get_settings, done=apply_settings)

    @Slot(str, "QVariant")
    def setParameter(self, name: str, value: Any) -> None: self._async(self.api.set_parameter, name, value, done=lambda r: self.refresh_data() if r.success else None)

    @Slot()
    def refreshWifi(self) -> None:
        self._async(self.api.list_wifi_networks, done=lambda r: self._on_wifi_list(r))

    def _on_wifi_list(self, result) -> None:
        if result.success:
            self._wifi_networks = result.data or []
            self.dataChanged.emit()

    @Property("QVariantList", notify=dataChanged)
    def wifiNetworks(self) -> list[dict[str, Any]]: return getattr(self, '_wifi_networks', [])

    @Slot(str, str)
    def connectWifi(self, ssid: str, password: str) -> None:
        self._async(self.api.connect_wifi, ssid, password, done=lambda r: self._on_wifi_connected(r, ssid))

    def _on_wifi_connected(self, result, ssid: str) -> None:
        if result.success:
            self._notify(f"已连接 {ssid}")
            self.refreshWifi()

    @Slot()
    def startOta(self) -> None: self._async(self.api.start_ota_upgrade)

    @Slot()
    def clearNotification(self) -> None: self._notification = ""; self.notificationChanged.emit()

    @Slot()
    def quit(self) -> None: QCoreApplication.quit()

    @Slot()
    def shutdown(self) -> None:
        self._poll_timer.stop()
        self._history_timer.stop()
        self.map_sync_manager.stop()
        self.pool.waitForDone(1500)
        close = getattr(self.api, "close", None)
        if close:
            close()
