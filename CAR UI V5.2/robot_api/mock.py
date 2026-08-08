from __future__ import annotations

import logging
import math
import time
import uuid
from pathlib import Path
from typing import Any

from backend.storage import JsonStorage
from backend.system_audio import SystemAudio
from .base import RobotApiBase
from .types import ApiResult, NavigationState, RobotSnapshot


class MockRobotApi(RobotApiBase):
    """Complete deterministic demo implementation backed by local JSON."""

    def __init__(self, data_dir: str | Path | None = None):
        self.log = logging.getLogger("API")
        self.storage = JsonStorage(data_dir or Path(__file__).resolve().parents[1] / "data")
        self.system_audio = SystemAudio()
        settings = self.storage.read("settings.json")
        self.snapshot = RobotSnapshot(voice_control_enabled=settings["voice_control_enabled"], visual_follow_enabled=settings["visual_follow_enabled"])
        self.snapshot.detected_actors = [{"id": "Actor1", "x": .56, "y": .58, "distance": 1.0}, {"id": "Actor2", "x": .34, "y": .33, "distance": 1.7}, {"id": "Actor3", "x": .15, "y": .38, "distance": 2.2}]
        self._nav_started = 0.0
        self._terminal_since = 0.0
        self._recording_name = ""
        self._sync_system_volume(settings)

    def _log(self, action: str) -> None:
        self.log.info(action)

    def _sync_system_volume(self, settings: dict) -> None:
        system_volume, _error = self.system_audio.read_volume()
        if system_volume is not None and settings.get("volume") != system_volume:
            settings["volume"] = system_volume
            self.storage.write("settings.json", settings)

    def _read_system_stats(self) -> None:
        # CPU usage from /proc/stat
        try:
            with open('/proc/stat') as f:
                parts = f.readline().split()
            idle = int(parts[4])
            total = sum(int(p) for p in parts[1:])
            if hasattr(self, '_prev_cpu'):
                d_idle = idle - self._prev_cpu[0]
                d_total = total - self._prev_cpu[1]
                if d_total > 0:
                    self.snapshot.cpu_percent = round(100.0 * (1.0 - d_idle / d_total), 1)
            self._prev_cpu = (idle, total)
        except Exception:
            pass
        # Memory usage from /proc/meminfo
        try:
            with open('/proc/meminfo') as f:
                info = {}
                for line in f:
                    parts = line.split(':')
                    if len(parts) == 2:
                        info[parts[0].strip()] = int(parts[1].strip().split()[0])
            total = info.get('MemTotal', 0)
            available = info.get('MemAvailable', 0)
            if total > 0:
                self.snapshot.memory_percent = round(100.0 * (1.0 - available / total), 1)
        except Exception:
            pass
        # CPU temperature from thermal zones
        try:
            for zone in range(10):
                path = f'/sys/class/thermal/thermal_zone{zone}/temp'
                with open(path) as f:
                    temp = int(f.read().strip())
                    if temp > 1000:
                        temp = temp / 1000.0
                    if 10 < temp < 120:
                        self.snapshot.cpu_temperature = round(temp, 1)
                        break
        except Exception:
            pass

    def get_robot_snapshot(self) -> ApiResult[RobotSnapshot]:
        now = time.time(); self.snapshot.timestamp = now
        self._read_system_stats()
        self.snapshot.charging_status = "充电中" if self.snapshot.charging else "未在充电"
        for index, actor in enumerate(self.snapshot.detected_actors): actor["distance"] = round(1 + index * .55 + math.sin(now / 3 + index) * .12, 1)
        if self.snapshot.navigation_state == NavigationState.NAVIGATING.value:
            elapsed = now - self._nav_started
            self.snapshot.navigation_progress = min(100, int(elapsed * 8))
            if self.snapshot.navigation_progress >= 100:
                self.snapshot.navigation_state = NavigationState.ARRIVED.value; self.snapshot.navigation_message = "已到达目标点"; self._terminal_since = now
        elif self.snapshot.navigation_state in {NavigationState.ARRIVED.value, NavigationState.CANCELLED.value} and self._terminal_since and now - self._terminal_since > 1.5:
            self.snapshot.navigation_state = NavigationState.IDLE.value; self.snapshot.navigation_message = "请选择目标点"; self.snapshot.navigation_progress = 0; self._terminal_since = 0.0
        return ApiResult.ok(self.snapshot)

    def get_current_pose(self) -> ApiResult[dict[str, float]]: return ApiResult.ok({"x": 6.46, "y": 65.64, "yaw": 0.0})
    def start_single_navigation(self, point_id: str) -> ApiResult[None]:
        point = next((p for p in self.storage.read("mock_points.json") if p["id"] == point_id), None)
        if not point: return ApiResult.fail("目标点不存在", "NOT_FOUND")
        self.snapshot.navigation_state = NavigationState.NAVIGATING.value; self.snapshot.navigation_target = point["name"]; self.snapshot.navigation_message = "导航中"; self.snapshot.navigation_progress = 0; self._nav_started = time.time(); self._log(f"navigate {point_id}"); return ApiResult.ok()
    def start_pose_navigation(self, x: float, y: float, yaw: float = 0.0) -> ApiResult[None]:
        self.snapshot.navigation_state = NavigationState.NAVIGATING.value; self.snapshot.navigation_target = f"地图目标 ({x:.2f}, {y:.2f})"; self.snapshot.navigation_message = "导航中"; self.snapshot.navigation_progress = 0; self._nav_started = time.time(); self._log(f"navigate pose x={x:.3f} y={y:.3f} yaw={yaw:.3f}"); return ApiResult.ok()
    def start_route_navigation(self, point_ids: list[str], ordered: bool = True) -> ApiResult[None]:
        if not point_ids: return ApiResult.fail("路径为空", "EMPTY_ROUTE")
        self.snapshot.navigation_state = NavigationState.NAVIGATING.value; self.snapshot.navigation_target = f"多点路线（{len(point_ids)}站）"; self.snapshot.navigation_message = "按顺序导航中" if ordered else "路线导航中"; self.snapshot.navigation_progress = 0; self._nav_started = time.time(); self._log(f"route {point_ids}"); return ApiResult.ok()
    def pause_navigation(self) -> ApiResult[None]: self.snapshot.navigation_state = NavigationState.PAUSED.value; self.snapshot.navigation_message = "导航已暂停"; return ApiResult.ok()
    def resume_navigation(self) -> ApiResult[None]: self.snapshot.navigation_state = NavigationState.NAVIGATING.value; self.snapshot.navigation_message = "导航中"; self._nav_started = time.time() - self.snapshot.navigation_progress / 8; return ApiResult.ok()
    def cancel_navigation(self) -> ApiResult[None]: self.snapshot.navigation_state = NavigationState.CANCELLED.value; self.snapshot.navigation_message = "导航已取消"; self._terminal_since = time.time(); return ApiResult.ok()
    def start_slam_navigation(self) -> ApiResult[None]:
        self.snapshot.slam_running = True; self.snapshot.slam_mode = "navigation"; self.snapshot.slam_message = "导航系统运行中"; return ApiResult.ok()
    def stop_slam_system(self) -> ApiResult[None]:
        self.snapshot.slam_running = False; self.snapshot.slam_mode = "stopped"; self.snapshot.slam_message = "SLAM 系统已停止"; return ApiResult.ok()
    def rviz_zoom_in(self) -> ApiResult[None]: self._log("rviz_zoom_in"); return ApiResult.ok()
    def rviz_zoom_out(self) -> ApiResult[None]: self._log("rviz_zoom_out"); return ApiResult.ok()
    def rviz_reset_view(self) -> ApiResult[None]: self._log("rviz_reset_view"); return ApiResult.ok()
    def open_rviz_fullscreen(self) -> ApiResult[None]: self._log("open_rviz_fullscreen"); return ApiResult.ok()
    def list_points(self) -> ApiResult[list[dict[str, Any]]]: return ApiResult.ok(self.storage.read("mock_points.json"))
    def save_point(self, name: str, x: float, y: float, yaw: float, is_charging_point: bool = False) -> ApiResult[dict[str, Any]]:
        points = self.storage.read("mock_points.json")
        if not name.strip(): return ApiResult.fail("名称不能为空", "EMPTY_NAME")
        if any(p["name"].casefold() == name.strip().casefold() for p in points): return ApiResult.fail("目标点名称已存在", "DUPLICATE_NAME")
        if is_charging_point:
            for p in points: p["is_charging_point"] = False
        point = {"id": uuid.uuid4().hex[:10], "name": name.strip(), "x": float(x), "y": float(y), "yaw": float(yaw), "is_charging_point": is_charging_point, "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "last_navigated_at": None}
        points.append(point); self.storage.write("mock_points.json", points); return ApiResult.ok(point, "目标点已保存")
    def rename_point(self, point_id: str, new_name: str) -> ApiResult[None]:
        points = self.storage.read("mock_points.json"); name = new_name.strip()
        if not name: return ApiResult.fail("名称不能为空", "EMPTY_NAME")
        if any(p["id"] != point_id and p["name"].casefold() == name.casefold() for p in points): return ApiResult.fail("目标点名称已存在", "DUPLICATE_NAME")
        for p in points:
            if p["id"] == point_id: p["name"] = name; self.storage.write("mock_points.json", points); return ApiResult.ok()
        return ApiResult.fail("目标点不存在", "NOT_FOUND")
    def update_point_yaw(self, point_id: str, yaw: float) -> ApiResult[None]:
        points = self.storage.read("mock_points.json")
        normalized = float(yaw) % (2.0 * math.pi)
        for point in points:
            if point["id"] == point_id:
                point["yaw"] = normalized
                self.storage.write("mock_points.json", points)
                return ApiResult.ok(message="目标朝向已保存")
        return ApiResult.fail("目标点不存在", "NOT_FOUND")
    def delete_point(self, point_id: str) -> ApiResult[None]:
        points = self.storage.read("mock_points.json"); filtered = [p for p in points if p["id"] != point_id]
        if len(filtered) == len(points): return ApiResult.fail("目标点不存在", "NOT_FOUND")
        self.storage.write("mock_points.json", filtered); return ApiResult.ok()
    def set_charging_point(self, point_id: str) -> ApiResult[None]:
        points = self.storage.read("mock_points.json"); found = False
        for p in points: p["is_charging_point"] = p["id"] == point_id; found |= p["id"] == point_id
        if not found: return ApiResult.fail("目标点不存在", "NOT_FOUND")
        self.storage.write("mock_points.json", points); return ApiResult.ok()
    def get_charging_point(self) -> ApiResult[dict[str, Any]]:
        point = next((p for p in self.storage.read("mock_points.json") if p["is_charging_point"]), None)
        return ApiResult.ok(point) if point else ApiResult.fail("尚未设置充电点", "NO_CHARGING_POINT")
    def start_charging(self) -> ApiResult[None]:
        point = self.get_charging_point()
        if not point.success: return ApiResult.fail(point.message, point.error_code)
        self.snapshot.charging = True; self.snapshot.navigation_state = NavigationState.NAVIGATING.value; self.snapshot.navigation_target = point.data["name"]; self.snapshot.navigation_message = "正在返回充电点"; self._nav_started = time.time(); return ApiResult.ok()
    def cancel_charging(self) -> ApiResult[None]: self.snapshot.charging = False; return self.cancel_navigation()
    def start_mapping(self) -> ApiResult[None]:
        self.snapshot.mapping_state = "MAPPING"; self.snapshot.mapping_active = True; self.snapshot.mapping_message = "建图中"; self._log("start_mapping"); return ApiResult.ok()
    def stop_mapping(self) -> ApiResult[None]:
        self.snapshot.mapping_state = "STOPPED"; self.snapshot.mapping_active = False; self.snapshot.mapping_message = "建图已停止"; self._log("stop_mapping"); return ApiResult.ok()
    def save_map(self, name: str) -> ApiResult[dict[str, Any]]:
        name = name.strip()
        if not name: return ApiResult.fail("名称不能为空", "EMPTY_NAME")
        self.snapshot.mapping_state = "COMPLETED"; self.snapshot.mapping_active = False; self.snapshot.mapping_message = "地图已保存"; self._log(f"save_map {name}"); return ApiResult.ok({"name": name})
    def load_map(self, yaml_path: str) -> ApiResult[dict[str, Any]]:
        path = Path(yaml_path)
        if not path.is_file() or path.suffix.casefold() != ".yaml":
            return ApiResult.fail("地图 YAML 不存在", "MAP_NOT_FOUND")
        self._loaded_map_yaml = str(path.resolve())
        self.snapshot.map_available = True
        self._log(f"load_map {path}")
        return ApiResult.ok(
            {"name": path.stem, "yaml_path": self._loaded_map_yaml},
            "地图加载成功",
        )
    def _save_setting(self, key: str, value: Any) -> ApiResult[None]:
        settings = self.storage.read("settings.json"); settings[key] = value; self.storage.write("settings.json", settings); return ApiResult.ok()
    def set_voice_control_enabled(self, enabled: bool) -> ApiResult[None]: self.snapshot.voice_control_enabled = enabled; return self._save_setting("voice_control_enabled", enabled)
    def set_visual_follow_enabled(self, enabled: bool) -> ApiResult[None]: self.snapshot.visual_follow_enabled = enabled; return self._save_setting("visual_follow_enabled", enabled)
    def set_unknown_voice_control_allowed(self, enabled: bool) -> ApiResult[None]: return self._save_setting("unknown_voice_allowed", enabled)
    def list_detected_actors(self) -> ApiResult[list[dict[str, Any]]]: return ApiResult.ok(self.snapshot.detected_actors)
    def select_follow_target(self, actor_id: str) -> ApiResult[None]: self.snapshot.follow_target = actor_id; self.snapshot.follow_state = "SELECTED"; return ApiResult.ok()
    def start_following(self, actor_id: str) -> ApiResult[None]: self.snapshot.follow_target = actor_id; self.snapshot.follow_state = "FOLLOWING"; return ApiResult.ok()
    def stop_following(self) -> ApiResult[None]: self.snapshot.follow_state = "IDLE"; return ApiResult.ok()
    def release_control_to_gamepad(self) -> ApiResult[None]:
        self._log("release control to gamepad")
        return ApiResult.ok()
    @staticmethod
    def _normalize_voiceprints(items: list[dict[str, Any]]) -> list[dict[str, Any]]:
        ordered = sorted(
            items,
            key=lambda item: (int(item.get("priority", 1_000_000)), str(item.get("id", ""))),
        )
        return [
            {**item, "priority": index}
            for index, item in enumerate(ordered, start=1)
        ]

    def _read_voiceprints(self) -> list[dict[str, Any]]:
        items = self._normalize_voiceprints(self.storage.read("mock_voiceprints.json"))
        self.storage.write("mock_voiceprints.json", items)
        return items

    def list_voiceprints(self) -> ApiResult[list[dict[str, Any]]]:
        return ApiResult.ok(self._read_voiceprints())

    def begin_voiceprint_recording(self, name: str) -> ApiResult[None]:
        if len(self._read_voiceprints()) >= 10:
            return ApiResult.fail("声纹已满，请先删除一个声纹", "VOICEPRINT_LIMIT")
        self._recording_name = name.strip()
        return ApiResult.ok() if self._recording_name else ApiResult.fail("名称不能为空", "EMPTY_NAME")
    def cancel_voiceprint_recording(self) -> ApiResult[None]: self._recording_name = ""; return ApiResult.ok()
    def save_voiceprint(self, name: str) -> ApiResult[dict[str, Any]]:
        items = self._read_voiceprints(); name = name.strip()
        if not name: return ApiResult.fail("名称不能为空", "EMPTY_NAME")
        if len(items) >= 10: return ApiResult.fail("声纹已满，请先删除一个声纹", "VOICEPRINT_LIMIT")
        if any(v["name"].casefold() == name.casefold() for v in items): return ApiResult.fail("声纹名称已存在", "DUPLICATE_NAME")
        item = {"id": uuid.uuid4().hex[:10], "name": name, "priority": len(items) + 1}; items.append(item); self.storage.write("mock_voiceprints.json", items); self._recording_name = ""; return ApiResult.ok(item)
    def rename_voiceprint(self, voiceprint_id: str, new_name: str) -> ApiResult[None]:
        items = self._read_voiceprints(); name = new_name.strip()
        if not name: return ApiResult.fail("名称不能为空", "EMPTY_NAME")
        if any(v["id"] != voiceprint_id and v["name"].casefold() == name.casefold() for v in items): return ApiResult.fail("声纹名称已存在", "DUPLICATE_NAME")
        for v in items:
            if v["id"] == voiceprint_id: v["name"] = name; self.storage.write("mock_voiceprints.json", items); return ApiResult.ok()
        return ApiResult.fail("声纹不存在", "NOT_FOUND")
    def delete_voiceprint(self, voiceprint_id: str) -> ApiResult[None]:
        items = self._read_voiceprints()
        if not any(item["id"] == voiceprint_id for item in items):
            return ApiResult.fail("声纹不存在", "NOT_FOUND")
        remaining = [item for item in items if item["id"] != voiceprint_id]
        self.storage.write("mock_voiceprints.json", self._normalize_voiceprints(remaining))
        return ApiResult.ok()

    def move_voiceprint(self, voiceprint_id: str, direction: int) -> ApiResult[None]:
        if direction not in {-1, 1}:
            return ApiResult.fail("优先级调整方向无效", "INVALID_DIRECTION")
        items = self._read_voiceprints()
        index = next((i for i, item in enumerate(items) if item["id"] == voiceprint_id), -1)
        if index < 0:
            return ApiResult.fail("声纹不存在", "NOT_FOUND")
        target = index + direction
        if target < 0 or target >= len(items):
            return ApiResult.ok()
        items[index], items[target] = items[target], items[index]
        reordered = [
            {**item, "priority": priority}
            for priority, item in enumerate(items, start=1)
        ]
        self.storage.write("mock_voiceprints.json", reordered)
        return ApiResult.ok()
    def get_settings(self) -> ApiResult[dict[str, Any]]:
        settings = self.storage.read("settings.json")
        self._sync_system_volume(settings)
        return ApiResult.ok(settings)

    def set_volume(self, value: int) -> ApiResult[None]:
        value = max(0, min(100, int(value)))
        success, detail = self.system_audio.set_volume(value)
        if not success:
            return ApiResult.fail(detail, "SYSTEM_AUDIO_UNAVAILABLE")
        return self._save_setting("volume", value)
    def set_parameter(self, name: str, value: Any) -> ApiResult[None]:
        settings = self.storage.read("settings.json"); settings.setdefault("parameters", {})[name] = value; self.storage.write("settings.json", settings); return ApiResult.ok()
    def list_wifi_networks(self) -> ApiResult[list[dict[str, Any]]]:
        import subprocess
        try:
            result = subprocess.run(
                ["nmcli", "-t", "-f", "IN-USE,SSID,SIGNAL,SECURITY", "device", "wifi", "list"],
                capture_output=True, text=True, timeout=10
            )
            networks = []
            seen = set()
            for line in result.stdout.strip().split("\n"):
                if not line:
                    continue
                parts = line.split(":")
                if len(parts) < 4:
                    continue
                in_use = parts[0] == "*"
                ssid = parts[1]
                if not ssid or ssid in seen:
                    continue
                seen.add(ssid)
                strength = int(parts[2]) if parts[2].isdigit() else 0
                secured = parts[3] != "" and parts[3] != "--"
                networks.append({"ssid": ssid, "strength": strength, "secured": secured, "connected": in_use})
            networks.sort(key=lambda n: (-n["connected"], -n["strength"]))
            return ApiResult.ok(networks)
        except Exception as e:
            self.log.error("获取 Wi-Fi 列表失败: %s", e)
            return ApiResult.fail("获取 Wi-Fi 列表失败", "WIFI_ERROR")

    def connect_wifi(self, ssid: str, password: str) -> ApiResult[None]:
        import subprocess
        try:
            if password:
                result = subprocess.run(
                    ["nmcli", "device", "wifi", "connect", ssid, "password", password],
                    capture_output=True, text=True, timeout=30
                )
            else:
                result = subprocess.run(
                    ["nmcli", "device", "wifi", "connect", ssid],
                    capture_output=True, text=True, timeout=30
                )
            if result.returncode == 0:
                return ApiResult.ok(message=f"已连接 {ssid}")
            else:
                error_msg = result.stderr.strip() or "连接失败"
                return ApiResult.fail(error_msg, "WIFI_CONNECT_ERROR")
        except Exception as e:
            self.log.error("连接 Wi-Fi 失败: %s", e)
            return ApiResult.fail("连接 Wi-Fi 失败", "WIFI_CONNECT_ERROR")
    def get_ota_status(self) -> ApiResult[dict[str, Any]]: return ApiResult.ok({"version": "1.4.2", "status": "已是最新版本", "progress": 0})
    def start_ota_upgrade(self) -> ApiResult[None]: self._log("start_ota_upgrade"); return ApiResult.ok(message="模拟升级已启动")
