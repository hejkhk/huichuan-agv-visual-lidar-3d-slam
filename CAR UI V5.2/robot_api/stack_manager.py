from __future__ import annotations

import os
import logging
import re
import shutil
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Any

import yaml

from .types import ApiResult


class SlamStackManager:
    """Start, stop and save the Huichuan stack without touching device ports.

    The robot launcher's stable runtime files are the ownership contract. This
    lets the UI attach to a stack started in a terminal and lets a terminal
    attach after the UI started it. Only the registered launcher PID is ever
    signalled.
    """

    SCRIPT_BY_MODE = {
        "mapping": "START_DUAL_2D_3D_MAPPING.sh",
        "navigation": "START_DUAL_2D_3D_NAVIGATION.sh",
        "localization": "START_DUAL_2D_3D_LOCALIZATION.sh",
    }

    def __init__(self, ui_root: str | Path):
        self.ui_root = Path(ui_root).resolve()
        self.project_root = self._find_project_root()
        self.state_dir = Path.home() / ".cache" / "huichuan_agv"
        self.log_dir = self.ui_root / "logs"
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.log = logging.getLogger("STACK")
        self._lock = threading.RLock()
        self._child: subprocess.Popen[Any] | None = None
        self._last_exit_code: int | None = None

    def _find_project_root(self) -> Path | None:
        configured = os.getenv("HUICHUAN_SLAM_ROOT", "").strip()
        candidates = []
        if configured:
            candidates.append(Path(configured).expanduser())
        candidates.extend(
            [
                # Production layout: this UI is vendored directly under the
                # Huichuan project root.
                self.ui_root.parent,
                self.ui_root.parent / "huichuan-agv-visual-lidar-3d-slam",
                Path.home() / "huichuan-agv-visual-lidar-3d-slam",
                Path.home() / "Desktop" / "huichuan-agv-visual-lidar-3d-slam",
                Path.home() / "桌面" / "huichuan-agv-visual-lidar-3d-slam",
                Path.home() / "视频" / "huichuan-agv-visual-lidar-3d-slam-step" /
                "huichuan-agv-ros2-foxy-main",
            ]
        )
        for candidate in candidates:
            candidate = candidate.resolve()
            if (candidate / "START_DUAL_2D_3D_MAPPING.sh").is_file():
                return candidate
        return None

    @staticmethod
    def _atomic_write_text(path: Path, value: str) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(f".{path.name}.tmp-{os.getpid()}")
        try:
            temporary.write_text(value.rstrip("\n") + "\n", encoding="utf-8")
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    @staticmethod
    def _read_text(path: Path) -> str:
        try:
            return path.read_text(encoding="utf-8").strip()
        except OSError:
            return ""

    @staticmethod
    def _pid_alive(pid: int) -> bool:
        if pid <= 1:
            return False
        try:
            os.kill(pid, 0)
            return True
        except OSError:
            return False

    def status(self) -> dict[str, Any]:
        if self._child is not None:
            return_code = self._child.poll()
            if return_code is not None:
                self._last_exit_code = return_code
                self._child = None
        pid_text = self._read_text(self.state_dir / "launcher.pid")
        pid = int(pid_text) if pid_text.isdigit() else 0
        running = self._pid_alive(pid)
        if not running and pid_text:
            for name in (
                "launcher.pid",
                "mode",
                "project_root",
                "run_dir",
                "active_map",
            ):
                try:
                    (self.state_dir / name).unlink(missing_ok=True)
                except OSError:
                    pass
        selected_map = self._read_text(self.state_dir / "selected_map")
        active_map = self._read_text(self.state_dir / "active_map") if running else ""
        return {
            "running": running,
            "pid": pid if running else 0,
            "mode": self._read_text(self.state_dir / "mode") if running else "stopped",
            "project_root": self._read_text(self.state_dir / "project_root"),
            "run_dir": self._read_text(self.state_dir / "run_dir"),
            "map_name": active_map or selected_map,
            "available": self.project_root is not None,
            "last_exit_code": self._last_exit_code,
            "log_path": str(self.log_dir / "slam_launcher.log"),
        }

    def start(self, mode: str, map_name: str = "") -> ApiResult[dict[str, Any]]:
        if mode not in self.SCRIPT_BY_MODE:
            return ApiResult.fail("未知的 SLAM 运行模式", "INVALID_STACK_MODE")
        if self.project_root is None:
            return ApiResult.fail(
                "未找到汇川 SLAM 工程，请设置 HUICHUAN_SLAM_ROOT",
                "SLAM_PROJECT_NOT_FOUND",
            )
        current = self.status()
        if current["running"]:
            same_localization_map = (
                mode == "localization"
                and bool(map_name)
                and current.get("map_name") == map_name
            )
            if current["mode"] == mode and (
                mode != "localization" or same_localization_map
            ):
                return ApiResult.ok(current, "SLAM 系统已经运行")
            stopped = self.stop()
            if not stopped.success:
                return stopped

        script = self.project_root / self.SCRIPT_BY_MODE[mode]
        command = ["bash", str(script)]
        if mode == "localization":
            if not map_name:
                return ApiResult.fail("没有指定定位地图", "MAP_NAME_REQUIRED")
            command.append(map_name)

        environment = os.environ.copy()
        environment["ROS_DOMAIN_ID"] = environment.get("ROS_DOMAIN_ID", "88")
        environment["RMW_IMPLEMENTATION"] = "rmw_cyclonedds_cpp"
        dds_config = self.project_root / "visual_laser_slam" / "cyclonedds_dual_3d.xml"
        if dds_config.is_file():
            environment["CYCLONEDDS_URI"] = environment.get(
                "DUAL_3D_CYCLONEDDS_URI", f"file://{dds_config}"
            )
        environment["USE_RVIZ"] = environment.get("ROBOT_UI_START_RVIZ", "false")
        environment["PYTHONUNBUFFERED"] = "1"
        if shutil.which("stdbuf"):
            command = ["stdbuf", "-oL", "-eL", *command]
        log_path = self.log_dir / "slam_launcher.log"
        with self._lock:
            log_handle = log_path.open("ab", buffering=0)
            try:
                header = (
                    f"\n{'=' * 72}\n"
                    f"{time.strftime('%Y-%m-%d %H:%M:%S')} UI START mode={mode} "
                    f"domain={environment['ROS_DOMAIN_ID']} "
                    f"rmw={environment['RMW_IMPLEMENTATION']} "
                    f"rviz={environment['USE_RVIZ']}\n"
                    f"dds={environment.get('CYCLONEDDS_URI', 'default')}\n"
                    f"cwd={self.project_root}\n"
                    f"command={' '.join(command)}\n"
                    f"{'=' * 72}\n"
                )
                log_handle.write(header.encode("utf-8", errors="replace"))
                self._child = subprocess.Popen(
                    command,
                    cwd=self.project_root,
                    env=environment,
                    stdin=subprocess.DEVNULL,
                    stdout=log_handle,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
                self._last_exit_code = None
            except OSError as exc:
                log_handle.close()
                self.log.exception("SLAM 启动失败")
                return ApiResult.fail(f"SLAM 启动失败：{exc}", "STACK_START_FAILED")
            finally:
                log_handle.close()
        if mode == "localization":
            try:
                self._atomic_write_text(self.state_dir / "selected_map", map_name)
            except OSError:
                # The launcher also publishes this state. A cache write error
                # must not report startup failure after the child is running.
                self.log.exception("无法记录当前定位地图 %s", map_name)
        self.log.info("已启动 SLAM mode=%s pid=%s log=%s", mode, self._child.pid, log_path)
        return ApiResult.ok(
            {"mode": mode, "pid": self._child.pid},
            "SLAM 系统正在启动",
        )

    def stop(self) -> ApiResult[None]:
        current = self.status()
        pid = int(current.get("pid", 0))
        if not current.get("running") or pid <= 1:
            return ApiResult.ok(message="SLAM 系统已经停止")
        try:
            os.kill(pid, signal.SIGINT)
        except OSError as exc:
            return ApiResult.fail(f"无法停止 SLAM：{exc}", "STACK_STOP_FAILED")
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline and self._pid_alive(pid):
            time.sleep(0.2)
        if self._pid_alive(pid):
            try:
                os.kill(pid, signal.SIGTERM)
            except OSError:
                pass
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and self._pid_alive(pid):
                time.sleep(0.2)
        if self._pid_alive(pid):
            return ApiResult.fail("SLAM 进程未能正常结束", "STACK_STOP_TIMEOUT")
        self.log.info("SLAM 系统已停止 pid=%s", pid)
        return ApiResult.ok(message="SLAM 系统已停止")

    @staticmethod
    def _safe_map_name(name: str) -> str:
        normalized = name.strip()
        if not normalized or not re.fullmatch(r"[\w\u4e00-\u9fff-]+", normalized):
            return ""
        return normalized

    def save_map(self, name: str) -> ApiResult[dict[str, Any]]:
        safe_name = self._safe_map_name(name)
        if not safe_name:
            return ApiResult.fail("地图名称只能包含文字、数字、下划线和短横线", "INVALID_MAP_NAME")
        if self.project_root is None:
            return ApiResult.fail("未找到汇川 SLAM 工程", "SLAM_PROJECT_NOT_FOUND")
        output_dir = self.project_root / "Loc_MAP"
        output_dir.mkdir(parents=True, exist_ok=True)
        basename = output_dir / safe_name
        try:
            map_result = subprocess.run(
                [
                    "ros2", "run", "nav2_map_server", "map_saver_cli",
                    "-f", str(basename), "--ros-args",
                    "-p", "map_subscribe_transient_local:=true",
                    "-p", "save_map_timeout:=20000",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=35,
            )
            state_result = subprocess.run(
                [
                    "ros2", "service", "call", "/write_state",
                    "cartographer_ros_msgs/srv/WriteState",
                    "{filename: '%s'}" % str(basename.with_suffix(".pbstream")),
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=35,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return ApiResult.fail(f"地图保存失败：{exc}", "MAP_SAVE_FAILED")

        required = [
            basename.with_suffix(".yaml"),
            basename.with_suffix(".pgm"),
            basename.with_suffix(".pbstream"),
        ]
        if map_result.returncode != 0 or state_result.returncode != 0 or not all(
            item.is_file() and item.stat().st_size > 0 for item in required
        ):
            detail = (map_result.stdout + "\n" + state_result.stdout)[-1200:]
            return ApiResult.fail(f"地图文件保存不完整：{detail}", "MAP_SAVE_INCOMPLETE")

        ui_map_dir = self.ui_root / "map"
        ui_map_dir.mkdir(parents=True, exist_ok=True)
        for source in required:
            shutil.copy2(source, ui_map_dir / source.name)
        return ApiResult.ok(
            {"name": safe_name, "yaml_path": str(required[0])},
            "地图、PBStream 已保存",
        )

    def prepare_localization_map(self, yaml_path: str) -> ApiResult[str]:
        if self.project_root is None:
            return ApiResult.fail("未找到汇川 SLAM 工程", "SLAM_PROJECT_NOT_FOUND")
        source_yaml = Path(yaml_path).resolve()
        source_pbstream = source_yaml.with_suffix(".pbstream")
        try:
            payload = yaml.safe_load(source_yaml.read_text(encoding="utf-8")) or {}
        except (OSError, yaml.YAMLError) as exc:
            return ApiResult.fail(f"地图 YAML 无法解析：{exc}", "MAP_YAML_INVALID")
        image_value = str(payload.get("image", "")).strip()
        source_image = Path(image_value).expanduser() if image_value else Path()
        if image_value and not source_image.is_absolute():
            source_image = (source_yaml.parent / source_image).resolve()
        fallback_image = source_yaml.with_suffix(".pgm")
        if not source_image.is_file() and fallback_image.is_file():
            source_image = fallback_image
        if not all(path.is_file() for path in (source_yaml, source_image, source_pbstream)):
            return ApiResult.fail(
                "定位地图必须包含 YAML、YAML 引用的图像和同名 PBStream",
                "LOCALIZATION_MAP_INCOMPLETE",
            )
        destination = self.project_root / "Loc_MAP"
        destination.mkdir(parents=True, exist_ok=True)
        stem = source_yaml.stem
        target_yaml = destination / f"{stem}.yaml"
        target_image = destination / f"{stem}{source_image.suffix.lower()}"
        target_pbstream = destination / f"{stem}.pbstream"
        for source, target in (
            (source_image, target_image),
            (source_pbstream, target_pbstream),
        ):
            if source != target.resolve():
                shutil.copy2(source, target)
        payload["image"] = target_image.name
        temporary_yaml = target_yaml.with_suffix(".yaml.tmp")
        temporary_yaml.write_text(
            yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
            encoding="utf-8",
        )
        os.replace(temporary_yaml, target_yaml)
        return ApiResult.ok(source_yaml.stem)
