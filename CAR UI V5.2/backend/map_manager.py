from __future__ import annotations

import os
import re
import shutil
import threading
import time
import uuid
from datetime import datetime
from pathlib import Path, PurePosixPath
from typing import Any, Callable

import yaml

from robot_api.types import ApiResult, NavigationState, RobotSnapshot
from .map_sync_manager import MapSyncManager


class MapManager:
    """Map catalogue, file transactions, and robot-safety gate."""

    SAFE_NAVIGATION_STATES = {
        NavigationState.IDLE.value,
        NavigationState.TARGET_SELECTED.value,
        NavigationState.ARRIVED.value,
        NavigationState.FAILED.value,
        NavigationState.CANCELLED.value,
    }
    SAFE_MAPPING_STATES = {"", "IDLE", "STOPPED", "COMPLETED", "FAILED"}

    def __init__(
        self,
        map_dir: str | Path,
        cache_dir: str | Path,
        sync_manager: MapSyncManager,
        *,
        state_provider: Callable[[], RobotSnapshot | dict[str, Any]],
        map_loader: Callable[[str], ApiResult[Any]],
        current_map_provider: Callable[[], str] | None = None,
    ):
        self.map_dir = Path(map_dir).resolve()
        self.cache_dir = Path(cache_dir).resolve()
        self.sync_manager = sync_manager
        self.state_provider = state_provider
        self.map_loader = map_loader
        self.current_map_provider = current_map_provider
        self._lock = threading.RLock()
        self._operation = {
            "status": "IDLE",
            "message": "",
            "error_code": "",
            "map_id": "",
        }
        self._maps: list[dict[str, Any]] = []
        self._errors: list[dict[str, str]] = []
        self._current_map_id = ""
        self._catalog_generation = -1
        self.sync_manager.add_listener(self._on_sync_changed)
        self.refresh_maps(force=True)

    def _on_sync_changed(self) -> None:
        self.refresh_maps(force=True)

    @staticmethod
    def _atomic_write(path: Path, data: bytes) -> None:
        temporary = path.with_name(f".{path.name}.tmp-{uuid.uuid4().hex}")
        try:
            with temporary.open("xb") as handle:
                handle.write(data)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, path)
            try:
                directory_fd = os.open(path.parent, os.O_RDONLY)
                try:
                    os.fsync(directory_fd)
                finally:
                    os.close(directory_fd)
            except OSError:
                pass
        finally:
            temporary.unlink(missing_ok=True)

    @classmethod
    def _atomic_copy(cls, source: Path, target: Path) -> None:
        temporary = target.with_name(f".{target.name}.tmp-{uuid.uuid4().hex}")
        try:
            with source.open("rb") as reader, temporary.open("xb") as writer:
                shutil.copyfileobj(reader, writer, length=1024 * 1024)
                writer.flush()
                os.fsync(writer.fileno())
            os.replace(temporary, target)
        finally:
            temporary.unlink(missing_ok=True)

    @staticmethod
    def _validate_pgm(path: Path) -> str:
        try:
            with path.open("rb") as handle:
                data = handle.read(65536)
        except OSError as exc:
            return f"PGM 读取失败：{exc}"
        tokens: list[bytes] = []
        index = 0
        while index < len(data) and len(tokens) < 4:
            while index < len(data) and data[index] in b" \t\r\n":
                index += 1
            if index < len(data) and data[index] == ord("#"):
                newline = data.find(b"\n", index)
                index = len(data) if newline < 0 else newline + 1
                continue
            start = index
            while index < len(data) and data[index] not in b" \t\r\n#":
                index += 1
            if index > start:
                tokens.append(data[start:index])
        if len(tokens) < 4 or tokens[0] not in {b"P2", b"P5"}:
            return "PGM 文件头无效"
        try:
            width, height, maximum = map(int, tokens[1:4])
        except ValueError:
            return "PGM 尺寸或灰度范围无效"
        if width <= 0 or height <= 0 or maximum <= 0 or maximum > 65535:
            return "PGM 尺寸或灰度范围无效"
        if (
            index < len(data)
            and data[index] == ord("\r")
            and index + 1 < len(data)
            and data[index + 1] == ord("\n")
        ):
            index += 2
        elif index < len(data) and data[index] in b" \t\n":
            index += 1
        if path.stat().st_size <= index:
            return "PGM 图像数据为空"
        if tokens[0] == b"P5":
            bytes_per_pixel = 1 if maximum < 256 else 2
            expected = width * height * bytes_per_pixel
            if path.stat().st_size - index < expected:
                return "PGM 图像数据不完整"
        return ""

    @staticmethod
    def _read_yaml(path: Path, pgm_name: str) -> tuple[dict[str, Any] | None, str]:
        try:
            text = path.read_text(encoding="utf-8")
            data = yaml.safe_load(text)
        except (OSError, UnicodeError, yaml.YAMLError) as exc:
            return None, f"YAML 读取失败：{exc}"
        if not isinstance(data, dict):
            return None, "YAML 内容必须是对象"
        image = data.get("image")
        if not isinstance(image, str) or Path(image).name != pgm_name:
            return None, f"YAML image 应引用 {pgm_name}"
        for key in ("resolution", "origin", "negate", "occupied_thresh", "free_thresh"):
            if key not in data:
                return None, f"YAML 缺少字段 {key}"
        return data, ""

    def _scan_catalog(self) -> tuple[list[dict[str, Any]], list[dict[str, str]]]:
        files: dict[str, dict[str, Path]] = {}
        errors: list[dict[str, str]] = []
        try:
            entries = list(self.cache_dir.iterdir())
        except OSError as exc:
            return [], [{"name": "", "error": f"缓存目录读取失败：{exc}"}]
        for path in entries:
            if (
                not path.is_file()
                or path.name.startswith(".")
                or path.suffix.casefold() not in {".pgm", ".yaml", ".pbstream"}
            ):
                continue
            files.setdefault(path.stem, {})[path.suffix.casefold()] = path

        maps: list[dict[str, Any]] = []
        for stem, pair in files.items():
            pgm = pair.get(".pgm")
            yaml_path = pair.get(".yaml")
            if pgm is None or yaml_path is None:
                errors.append({"name": stem, "error": "缓存地图缺少配对的 PGM 或 YAML"})
                continue
            pgm_error = self._validate_pgm(pgm)
            _yaml_data, yaml_error = self._read_yaml(yaml_path, pgm.name)
            error = pgm_error or yaml_error
            if error:
                errors.append({"name": stem, "error": error})
                continue
            source_pgm = self.map_dir / pgm.name
            source_yaml = self.map_dir / yaml_path.name
            source_pbstream = self.map_dir / f"{stem}.pbstream"
            if not source_pgm.is_file() or not source_yaml.is_file():
                errors.append({"name": stem, "error": "主地图目录缺少对应文件"})
                continue
            source_stat = source_pgm.stat()
            # Prefer a real birth time where the filesystem exposes it.
            # Linux fallback uses the PGM modification time, which copy2
            # preserves and is more meaningful than inode-change time.
            created_timestamp = float(
                getattr(source_stat, "st_birthtime", source_stat.st_mtime)
            )
            maps.append(
                {
                    "id": stem,
                    "name": stem,
                    "pgm_path": str(source_pgm),
                    "yaml_path": str(source_yaml),
                    "pbstream_path": str(source_pbstream),
                    "localization_ready": source_pbstream.is_file(),
                    "cache_pgm_path": str(pgm),
                    "cache_yaml_path": str(yaml_path),
                    "cache_pgm_url": pgm.as_uri(),
                    "is_current": stem == self._current_map_id,
                    "modified_time": max(pgm.stat().st_mtime, yaml_path.stat().st_mtime),
                    "created_time": created_timestamp,
                    "created_time_text": datetime.fromtimestamp(
                        created_timestamp
                    ).strftime("%Y-%m-%d %H:%M"),
                    "is_complete": True,
                    "error_message": "",
                }
            )
        maps.sort(key=lambda item: (str(item["name"]).casefold(), str(item["name"])))
        sync_errors = self.sync_manager.snapshot().get("errors", [])
        errors.extend(dict(item) for item in sync_errors if isinstance(item, dict))
        return maps, errors

    def refresh_maps(self, *, force: bool = False) -> list[dict[str, Any]]:
        sync_generation = int(self.sync_manager.snapshot()["generation"])
        runtime_current = ""
        if self.current_map_provider is not None:
            try:
                runtime_current = str(self.current_map_provider() or "").strip()
            except Exception:
                runtime_current = ""
        with self._lock:
            current_changed = bool(
                runtime_current and runtime_current != self._current_map_id
            )
            if current_changed:
                self._current_map_id = runtime_current
            if (
                not force
                and not current_changed
                and sync_generation == self._catalog_generation
            ):
                return [dict(item) for item in self._maps]
            maps, errors = self._scan_catalog()
            self._maps = maps
            self._errors = errors
            self._catalog_generation = sync_generation
            if self._current_map_id and not any(
                item["id"] == self._current_map_id for item in maps
            ):
                self._current_map_id = ""
            return [dict(item) for item in maps]

    def get_map_list(self) -> list[dict[str, Any]]:
        with self._lock:
            return [dict(item) for item in self._maps]

    def get_current_map(self) -> dict[str, Any]:
        maps = self.get_map_list()
        return next((item for item in maps if item["is_current"]), {})

    def get_errors(self) -> list[dict[str, str]]:
        with self._lock:
            return [dict(item) for item in self._errors]

    def get_map_operation_state(self) -> dict[str, str]:
        with self._lock:
            state = dict(self._operation)
        sync = self.sync_manager.snapshot()
        if state["status"] == "IDLE" and sync["status"] in {"SYNCING", "ERROR"}:
            state["status"] = str(sync["status"])
            if sync["status"] == "SYNCING":
                state["message"] = "正在同步地图"
            elif sync["errors"]:
                state["message"] = str(sync["errors"][0].get("error", "地图同步失败"))
        return state

    def _set_operation(
        self, status: str, *, message: str = "", error_code: str = "", map_id: str = ""
    ) -> None:
        with self._lock:
            self._operation = {
                "status": status,
                "message": message,
                "error_code": error_code,
                "map_id": map_id,
            }

    def _claim_operation(self, status: str, message: str, map_id: str) -> bool:
        with self._lock:
            if self._operation["status"] != "IDLE":
                return False
            self._operation = {
                "status": status,
                "message": message,
                "error_code": "",
                "map_id": map_id,
            }
            return True

    def _snapshot_value(self, name: str, default: Any = None) -> Any:
        snapshot = self.state_provider()
        if isinstance(snapshot, dict):
            return snapshot.get(name, default)
        return getattr(snapshot, name, default)

    def _robot_block_reason(self) -> str:
        navigation = str(
            self._snapshot_value("navigation_state", NavigationState.IDLE.value)
        ).upper()
        if navigation not in self.SAFE_NAVIGATION_STATES:
            reasons = {
                "STARTING": "正在启动导航，无法操作地图",
                "NAVIGATING": "当前正在导航，无法操作地图",
                "PAUSED": "导航任务已暂停但尚未结束，无法操作地图",
            }
            return reasons.get(navigation, f"机器人任务状态为 {navigation}，无法操作地图")
        mapping_state = str(self._snapshot_value("mapping_state", "IDLE")).upper()
        if bool(self._snapshot_value("mapping_active", False)) or mapping_state not in self.SAFE_MAPPING_STATES:
            return "当前正在建图，无法操作地图"
        if bool(self._snapshot_value("charging", False)):
            return "当前正在执行回充或充电任务，无法操作地图"
        follow_state = str(self._snapshot_value("follow_state", "IDLE")).upper()
        if follow_state == "FOLLOWING":
            return "当前正在视觉跟随，无法操作地图"
        system_status = str(self._snapshot_value("system_status", "")).upper()
        if "ESTOP" in system_status or "E-STOP" in system_status or "急停" in system_status:
            return "机器人处于急停状态，无法操作地图"
        return ""

    def action_availability(self, map_id: str, action: str) -> dict[str, Any]:
        maps = self.get_map_list()
        target = next((item for item in maps if item["id"] == map_id), None)
        if target is None:
            return {"allowed": False, "reason": "地图不存在"}
        operation = self.get_map_operation_state()
        if operation["status"] != "IDLE":
            return {"allowed": False, "reason": operation["message"] or "地图操作正在进行"}
        if self.sync_manager.is_busy(map_id):
            return {"allowed": False, "reason": "地图文件仍在同步或写入"}
        if action == "use" and not target.get("localization_ready", False):
            return {
                "allowed": False,
                "reason": "该地图缺少同名 PBStream，无法进行重定位",
            }
        robot_reason = self._robot_block_reason()
        if robot_reason:
            return {"allowed": False, "reason": robot_reason}
        if action in {"rename", "delete"} and target["is_current"]:
            return {"allowed": False, "reason": "当前正在使用的地图不能修改，请先切换地图"}
        if action == "delete" and len(maps) <= 1:
            return {"allowed": False, "reason": "至少需要保留一张有效地图"}
        if action == "use" and target["is_current"]:
            stack_running = bool(self._snapshot_value("slam_running", False))
            stack_mode = str(self._snapshot_value("slam_mode", "")).lower()
            if stack_running and stack_mode == "localization":
                return {"allowed": False, "reason": "该地图已经在使用"}
        return {"allowed": True, "reason": ""}

    @staticmethod
    def validate_name(name: str, existing_names: set[str]) -> ApiResult[str]:
        normalized = name.strip()
        if not normalized:
            return ApiResult.fail("地图名称不能为空", "EMPTY_NAME")
        if normalized.casefold().endswith((".pgm", ".yaml")):
            return ApiResult.fail("地图名称不要包含 .pgm 或 .yaml 扩展名", "INVALID_NAME")
        if "\x00" in normalized or "/" in normalized or "\\" in normalized or ".." in normalized:
            return ApiResult.fail("地图名称包含危险路径字符", "INVALID_NAME")
        if normalized.casefold() in {item.casefold() for item in existing_names}:
            return ApiResult.fail("地图名称已存在", "DUPLICATE_NAME")
        return ApiResult.ok(normalized)

    @staticmethod
    def _updated_yaml_text(path: Path, old_pgm: str, new_pgm: str) -> str:
        text = path.read_text(encoding="utf-8")
        parsed = yaml.safe_load(text)
        if not isinstance(parsed, dict) or not isinstance(parsed.get("image"), str):
            raise ValueError("YAML image 字段无效")
        old_reference = str(parsed["image"])
        if Path(old_reference).name != old_pgm:
            raise ValueError(f"YAML image 应引用 {old_pgm}")
        reference_path = PurePosixPath(old_reference)
        new_reference = str(reference_path.parent / new_pgm)
        if str(reference_path.parent) == ".":
            new_reference = new_pgm

        pattern = re.compile(r"^([ \t]*image[ \t]*:[ \t]*)(.*?)([ \t]*(?:#.*)?)$", re.MULTILINE)
        match = pattern.search(text)
        if match is None:
            raise ValueError("YAML 中未找到 image 字段")
        value = match.group(2).strip()
        quote = value[0] if len(value) >= 2 and value[0] in {"'", '"'} and value[-1] == value[0] else ""
        replacement_value = f"{quote}{new_reference}{quote}" if quote else new_reference
        return text[: match.start()] + match.group(1) + replacement_value + match.group(3) + text[match.end() :]

    def select_map(self, map_id: str) -> ApiResult[dict[str, Any]]:
        availability = self.action_availability(map_id, "use")
        if not availability["allowed"]:
            return ApiResult.fail(availability["reason"], "MAP_OPERATION_BLOCKED")
        target = next(item for item in self.refresh_maps() if item["id"] == map_id)
        if not self._claim_operation("LOADING_MAP", "正在加载地图", map_id):
            return ApiResult.fail("另一个地图操作正在进行", "MAP_OPERATION_BUSY")
        try:
            result = self.map_loader(str(target["yaml_path"]))
            if not isinstance(result, ApiResult):
                return ApiResult.fail("地图加载接口返回格式无效", "INVALID_API_RESULT")
            if not result.success:
                return ApiResult.fail(result.message, result.error_code)
            with self._lock:
                self._current_map_id = map_id
                self._catalog_generation = -1
            selected = next(
                item for item in self.refresh_maps(force=True) if item["id"] == map_id
            )
            return ApiResult.ok(selected, result.message or "地图加载成功")
        finally:
            self._set_operation("IDLE")

    def rename_map(self, map_id: str, new_name: str) -> ApiResult[dict[str, Any]]:
        availability = self.action_availability(map_id, "rename")
        if not availability["allowed"]:
            return ApiResult.fail(availability["reason"], "MAP_OPERATION_BLOCKED")
        maps = self.refresh_maps()
        target = next(item for item in maps if item["id"] == map_id)
        validation = self.validate_name(new_name, {str(item["name"]) for item in maps})
        if not validation.success or validation.data is None:
            return ApiResult.fail(validation.message, validation.error_code)
        normalized = validation.data
        if normalized == map_id:
            return ApiResult.ok(target, "地图名称未变化")

        old_pgm = Path(target["pgm_path"])
        old_yaml = Path(target["yaml_path"])
        old_pbstream = Path(target["pbstream_path"])
        new_pgm = self.map_dir / f"{normalized}.pgm"
        new_yaml = self.map_dir / f"{normalized}.yaml"
        new_pbstream = self.map_dir / f"{normalized}.pbstream"
        token = uuid.uuid4().hex
        backups: list[tuple[Path, Path]] = []
        created: list[Path] = []
        if not self._claim_operation("RENAMING", "正在重命名地图", map_id):
            return ApiResult.fail("另一个地图操作正在进行", "MAP_OPERATION_BUSY")
        try:
            yaml_text = self._updated_yaml_text(
                old_yaml, old_pgm.name, new_pgm.name
            )
            with self.sync_manager.transaction():
                if new_pgm.exists() or new_yaml.exists() or new_pbstream.exists():
                    return ApiResult.fail("地图名称已存在", "DUPLICATE_NAME")
                self._atomic_copy(old_pgm, new_pgm)
                created.append(new_pgm)
                self._atomic_write(new_yaml, yaml_text.encode("utf-8"))
                created.append(new_yaml)
                if old_pbstream.is_file():
                    self._atomic_copy(old_pbstream, new_pbstream)
                    created.append(new_pbstream)
                for original in (old_pgm, old_yaml, old_pbstream):
                    if not original.exists():
                        continue
                    backup = original.with_name(f".{original.name}.rename-{token}")
                    os.replace(original, backup)
                    backups.append((original, backup))
                if not self.sync_manager.synchronize(force=True):
                    raise OSError("缓存同步失败")
                for _original, backup in backups:
                    backup.unlink(missing_ok=True)
            with self._lock:
                self._catalog_generation = -1
            renamed = next(
                item
                for item in self.refresh_maps(force=True)
                if item["id"] == normalized
            )
            return ApiResult.ok(renamed, "地图已重命名")
        except (OSError, UnicodeError, ValueError, yaml.YAMLError) as exc:
            for original, backup in reversed(backups):
                if backup.exists() and not original.exists():
                    os.replace(backup, original)
            for created_path in created:
                created_path.unlink(missing_ok=True)
            try:
                self.sync_manager.synchronize(force=True)
            except Exception:
                pass
            return ApiResult.fail(f"地图重命名失败：{exc}", "RENAME_FAILED")
        finally:
            self._set_operation("IDLE")

    def delete_map(self, map_id: str) -> ApiResult[None]:
        availability = self.action_availability(map_id, "delete")
        if not availability["allowed"]:
            return ApiResult.fail(availability["reason"], "MAP_OPERATION_BLOCKED")
        target = next(item for item in self.refresh_maps() if item["id"] == map_id)
        originals = [Path(target["pgm_path"]), Path(target["yaml_path"])]
        pbstream = Path(target["pbstream_path"])
        if pbstream.is_file():
            originals.append(pbstream)
        token = uuid.uuid4().hex
        backups: list[tuple[Path, Path]] = []
        if not self._claim_operation("DELETING", "正在删除地图", map_id):
            return ApiResult.fail("另一个地图操作正在进行", "MAP_OPERATION_BUSY")
        try:
            with self.sync_manager.transaction():
                for original in originals:
                    backup = original.with_name(f".{original.name}.delete-{token}")
                    os.replace(original, backup)
                    backups.append((original, backup))
                if not self.sync_manager.synchronize(force=True):
                    raise OSError("缓存同步失败")
                for _original, backup in backups:
                    backup.unlink(missing_ok=True)
            with self._lock:
                self._catalog_generation = -1
            self.refresh_maps(force=True)
            return ApiResult.ok(message="地图已删除")
        except OSError as exc:
            for original, backup in reversed(backups):
                if backup.exists() and not original.exists():
                    os.replace(backup, original)
            try:
                self.sync_manager.synchronize(force=True)
            except Exception:
                pass
            return ApiResult.fail(f"地图删除失败：{exc}", "DELETE_FAILED")
        finally:
            self._set_operation("IDLE")
