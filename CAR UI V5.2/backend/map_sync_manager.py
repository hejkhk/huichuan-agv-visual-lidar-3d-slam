from __future__ import annotations

import hashlib
import logging
import os
import shutil
import threading
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Callable, Iterator


class MapSyncManager:
    """Maintain an atomic UI cache of map image, metadata and PBStream files.

    ``map_dir`` is authoritative.  The worker only publishes a pair after both
    source files have remained unchanged for ``settle_seconds``.  Cache writes
    use a same-directory temporary file followed by ``os.replace`` so QML
    never opens a partially copied file.
    """

    def __init__(
        self,
        map_dir: str | Path,
        cache_dir: str | Path,
        *,
        scan_interval: float = 2.0,
        settle_seconds: float = 1.2,
    ):
        self.map_dir = Path(map_dir).resolve()
        self.cache_dir = Path(cache_dir).resolve()
        self.scan_interval = max(0.25, float(scan_interval))
        self.settle_seconds = max(0.0, float(settle_seconds))
        self.log = logging.getLogger("MAP_SYNC")
        self.map_dir.mkdir(parents=True, exist_ok=True)
        self.cache_dir.mkdir(parents=True, exist_ok=True)

        self._lock = threading.RLock()
        self._sync_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._observed: dict[str, tuple[tuple[tuple[int, int], tuple[int, int]], float]] = {}
        self._missing_since: dict[str, float] = {}
        self._status = "IDLE"
        self._errors: list[dict[str, str]] = []
        self._busy_stems: set[str] = set()
        self._generation = 0
        self._last_sync_time = 0.0
        self._listeners: list[Callable[[], None]] = []

    @staticmethod
    def _visible_file(path: Path) -> bool:
        return (
            path.is_file()
            and not path.name.startswith(".")
            and path.suffix.casefold() in {".pgm", ".yaml", ".pbstream"}
        )

    def _files_by_stem(self, directory: Path) -> dict[str, dict[str, Path]]:
        result: dict[str, dict[str, Path]] = {}
        try:
            entries = list(directory.iterdir())
        except OSError as exc:
            self.log.warning("无法扫描地图目录 %s: %s", directory, exc)
            return result
        for path in entries:
            if not self._visible_file(path):
                continue
            result.setdefault(path.stem, {})[path.suffix.casefold()] = path
        return result

    @staticmethod
    def _file_signature(path: Path) -> tuple[int, int]:
        stat = path.stat()
        return stat.st_size, stat.st_mtime_ns

    def _pair_signature(
        self, files: dict[str, Path]
    ) -> tuple[tuple[int, int], tuple[int, int]]:
        return self._file_signature(files[".pgm"]), self._file_signature(files[".yaml"])

    @staticmethod
    def _digest(path: Path) -> bytes:
        digest = hashlib.sha256()
        with path.open("rb") as handle:
            for block in iter(lambda: handle.read(1024 * 1024), b""):
                digest.update(block)
        return digest.digest()

    @classmethod
    def _same_content(cls, source: Path, target: Path) -> bool:
        try:
            if source.stat().st_size != target.stat().st_size:
                return False
            return cls._digest(source) == cls._digest(target)
        except OSError:
            return False

    @staticmethod
    def _fsync_directory(directory: Path) -> None:
        try:
            descriptor = os.open(directory, os.O_RDONLY)
        except OSError:
            return
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    def _atomic_copy(self, source: Path, target: Path) -> None:
        temporary = target.with_name(f".{target.name}.tmp-{uuid.uuid4().hex}")
        try:
            with source.open("rb") as reader, temporary.open("xb") as writer:
                shutil.copyfileobj(reader, writer, length=1024 * 1024)
                writer.flush()
                os.fsync(writer.fileno())
            source_stat = source.stat()
            os.utime(
                temporary,
                ns=(source_stat.st_atime_ns, source_stat.st_mtime_ns),
            )
            os.replace(temporary, target)
            self._fsync_directory(target.parent)
        finally:
            try:
                temporary.unlink()
            except FileNotFoundError:
                pass

    def _remove_cache_pair(self, stem: str, files: dict[str, Path]) -> bool:
        token = uuid.uuid4().hex
        moved: list[tuple[Path, Path]] = []
        try:
            for suffix in (".pgm", ".yaml", ".pbstream"):
                path = files.get(suffix)
                if path is None or not path.exists():
                    continue
                staged = path.with_name(f".{path.name}.delete-{token}")
                os.replace(path, staged)
                moved.append((path, staged))
            for _original, staged in moved:
                staged.unlink(missing_ok=True)
            self._fsync_directory(self.cache_dir)
            return bool(moved)
        except OSError:
            for original, staged in reversed(moved):
                if staged.exists() and not original.exists():
                    os.replace(staged, original)
            raise

    def _set_runtime_state(
        self,
        *,
        status: str | None = None,
        errors: list[dict[str, str]] | None = None,
        busy_stems: set[str] | None = None,
        changed: bool = False,
    ) -> None:
        with self._lock:
            if status is not None:
                self._status = status
            if errors is not None:
                self._errors = errors
            if busy_stems is not None:
                self._busy_stems = set(busy_stems)
            if changed:
                self._generation += 1
            self._last_sync_time = time.time()

    def synchronize(self, *, force: bool = False) -> bool:
        """Synchronize eligible pairs now.

        Normal background scans wait for stable source metadata.  ``force`` is
        reserved for MapManager transactions whose files have already been
        completely written and fsynced.
        """

        with self._sync_lock:
            previous_state = self.snapshot()
            now = time.monotonic()
            source = self._files_by_stem(self.map_dir)
            cache = self._files_by_stem(self.cache_dir)
            errors: list[dict[str, str]] = []
            eligible: dict[str, dict[str, Path]] = {}
            busy: set[str] = set()

            source_stems = set(source)
            for stem, files in source.items():
                missing = [suffix for suffix in (".pgm", ".yaml") if suffix not in files]
                if missing:
                    busy.add(stem)
                    errors.append(
                        {
                            "name": stem,
                            "error": "缺少 " + "、".join(missing) + " 配对文件，等待外部写入完成",
                        }
                    )
                    self._observed.pop(stem, None)
                    continue
                try:
                    signature = self._pair_signature(files)
                except OSError as exc:
                    busy.add(stem)
                    errors.append({"name": stem, "error": f"读取文件状态失败：{exc}"})
                    continue

                previous = self._observed.get(stem)
                if previous is None or previous[0] != signature:
                    oldest_mtime = min(
                        files[".pgm"].stat().st_mtime,
                        files[".yaml"].stat().st_mtime,
                    )
                    stable_since = now if time.time() - oldest_mtime < self.settle_seconds else now - self.settle_seconds
                    self._observed[stem] = (signature, stable_since)
                else:
                    stable_since = previous[1]

                if force or now - stable_since >= self.settle_seconds:
                    eligible[stem] = files
                else:
                    busy.add(stem)

            for stem in list(self._observed):
                if stem not in source_stems:
                    self._observed.pop(stem, None)

            changed = False
            self._set_runtime_state(status="SYNCING", errors=errors, busy_stems=busy)
            for stem, files in eligible.items():
                try:
                    for suffix in (".pgm", ".yaml", ".pbstream"):
                        if suffix not in files:
                            if suffix == ".pbstream":
                                stale = self.cache_dir / f"{stem}{suffix}"
                                if stale.exists():
                                    stale.unlink()
                                    changed = True
                            continue
                        source_path = files[suffix]
                        cache_path = self.cache_dir / f"{stem}{suffix}"
                        if not self._same_content(source_path, cache_path):
                            self._atomic_copy(source_path, cache_path)
                            changed = True
                except OSError as exc:
                    errors.append({"name": stem, "error": f"同步失败：{exc}"})
                    busy.add(stem)
                    self.log.exception("地图 %s 同步失败", stem)

            for stem, files in cache.items():
                if stem in source:
                    self._missing_since.pop(stem, None)
                    continue
                missing_since = self._missing_since.setdefault(stem, now)
                if not force and now - missing_since < self.settle_seconds:
                    busy.add(stem)
                    continue
                try:
                    changed = self._remove_cache_pair(stem, files) or changed
                    self._missing_since.pop(stem, None)
                except OSError as exc:
                    errors.append({"name": stem, "error": f"清理缓存失败：{exc}"})
                    busy.add(stem)
                    self.log.exception("地图缓存 %s 删除失败", stem)

            status = "ERROR" if any("失败" in item["error"] for item in errors) else (
                "SYNCING" if busy else "IDLE"
            )
            self._set_runtime_state(
                status=status,
                errors=errors,
                busy_stems=busy,
                changed=changed,
            )
            current_state = self.snapshot()
            observable_changed = any(
                previous_state[key] != current_state[key]
                for key in ("status", "errors", "busy_stems", "generation")
            )
            if observable_changed:
                with self._lock:
                    listeners = list(self._listeners)
                for listener in listeners:
                    try:
                        listener()
                    except Exception:
                        self.log.exception("地图同步监听器异常")
            return not any("失败" in item["error"] for item in errors)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.synchronize()
            except Exception as exc:  # defensive boundary for the daemon
                self.log.exception("地图同步线程异常")
                self._set_runtime_state(
                    status="ERROR",
                    errors=[{"name": "", "error": f"地图同步线程异常：{exc}"}],
                )
            self._stop_event.wait(self.scan_interval)

    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._status = "SYNCING"
            self._thread = threading.Thread(
                target=self._run,
                name="map-sync-manager",
                daemon=True,
            )
            self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(max(0.0, timeout))

    def add_listener(self, listener: Callable[[], None]) -> None:
        with self._lock:
            if listener not in self._listeners:
                self._listeners.append(listener)

    @contextmanager
    def transaction(self) -> Iterator[None]:
        """Exclude the background synchronizer during a map file transaction."""

        with self._sync_lock:
            yield

    def snapshot(self) -> dict[str, object]:
        with self._lock:
            return {
                "status": self._status,
                "errors": [dict(item) for item in self._errors],
                "busy_stems": sorted(self._busy_stems),
                "generation": self._generation,
                "last_sync_time": self._last_sync_time,
            }

    def is_busy(self, stem: str | None = None) -> bool:
        with self._lock:
            if stem is not None:
                return stem in self._busy_stems
            return self._status == "SYNCING" or bool(self._busy_stems)
