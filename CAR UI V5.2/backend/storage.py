from __future__ import annotations

import json
import os
import shutil
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from threading import RLock
from typing import Any


DEFAULT_POINTS = [
    {"id": "p1", "name": "前台", "x": 6.46, "y": 65.64, "yaw": 0.0, "is_charging_point": False, "created_at": "2026-01-01T00:00:00", "last_navigated_at": None},
    {"id": "p2", "name": "会议室", "x": 12.4, "y": 21.2, "yaw": 1.57, "is_charging_point": False, "created_at": "2026-01-01T00:00:00", "last_navigated_at": None},
    {"id": "p3", "name": "展厅", "x": -3.1, "y": 8.8, "yaw": 3.14, "is_charging_point": False, "created_at": "2026-01-01T00:00:00", "last_navigated_at": None},
    {"id": "charge", "name": "充电站", "x": 0.0, "y": 0.0, "yaw": 0.0, "is_charging_point": True, "created_at": "2026-01-01T00:00:00", "last_navigated_at": None},
]
DEFAULT_VOICEPRINTS = [
    {"id": "v1", "name": "声纹3", "priority": 1},
    {"id": "v2", "name": "Voice 787", "priority": 2},
    {"id": "v3", "name": "Voice 10086", "priority": 3},
]
DEFAULT_SETTINGS = {"recent_point_ids": ["p3", "p2", "p1"], "volume": 68, "language": "zh", "performance_mode": 1, "ros_domain_id": 88, "show_home_tutorial_on_startup": False, "voice_control_enabled": True, "visual_follow_enabled": True, "unknown_voice_allowed": True, "parameters": {"max_speed": 0.5, "follow_distance": 1.0}}


class JsonStorage:
    """Thread-safe JSON persistence with corrupt-file recovery and atomic writes."""

    def __init__(self, data_dir: str | Path):
        self.data_dir = Path(data_dir)
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self._lock = RLock()
        self._defaults = {"mock_points.json": DEFAULT_POINTS, "mock_voiceprints.json": DEFAULT_VOICEPRINTS, "settings.json": DEFAULT_SETTINGS}
        for filename, default in self._defaults.items():
            self._ensure(filename, default)

    def _ensure(self, filename: str, default: Any) -> None:
        path = self.data_dir / filename
        if not path.exists():
            self.write(filename, default)
            return
        try:
            self.read(filename)
        except (json.JSONDecodeError, OSError, ValueError):
            backup = path.with_suffix(path.suffix + f".corrupt-{datetime.now().strftime('%Y%m%d%H%M%S')}")
            shutil.copy2(path, backup)
            self.write(filename, default)

    def read(self, filename: str) -> Any:
        with self._lock:
            with (self.data_dir / filename).open("r", encoding="utf-8") as handle:
                return json.load(handle)

    def write(self, filename: str, value: Any) -> None:
        with self._lock:
            target = self.data_dir / filename
            temp = target.with_suffix(target.suffix + ".tmp")
            with temp.open("w", encoding="utf-8") as handle:
                json.dump(value, handle, ensure_ascii=False, indent=2)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temp, target)

    def reset(self) -> None:
        for filename, default in self._defaults.items():
            self.write(filename, deepcopy(default))
