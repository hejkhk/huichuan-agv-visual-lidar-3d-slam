from __future__ import annotations

import os

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from backend.ui_backend import UiBackend
from robot_api.mock import MockRobotApi


@pytest.fixture()
def backend(tmp_path):
    api = MockRobotApi(str(tmp_path))
    be = UiBackend(api, str(tmp_path), str(tmp_path))
    yield be
    be.shutdown()


def test_sample_history_populates_deque(backend):
    backend._snapshot = {
        "battery_percent": 80, "cpu_percent": 50, "memory_percent": 60,
        "cpu_temperature": 45, "battery_voltage": 24.0, "vx": 0.5,
    }
    initial_len = len(backend._history["timestamps"])
    backend._sample_history()
    assert len(backend._history["timestamps"]) == initial_len + 1
    assert len(backend._history["battery_percent"]) == initial_len + 1
    assert backend._history["battery_percent"][-1] == 80.0
    assert backend._history["cpu_percent"][-1] == 50.0


def test_sample_history_skips_empty_snapshot(backend):
    backend._snapshot = {}
    backend._sample_history()
    assert len(backend._history["timestamps"]) == 0


def test_history_timer_interval_by_performance_mode(backend):
    backend.setPerformanceMode(0)
    assert backend._history_timer.interval() == 6000
    backend.setPerformanceMode(1)
    assert backend._history_timer.interval() == 3000
    backend.setPerformanceMode(2)
    assert backend._history_timer.interval() == 3000


def test_history_property_returns_lists(backend):
    backend._snapshot = {
        "battery_percent": 80, "cpu_percent": 50, "memory_percent": 60,
        "cpu_temperature": 45, "battery_voltage": 24.0, "vx": 0.5,
    }
    backend._sample_history()
    h = backend.history
    assert isinstance(h, dict)
    assert isinstance(h["timestamps"], list)
    assert isinstance(h["battery_percent"], list)
    assert len(h["timestamps"]) > 0
