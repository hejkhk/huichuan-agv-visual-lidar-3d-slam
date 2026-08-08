from __future__ import annotations

import os
import tempfile

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


def test_alert_battery_warning(backend):
    old = {"battery_percent": 25, "cpu_percent": 50, "memory_percent": 50,
           "cpu_temperature": 50, "navigation_state": "IDLE", "charging": False,
           "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
           "mapping_state": "", "follow_state": ""}
    new = dict(old)
    new["battery_percent"] = 15
    backend._detect_alerts(old, new)
    assert len(backend._alerts) == 1
    assert backend._alerts[0]["level"] == "WARNING"
    assert backend._alerts[0]["message_key"] == "电量偏低"


def test_alert_battery_error(backend):
    old = {"battery_percent": 15, "cpu_percent": 50, "memory_percent": 50,
           "cpu_temperature": 50, "navigation_state": "IDLE", "charging": False,
           "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
           "mapping_state": "", "follow_state": ""}
    new = dict(old)
    new["battery_percent"] = 5
    backend._detect_alerts(old, new)
    assert len(backend._alerts) == 1
    assert backend._alerts[0]["level"] == "ERROR"
    assert backend._alerts[0]["message_key"] == "电量严重不足"


def test_alert_cpu_warning(backend):
    old = {"battery_percent": 50, "cpu_percent": 80, "memory_percent": 50,
           "cpu_temperature": 50, "navigation_state": "IDLE", "charging": False,
           "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
           "mapping_state": "", "follow_state": ""}
    new = dict(old)
    new["cpu_percent"] = 90
    backend._detect_alerts(old, new)
    assert len(backend._alerts) == 1
    assert backend._alerts[0]["level"] == "WARNING"
    assert backend._alerts[0]["message_key"] == "CPU 占用率过高"


def test_alert_temperature_levels(backend):
    base = {"battery_percent": 50, "cpu_percent": 50, "memory_percent": 50,
            "cpu_temperature": 70, "navigation_state": "IDLE", "charging": False,
            "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
            "mapping_state": "", "follow_state": ""}
    old = dict(base)
    new = dict(base)
    new["cpu_temperature"] = 80
    backend._detect_alerts(old, new)
    assert backend._alerts[0]["level"] == "WARNING"
    assert backend._alerts[0]["message_key"] == "核心温度偏高"

    old2 = dict(base)
    old2["cpu_temperature"] = 80
    new2 = dict(base)
    new2["cpu_temperature"] = 90
    backend._detect_alerts(old2, new2)
    assert backend._alerts[0]["level"] == "ERROR"
    assert backend._alerts[0]["message_key"] == "核心温度过高"


def test_alert_navigation_failed(backend):
    old = {"battery_percent": 50, "cpu_percent": 50, "memory_percent": 50,
           "cpu_temperature": 50, "navigation_state": "NAVIGATING", "charging": False,
           "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
           "mapping_state": "", "follow_state": ""}
    new = dict(old)
    new["navigation_state"] = "FAILED"
    backend._detect_alerts(old, new)
    assert len(backend._alerts) == 1
    assert backend._alerts[0]["level"] == "ERROR"
    assert backend._alerts[0]["message_key"] == "导航失败"


def test_alert_no_duplicate(backend):
    base = {"battery_percent": 15, "cpu_percent": 50, "memory_percent": 50,
            "cpu_temperature": 50, "navigation_state": "IDLE", "charging": False,
            "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
            "mapping_state": "", "follow_state": ""}
    backend._detect_alerts(base, base)
    assert len(backend._alerts) == 0


def test_alert_deque_cap(backend):
    base = {"battery_percent": 50, "cpu_percent": 50, "memory_percent": 50,
            "cpu_temperature": 50, "navigation_state": "IDLE", "charging": False,
            "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
            "mapping_state": "", "follow_state": ""}
    for i in range(600):
        old = dict(base)
        new = dict(base)
        new["navigation_state"] = f"STATE_{i}"
        old["navigation_state"] = f"OLD_{i}"
        backend._detect_alerts(old, new)
    assert len(backend._alerts) <= 500


def test_clear_alerts(backend):
    base = {"battery_percent": 50, "cpu_percent": 50, "memory_percent": 50,
            "cpu_temperature": 50, "navigation_state": "IDLE", "charging": False,
            "ros_connected": True, "lidar_status": "NORMAL", "encoder_status": "NORMAL",
            "mapping_state": "", "follow_state": ""}
    old = dict(base)
    new = dict(base)
    new["battery_percent"] = 15
    backend._detect_alerts(old, new)
    assert len(backend._alerts) > 0
    backend.clearAlerts()
    assert len(backend._alerts) == 0
