from backend.app_state import AppState
from backend.storage import JsonStorage
from robot_api.mock import MockRobotApi


def test_navigation_controls(tmp_path):
    state = AppState(JsonStorage(tmp_path))
    assert state.navigation_controls("TARGET_SELECTED")["startEnabled"]
    assert state.navigation_controls("PAUSED")["pauseText"] == "resume"
    assert state.navigation_controls("NAVIGATING")["cancelEnabled"]


def test_cancel_navigation(tmp_path):
    api = MockRobotApi(tmp_path); api.start_single_navigation("p1")
    assert api.cancel_navigation().success
    assert api.get_robot_snapshot().data.navigation_state == "CANCELLED"


def test_arbitrary_pose_navigation(tmp_path):
    api = MockRobotApi(tmp_path)
    result = api.start_pose_navigation(1.25, -3.5, 0.75)
    assert result.success
    snapshot = api.get_robot_snapshot().data
    assert snapshot.navigation_state == "NAVIGATING"
    assert snapshot.navigation_target == "地图目标 (1.25, -3.50)"
