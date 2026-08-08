import math
from pathlib import Path

import pytest

from backend.ui_backend import UiBackend
from robot_api.mock import MockRobotApi


ROOT = Path(__file__).resolve().parents[1]


def _text(relative: str) -> str:
    return (ROOT / relative).read_text(encoding="utf-8")


def test_saved_point_heading_is_normalized_and_persisted(tmp_path):
    api = MockRobotApi(tmp_path)
    point = api.list_points().data[0]

    result = api.update_point_yaw(point["id"], -math.pi / 2)

    assert result.success
    updated = next(item for item in api.list_points().data if item["id"] == point["id"])
    assert updated["yaw"] == pytest.approx(3 * math.pi / 2)


def test_temporary_goal_defaults_to_vehicle_heading_and_is_editable(tmp_path):
    backend = UiBackend(MockRobotApi(tmp_path), tmp_path, tmp_path)
    try:
        backend._snapshot["current_pose"] = {"x": 1.0, "y": 2.0, "yaw": math.pi / 3}
        backend.selectMapGoal(4.0, 5.0)
        assert backend.mapGoal["yaw"] == pytest.approx(math.pi / 3)

        backend.setMapGoalYaw(-math.pi / 2)
        assert backend.mapGoal["yaw"] == pytest.approx(3 * math.pi / 2)
    finally:
        backend.shutdown()


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("navigation_state", "NAVIGATING", "导航任务进行中"),
        ("navigation_state", "PAUSED", "导航任务进行中"),
        ("mapping_active", True, "正在创建地图"),
        ("charging", True, "正在执行回充任务"),
    ],
)
def test_saved_heading_edit_has_robot_state_guard(tmp_path, field, value, message):
    backend = UiBackend(MockRobotApi(tmp_path), tmp_path, tmp_path)
    try:
        backend._snapshot.update({
            "navigation_state": "IDLE",
            "mapping_active": False,
            "mapping_state": "IDLE",
            "charging": False,
        })
        backend._snapshot[field] = value
        backend.updatePointYaw("front", math.pi)
        assert message in backend.notification
    finally:
        backend.shutdown()


def test_heading_controls_are_shared_across_saved_and_temporary_goals():
    dial = _text("qml/components/HeadingDial.qml")
    dialog = _text("qml/dialogs/HeadingDialog.qml")
    points = _text("qml/components/PointListItem.qml")
    map_view = _text("qml/components/RvizPlaceholder.qml")
    add_point = _text("qml/dialogs/AddPointDialog.qml")

    assert "Math.atan2(dy, dx)" in dial
    assert "backend.updatePointYaw" in dialog
    assert "backend.setMapGoalYaw" in dialog
    assert 'objectName: "saveHeadingButton"' in dialog
    assert "headingDegrees" in points
    assert 'objectName: "editMapGoalHeadingButton"' in map_view
    assert "goalMarker.goal?.yaw" in map_view
    assert "root.draftYaw" in add_point
    assert ' + "    Yaw: " +' not in add_point


def test_home_tutorial_explains_position_and_final_heading():
    model = _text("qml/components/HomeTutorialModel.qml")
    demo = _text("qml/components/HomeTutorialDemo.qml")

    assert "目标箭头同时表示到达位置和最终车头方向" in model
    assert "使用编辑目标朝向调整车头方向" in model
    assert "rotation: -Math.min" in demo
