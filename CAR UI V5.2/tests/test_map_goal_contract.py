from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _text(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_home_map_goal_wiring():
    home = _text("qml/pages/HomePage.qml")
    map_component = _text("qml/components/RvizPlaceholder.qml")

    assert "interactiveGoalSelection: true" in home
    assert "backend.selectMapGoal(worldX, worldY)" in home
    assert "onResetRequested: backend.clearNavigationSelection()" in home
    assert "signal mapGoalSelected(real worldX, real worldY)" in map_component
    assert "function worldX(pixel)" in map_component
    assert "function worldY(pixel)" in map_component
    assert "backend.hasMapGoal" in map_component


def test_cpu_guard_never_disables_goal_selection():
    map_component = _text("qml/components/RvizPlaceholder.qml")

    assert "mapGesturesAllowed: hostCpuPercent <= 70" in map_component
    assert "goalSelectionAllowed:" in map_component
    assert "interactiveGoalSelection && hasMap" in map_component
    assert "enabled: root.goalSelectionAllowed" in map_component
    assert "enabled: root.mapGesturesAllowed && root.hasMap" in map_component
    tap_block = map_component.split("TapHandler {", 1)[1].split(
        "DragHandler {", 1
    )[0]
    assert "mapGesturesAllowed" not in tap_block


def test_existing_map_subscription_contract_is_preserved():
    ros_client = _text("robot_api/ros2_client.py")

    assert 'ROBOT_UI_MAP_TOPIC", "/map"' in ros_client
    assert "_map_callback" in ros_client
    assert "OccupancyGrid" in ros_client


def test_saved_and_temporary_targets_are_mutually_exclusive():
    backend = _text("backend/ui_backend.py")

    select_point = backend.split("def selectPoint", 1)[1].split("@Slot", 1)[0]
    select_map_goal = backend.split("def selectMapGoal", 1)[1].split("@Slot", 1)[0]
    start_navigation = backend.split("def startSelectedNavigation", 1)[1].split("@Slot", 1)[0]

    assert "self._map_goal = {}" in select_point
    assert "self.state.clear_selection()" in select_map_goal
    assert "self.api.start_pose_navigation" in start_navigation
    assert "record_recent" not in start_navigation.split("point_id =", 1)[0]


def test_navigation_controls_have_a_dedicated_change_signal():
    backend = _text("backend/ui_backend.py")

    assert "navigationControlsChanged = Signal()" in backend
    assert 'notify=navigationControlsChanged' in backend
    for method in ("selectPoint", "selectMapGoal", "clearNavigationSelection"):
        block = backend.split(f"def {method}", 1)[1].split("@Slot", 1)[0]
        assert "self.navigationControlsChanged.emit()" in block
