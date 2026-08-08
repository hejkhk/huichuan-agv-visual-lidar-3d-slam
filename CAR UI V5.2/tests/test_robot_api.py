import inspect

from robot_api import create_robot_api
from robot_api.base import RobotApiBase
from robot_api.mock import MockRobotApi
from robot_api.team import TeamRobotApi


def test_team_and_mock_have_base_signatures():
    for name, member in inspect.getmembers(RobotApiBase, inspect.isfunction):
        if getattr(member, "__isabstractmethod__", False):
            assert inspect.signature(getattr(MockRobotApi, name)) == inspect.signature(member)
            assert inspect.signature(getattr(TeamRobotApi, name)) == inspect.signature(member)


def test_duplicate_names_rejected(tmp_path):
    api = MockRobotApi(tmp_path)
    assert not api.save_point("前台", 1, 2, 3).success
    assert not api.save_voiceprint("声纹3").success


def test_api_error_is_result_not_exception(tmp_path):
    api = MockRobotApi(tmp_path)
    result = api.start_single_navigation("missing")
    assert not result.success and result.error_code == "NOT_FOUND"


def test_api_mode_can_select_mock(monkeypatch, tmp_path):
    monkeypatch.setenv("ROBOT_API_MODE", "mock")
    assert isinstance(create_robot_api(tmp_path), MockRobotApi)


def test_voiceprints_are_prioritized_and_movable(tmp_path):
    api = MockRobotApi(tmp_path)
    initial = api.list_voiceprints().data
    assert [item["priority"] for item in initial] == [1, 2, 3]

    moved_id = initial[2]["id"]
    assert api.move_voiceprint(moved_id, -1).success
    reordered = api.list_voiceprints().data
    assert [item["id"] for item in reordered] == [initial[0]["id"], moved_id, initial[1]["id"]]
    assert [item["priority"] for item in reordered] == [1, 2, 3]


def test_voiceprint_limit_and_default_lowest_priority(tmp_path):
    api = MockRobotApi(tmp_path)
    while len(api.list_voiceprints().data) < 10:
        name = f"Voice {len(api.list_voiceprints().data) + 1}"
        result = api.save_voiceprint(name)
        assert result.success
        assert result.data["priority"] == len(api.list_voiceprints().data)

    assert not api.begin_voiceprint_recording("overflow").success
    full_result = api.save_voiceprint("overflow")
    assert not full_result.success
    assert full_result.error_code == "VOICEPRINT_LIMIT"


def test_voiceprint_delete_compacts_priorities(tmp_path):
    api = MockRobotApi(tmp_path)
    items = api.list_voiceprints().data
    assert api.delete_voiceprint(items[1]["id"]).success
    remaining = api.list_voiceprints().data
    assert [item["priority"] for item in remaining] == list(range(1, len(remaining) + 1))


def test_mapping_lifecycle(tmp_path):
    api = MockRobotApi(tmp_path)
    snap = api.get_robot_snapshot().data
    assert snap.mapping_state == "IDLE"
    assert not snap.mapping_active

    assert api.start_mapping().success
    snap = api.get_robot_snapshot().data
    assert snap.mapping_state == "MAPPING"
    assert snap.mapping_active

    assert api.stop_mapping().success
    snap = api.get_robot_snapshot().data
    assert snap.mapping_state == "STOPPED"
    assert not snap.mapping_active

    result = api.save_map("办公室地图")
    assert result.success
    assert result.data["name"] == "办公室地图"
    snap = api.get_robot_snapshot().data
    assert snap.mapping_state == "COMPLETED"


def test_save_map_empty_name(tmp_path):
    api = MockRobotApi(tmp_path)
    result = api.save_map("")
    assert not result.success
    assert result.error_code == "EMPTY_NAME"
    result = api.save_map("   ")
    assert not result.success
    assert result.error_code == "EMPTY_NAME"


def test_mock_map_loader_requires_yaml_and_returns_loaded_identity(tmp_path):
    api = MockRobotApi(tmp_path)
    missing = api.load_map(str(tmp_path / "missing.yaml"))
    assert not missing.success
    assert missing.error_code == "MAP_NOT_FOUND"

    yaml_path = tmp_path / "中文地图.yaml"
    yaml_path.write_text("image: 中文地图.pgm\n", encoding="utf-8")
    loaded = api.load_map(str(yaml_path))
    assert loaded.success
    assert loaded.data["name"] == "中文地图"
    assert loaded.data["yaml_path"] == str(yaml_path.resolve())
