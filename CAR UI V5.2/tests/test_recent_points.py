from backend.app_state import AppState
from backend.storage import JsonStorage


def test_recent_points_keep_three_and_deduplicate(tmp_path):
    state = AppState(JsonStorage(tmp_path))
    assert state.record_recent(["p1", "p2", "p1", "p3", "charge"]) == ["charge", "p3", "p1"]


def test_select_does_not_update_history(tmp_path):
    state = AppState(JsonStorage(tmp_path))
    before = state.storage.read("settings.json")["recent_point_ids"]
    state.select_point("charge")
    assert state.storage.read("settings.json")["recent_point_ids"] == before


def test_start_record_updates_history(tmp_path):
    state = AppState(JsonStorage(tmp_path)); state.select_point("charge")
    assert state.record_recent([state.selected_point_id])[0] == "charge"
