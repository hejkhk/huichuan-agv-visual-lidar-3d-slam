import json

from backend.storage import JsonStorage


def test_json_round_trip(tmp_path):
    storage = JsonStorage(tmp_path); data = storage.read("settings.json"); data["volume"] = 37; storage.write("settings.json", data)
    assert JsonStorage(tmp_path).read("settings.json")["volume"] == 37


def test_corrupt_json_is_backed_up(tmp_path):
    storage = JsonStorage(tmp_path); (tmp_path / "settings.json").write_text("bad", encoding="utf-8")
    storage = JsonStorage(tmp_path)
    assert storage.read("settings.json")["volume"] == 68
    assert list(tmp_path.glob("settings.json.corrupt-*"))


def test_home_tutorial_startup_setting_defaults_off(tmp_path):
    storage = JsonStorage(tmp_path)
    assert storage.read("settings.json")["show_home_tutorial_on_startup"] is False
