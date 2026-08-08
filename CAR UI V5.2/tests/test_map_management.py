from __future__ import annotations

import time
import threading
from pathlib import Path

import yaml

from backend.map_manager import MapManager
from backend.map_sync_manager import MapSyncManager
from robot_api.types import ApiResult, RobotSnapshot


def write_map_pair(
    directory: Path,
    name: str,
    *,
    image: str | None = None,
    marker: int = 0,
) -> None:
    directory.mkdir(parents=True, exist_ok=True)
    pixels = bytes([marker % 256, 100, 200, 255])
    (directory / f"{name}.pgm").write_bytes(b"P5\n2 2\n255\n" + pixels)
    content = {
        "image": image or f"{name}.pgm",
        "resolution": 0.05,
        "origin": [0.0, 0.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
    }
    (directory / f"{name}.yaml").write_text(
        yaml.safe_dump(content, allow_unicode=True, sort_keys=False),
        encoding="utf-8",
    )


def make_manager(tmp_path: Path, snapshot: RobotSnapshot | None = None):
    source = tmp_path / "map"
    cache = tmp_path / "map_cache"
    sync = MapSyncManager(source, cache, scan_interval=0.05, settle_seconds=0)
    state = snapshot or RobotSnapshot()
    load_calls: list[str] = []

    def loader(path: str):
        load_calls.append(path)
        return ApiResult.ok(message="地图加载成功")

    manager = MapManager(
        source,
        cache,
        sync,
        state_provider=lambda: state,
        map_loader=loader,
    )
    return source, cache, sync, manager, state, load_calls


def test_startup_sync_copies_complete_pair_atomically(tmp_path):
    source, cache, sync, _manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "warehouse")

    assert sync.synchronize(force=True)
    assert (cache / "warehouse.pgm").read_bytes() == (
        source / "warehouse.pgm"
    ).read_bytes()
    assert (cache / "warehouse.yaml").read_text(encoding="utf-8") == (
        source / "warehouse.yaml"
    ).read_text(encoding="utf-8")
    assert not list(cache.glob("*.tmp*"))


def test_incomplete_external_write_waits_for_pair(tmp_path):
    source, cache, sync, _manager, _state, _calls = make_manager(tmp_path)
    source.mkdir(parents=True, exist_ok=True)
    (source / "later.pgm").write_bytes(b"P5\n1 1\n255\n\x00")

    sync.synchronize()
    assert not (cache / "later.pgm").exists()
    assert sync.is_busy("later")

    write_map_pair(source, "later")
    assert sync.synchronize(force=True)
    assert (cache / "later.pgm").exists()
    assert (cache / "later.yaml").exists()


def test_external_update_and_delete_are_reflected_in_cache(tmp_path):
    source, cache, sync, _manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "office", marker=1)
    sync.synchronize(force=True)
    original = (cache / "office.pgm").read_bytes()

    write_map_pair(source, "office", marker=77)
    sync.synchronize(force=True)
    assert (cache / "office.pgm").read_bytes() != original

    (source / "office.pgm").unlink()
    (source / "office.yaml").unlink()
    sync.synchronize(force=True)
    assert not (cache / "office.pgm").exists()
    assert not (cache / "office.yaml").exists()


def test_catalog_ignores_corrupt_incomplete_and_wrong_image_maps(tmp_path):
    source, _cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "good")
    write_map_pair(source, "wrong", image="other.pgm")
    (source / "broken.pgm").write_bytes(b"not-pgm")
    (source / "broken.yaml").write_text(
        "image: broken.pgm\nresolution: 0.05\norigin: [0,0,0]\n"
        "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.2\n",
        encoding="utf-8",
    )
    (source / "partial.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    sync.synchronize(force=True)

    catalog = manager.refresh_maps(force=True)
    assert [item["id"] for item in catalog] == ["good"]
    assert catalog[0]["created_time"] > 0
    assert len(catalog[0]["created_time_text"]) == 16
    errors = {item["name"]: item["error"] for item in manager.get_errors()}
    assert "wrong" in errors and "image" in errors["wrong"]
    assert "broken" in errors and "PGM" in errors["broken"]
    assert "partial" in errors


def test_rename_updates_both_directories_and_only_yaml_image(tmp_path):
    source, cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "warehouse")
    yaml_path = source / "warehouse.yaml"
    text = yaml_path.read_text(encoding="utf-8")
    yaml_path.write_text("# keep this comment\n" + text, encoding="utf-8")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)

    result = manager.rename_map("warehouse", "办公室 地图")

    assert result.success, result.message
    for directory in (source, cache):
        assert (directory / "办公室 地图.pgm").exists()
        assert (directory / "办公室 地图.yaml").exists()
        assert not (directory / "warehouse.pgm").exists()
        assert not (directory / "warehouse.yaml").exists()
    renamed_text = (source / "办公室 地图.yaml").read_text(encoding="utf-8")
    assert "# keep this comment" in renamed_text
    renamed = yaml.safe_load(renamed_text)
    assert renamed["image"] == "办公室 地图.pgm"
    assert renamed["resolution"] == 0.05
    assert renamed["origin"] == [0.0, 0.0, 0.0]


def test_rename_rejects_dangerous_duplicate_and_extension_names(tmp_path):
    source, _cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "one")
    write_map_pair(source, "two")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)

    for name in ("", "../escape", "bad/name", "bad\\name", "x.pgm", "two"):
        result = manager.rename_map("one", name)
        assert not result.success, name
    assert (source / "one.pgm").exists()
    assert (source / "one.yaml").exists()


def test_select_calls_loader_with_main_yaml_and_marks_current_only_on_success(tmp_path):
    source, _cache, sync, manager, _state, calls = make_manager(tmp_path)
    write_map_pair(source, "one")
    write_map_pair(source, "two")
    (source / "one.pbstream").write_bytes(b"state-one")
    (source / "two.pbstream").write_bytes(b"state-two")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)

    result = manager.select_map("one")
    assert result.success
    assert calls == [str(source / "one.yaml")]
    assert manager.get_current_map()["id"] == "one"

    manager.map_loader = lambda _path: ApiResult.fail(
        "真实地图加载接口未接入", "NOT_IMPLEMENTED"
    )
    failed = manager.select_map("two")
    assert not failed.success
    assert manager.get_current_map()["id"] == "one"


def test_persisted_current_map_can_restart_when_stack_is_stopped(tmp_path):
    source = tmp_path / "map"
    cache = tmp_path / "map_cache"
    sync = MapSyncManager(source, cache, settle_seconds=0)
    state = RobotSnapshot()
    calls: list[str] = []

    write_map_pair(source, "office")
    (source / "office.pbstream").write_bytes(b"state")
    sync.synchronize(force=True)
    manager = MapManager(
        source,
        cache,
        sync,
        state_provider=lambda: state,
        map_loader=lambda path: calls.append(path) or ApiResult.ok(),
        current_map_provider=lambda: "office",
    )

    current = manager.refresh_maps(force=True)[0]
    assert current["is_current"]
    assert manager.action_availability("office", "use")["allowed"]
    assert manager.select_map("office").success
    assert calls == [str(source / "office.yaml")]

    state.slam_running = True
    state.slam_mode = "localization"
    assert not manager.action_availability("office", "use")["allowed"]


def test_current_map_and_last_map_cannot_be_deleted(tmp_path):
    source, _cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "one")
    write_map_pair(source, "two")
    (source / "one.pbstream").write_bytes(b"state-one")
    (source / "two.pbstream").write_bytes(b"state-two")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)
    assert manager.select_map("one").success

    assert not manager.delete_map("one").success
    assert manager.delete_map("two").success
    assert not manager.delete_map("one").success
    assert (source / "one.pgm").exists()


def test_delete_keeps_main_and_cache_consistent(tmp_path):
    source, cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "one")
    write_map_pair(source, "two")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)

    result = manager.delete_map("two")
    assert result.success, result.message
    for directory in (source, cache):
        assert not (directory / "two.pgm").exists()
        assert not (directory / "two.yaml").exists()
    assert [item["id"] for item in manager.get_map_list()] == ["one"]


def test_active_robot_states_block_all_mutations(tmp_path):
    source, _cache, sync, manager, state, _calls = make_manager(tmp_path)
    write_map_pair(source, "one")
    write_map_pair(source, "two")
    (source / "one.pbstream").write_bytes(b"state-one")
    (source / "two.pbstream").write_bytes(b"state-two")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)

    blockers = [
        ("navigation_state", "NAVIGATING"),
        ("navigation_state", "PAUSED"),
        ("mapping_state", "MAPPING"),
        ("charging", True),
        ("follow_state", "FOLLOWING"),
        ("system_status", "ESTOP"),
    ]
    for field, value in blockers:
        state.navigation_state = "IDLE"
        state.mapping_state = "IDLE"
        state.mapping_active = False
        state.charging = False
        state.follow_state = "IDLE"
        state.system_status = "正常"
        setattr(state, field, value)
        assert not manager.select_map("one").success, (field, value)
        assert not manager.rename_map("one", "renamed").success, (field, value)
        assert not manager.delete_map("one").success, (field, value)


def test_background_worker_stops_cleanly(tmp_path):
    source, cache, sync, _manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "worker")
    sync.start()
    deadline = time.monotonic() + 1
    while not (cache / "worker.pgm").exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    sync.stop()
    assert sync._thread is not None
    assert not sync._thread.is_alive()


def test_rename_failure_rolls_back_main_pair(tmp_path, monkeypatch):
    source, cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "old")
    write_map_pair(source, "other")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)

    monkeypatch.setattr(
        manager,
        "_atomic_write",
        lambda *_args, **_kwargs: (_ for _ in ()).throw(OSError("disk full")),
    )
    result = manager.rename_map("old", "new")

    assert not result.success
    for directory in (source, cache):
        assert (directory / "old.pgm").exists()
        assert (directory / "old.yaml").exists()
        assert not (directory / "new.pgm").exists()
        assert not (directory / "new.yaml").exists()


def test_delete_sync_failure_restores_main_pair(tmp_path, monkeypatch):
    source, _cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "one")
    write_map_pair(source, "two")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)

    monkeypatch.setattr(sync, "synchronize", lambda **_kwargs: False)
    result = manager.delete_map("two")

    assert not result.success
    assert (source / "two.pgm").exists()
    assert (source / "two.yaml").exists()


def test_loading_state_rejects_concurrent_map_switch(tmp_path):
    source, _cache, sync, manager, _state, _calls = make_manager(tmp_path)
    write_map_pair(source, "one")
    write_map_pair(source, "two")
    (source / "one.pbstream").write_bytes(b"state-one")
    (source / "two.pbstream").write_bytes(b"state-two")
    sync.synchronize(force=True)
    manager.refresh_maps(force=True)
    entered = threading.Event()
    release = threading.Event()

    def slow_loader(_path):
        entered.set()
        release.wait(1)
        return ApiResult.ok(message="地图加载成功")

    manager.map_loader = slow_loader
    result_holder = []
    thread = threading.Thread(
        target=lambda: result_holder.append(manager.select_map("one"))
    )
    thread.start()
    assert entered.wait(1)
    blocked = manager.select_map("two")
    release.set()
    thread.join(1)

    assert not blocked.success
    assert blocked.error_code in {"MAP_OPERATION_BLOCKED", "MAP_OPERATION_BUSY"}
    assert result_holder and result_holder[0].success
