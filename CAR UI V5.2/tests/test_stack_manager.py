import subprocess
import sys

import yaml

from robot_api.stack_manager import SlamStackManager


def test_embedded_ui_finds_parent_huichuan_project(tmp_path, monkeypatch):
    project = tmp_path / "huichuan-agv-visual-lidar-3d-slam"
    ui_root = project / "CAR UI V5.2"
    ui_root.mkdir(parents=True)
    (project / "START_DUAL_2D_3D_MAPPING.sh").write_text("#!/bin/bash\n")
    monkeypatch.delenv("HUICHUAN_SLAM_ROOT", raising=False)

    manager = SlamStackManager(ui_root)

    assert manager.project_root == project.resolve()


def test_status_exposes_persisted_selected_map(tmp_path, monkeypatch):
    project = tmp_path / "huichuan-agv-visual-lidar-3d-slam"
    ui_root = project / "CAR UI V5.2"
    ui_root.mkdir(parents=True)
    (project / "START_DUAL_2D_3D_MAPPING.sh").write_text("#!/bin/bash\n")
    monkeypatch.setenv("HUICHUAN_SLAM_ROOT", str(project))
    manager = SlamStackManager(ui_root)
    manager.state_dir = tmp_path / "state"
    manager.state_dir.mkdir()
    (manager.state_dir / "selected_map").write_text("floor_2\n")

    assert manager.status()["map_name"] == "floor_2"


def test_same_active_localization_map_does_not_restart(tmp_path, monkeypatch):
    project = tmp_path / "huichuan-agv-visual-lidar-3d-slam"
    ui_root = project / "CAR UI V5.2"
    ui_root.mkdir(parents=True)
    (project / "START_DUAL_2D_3D_MAPPING.sh").write_text("#!/bin/bash\n")
    (project / "START_DUAL_2D_3D_LOCALIZATION.sh").write_text("#!/bin/bash\n")
    monkeypatch.setenv("HUICHUAN_SLAM_ROOT", str(project))
    manager = SlamStackManager(ui_root)
    manager.state_dir = tmp_path / "state"
    manager.state_dir.mkdir()
    child = subprocess.Popen(
        [sys.executable, "-c", "import time; time.sleep(10)"],
        creationflags=getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0),
    )
    try:
        (manager.state_dir / "launcher.pid").write_text(f"{child.pid}\n")
        (manager.state_dir / "mode").write_text("localization\n")
        (manager.state_dir / "active_map").write_text("office\n")

        result = manager.start("localization", "office")

        assert result.success
        assert result.data["pid"] == child.pid
        assert child.poll() is None
    finally:
        child.terminate()
        child.wait(timeout=5)


def test_prepare_localization_map_normalizes_broken_image_reference(
    tmp_path, monkeypatch
):
    project = tmp_path / "huichuan-agv-visual-lidar-3d-slam"
    project.mkdir()
    (project / "START_DUAL_2D_3D_MAPPING.sh").write_text("#!/bin/bash\n")
    monkeypatch.setenv("HUICHUAN_SLAM_ROOT", str(project))

    source = tmp_path / "source"
    source.mkdir()
    source_yaml = source / "office.yaml"
    source_yaml.write_text(
        "image: old_export_name.pgm\n"
        "resolution: 0.05\n"
        "origin: [0.0, 0.0, 0.0]\n"
        "negate: 0\n"
        "occupied_thresh: 0.65\n"
        "free_thresh: 0.196\n",
        encoding="utf-8",
    )
    (source / "office.pgm").write_bytes(b"P5\n1 1\n255\n\x00")
    (source / "office.pbstream").write_bytes(b"pbstream")

    manager = SlamStackManager(tmp_path / "ui")
    result = manager.prepare_localization_map(str(source_yaml))

    assert result.success
    destination = project / "Loc_MAP"
    normalized = yaml.safe_load(
        (destination / "office.yaml").read_text(encoding="utf-8")
    )
    assert normalized["image"] == "office.pgm"
    assert (destination / "office.pgm").is_file()
    assert (destination / "office.pbstream").is_file()
