import json
from pathlib import Path

from robot_api import create_robot_api


ROOT = Path(__file__).resolve().parents[1]


def test_mock_api_uses_requested_external_data_directory(tmp_path):
    api = create_robot_api(tmp_path)
    assert api.storage.data_dir == tmp_path
    assert (tmp_path / "settings.json").exists()
    assert (tmp_path / "mock_points.json").exists()
    assert (tmp_path / "mock_voiceprints.json").exists()


def test_arm64_launcher_uses_bundled_qt_and_external_data():
    launcher = (ROOT / "packaging" / "launcher-arm64.sh").read_text(encoding="utf-8")
    build_script = (ROOT / "build_arm64_deb.sh").read_text(encoding="utf-8")
    deploy_config = (ROOT / "robot_ui.pyproject").read_text(encoding="utf-8")

    assert "/opt/robot-touch-ui" in launcher
    assert "PYTHONPATH" in launcher
    assert "ROBOT_UI_DATA_DIR" in launcher
    assert "ROBOT_UI_PROJECT_ROOT" in launcher
    assert "exec /usr/bin/python3" in launcher
    assert '"$PROJECT_ROOT/assets"' in build_script
    assert '"$PROJECT_ROOT/map"' in build_script
    assert '"$PROJECT_ROOT/map_cache"' in build_script
    assert '"assets/vehicle.png"' in deploy_config


def test_deploy_project_lists_all_qml_and_visual_assets():
    project = json.loads(
        (ROOT / "robot_ui.pyproject").read_text(encoding="utf-8")
    )
    listed = set(project["files"])
    expected = {
        path.relative_to(ROOT).as_posix()
        for root in (ROOT / "qml", ROOT / "assets")
        for path in root.rglob("*")
        if path.is_file()
    }
    assert expected <= listed
