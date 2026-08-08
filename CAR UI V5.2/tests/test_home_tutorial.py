from pathlib import Path

from backend.ui_backend import UiBackend
from robot_api.mock import MockRobotApi


ROOT = Path(__file__).resolve().parents[1]


def test_home_tutorial_contract_and_performance_levels():
    model = (ROOT / "qml/components/HomeTutorialModel.qml").read_text(encoding="utf-8")
    overlay = (ROOT / "qml/components/HomeTutorialOverlay.qml").read_text(encoding="utf-8")
    main = (ROOT / "qml/Main.qml").read_text(encoding="utf-8")
    developer = (ROOT / "qml/pages/DeveloperPanel.qml").read_text(encoding="utf-8")

    assert model.count('id: "') == 9
    for target in (
        "map", "map_tools", "travel_status", "vehicle", "navigation",
        "voice", "gamepad", "follow", "status_bar",
    ):
        assert f'target: "{target}"' in model
    assert "interval: 500" in overlay
    assert "onCycleFinished" in overlay
    assert "manualNavigation = true" in overlay
    assert "Performance.lowPower" in overlay
    assert "Performance.smooth" in overlay
    assert "ShaderEffectSource" in overlay
    assert "id: pageTransition" in overlay
    assert "id: focusIntro" in overlay
    assert "SequentialAnimation on scale" not in overlay
    assert "blurEnabled: true" in main
    assert 'objectName: "homeTutorialPrompt"' in main
    assert 'objectName: "showHomeTutorialOnStartupSwitch"' in developer
    assert 'objectName: "startHomeTutorialButton"' in developer


def test_home_tutorial_startup_setting_persists(tmp_path):
    backend = UiBackend(MockRobotApi(tmp_path), tmp_path, tmp_path)
    try:
        assert backend.settings["show_home_tutorial_on_startup"] is False
        backend.setShowHomeTutorialOnStartup(True)
        assert backend.settings["show_home_tutorial_on_startup"] is True
        assert backend.storage.read("settings.json")["show_home_tutorial_on_startup"] is True
    finally:
        backend.shutdown()
