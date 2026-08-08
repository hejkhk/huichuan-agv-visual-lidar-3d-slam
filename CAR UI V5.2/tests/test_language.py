from pathlib import Path
import re

from backend.ui_backend import UiBackend
from robot_api.mock import MockRobotApi


def test_language_is_validated_and_persisted(tmp_path):
    backend = UiBackend(MockRobotApi(tmp_path), tmp_path)
    assert backend.language == "zh"
    backend.setLanguage("ru")
    assert backend.language == "ru"
    assert backend.storage.read("settings.json")["language"] == "ru"
    backend.setLanguage("unsupported")
    assert backend.language == "ru"
    backend.shutdown()

    restored = UiBackend(MockRobotApi(tmp_path), tmp_path)
    assert restored.language == "ru"
    restored.shutdown()


def test_ros_domain_id_is_validated_and_persisted(tmp_path):
    backend = UiBackend(MockRobotApi(tmp_path), tmp_path)
    try:
        assert backend.rosDomainId == 88
        backend.setRosDomainId(42)
        assert backend.rosDomainId == 42
        assert backend.storage.read("settings.json")["ros_domain_id"] == 42
        backend.setRosDomainId(233)
        assert backend.rosDomainId == 42
    finally:
        backend.shutdown()


def test_every_literal_qml_translation_key_has_english_and_russian_entries():
    qml_root = Path(__file__).resolve().parents[1] / "qml"
    call_pattern = re.compile(r'I18n\.t\("((?:\\.|[^"\\])*)"\)')
    required = set()
    for path in qml_root.rglob("*.qml"):
        if path.name != "I18n.qml":
            required.update(call_pattern.findall(path.read_text(encoding="utf-8")))

    source = (qml_root / "I18n.qml").read_text(encoding="utf-8")
    english, russian = source.split("readonly property var russian:", 1)
    key_pattern = re.compile(r'"((?:\\.|[^"\\])*)"\s*:')
    assert required <= set(key_pattern.findall(english))
    assert required <= set(key_pattern.findall(russian))
