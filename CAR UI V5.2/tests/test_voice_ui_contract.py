from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def read(relative_path: str) -> str:
    return (ROOT / relative_path).read_text(encoding="utf-8")


def test_voice_control_page_contains_only_supported_voice_information():
    page = read("qml/pages/VoiceControlPage.qml")
    types = read("robot_api/types.py")
    assert "当前使用语音的人" in page
    assert "已登记的人员" in page
    assert "语音状态" in page
    assert all(state in page for state in ("正在听", "正在回答", "可以说话"))
    assert "允许陌生人使用语音控制" in page
    assert "VoiceprintManagerPage.qml" in page
    assert "speaker_distance" not in page
    assert "speaker_angle" not in page
    assert "voice_command_history" not in page
    assert "speaker_distance" not in types
    assert "speaker_angle" not in types
    assert "voice_command_history" not in types


def test_voiceprint_manager_is_two_pages_of_five_with_priority_controls():
    page = read("qml/pages/VoiceprintManagerPage.qml")
    item = read("qml/components/VoiceprintListItem.qml")
    assert "readonly property int pageSize: 5" in page
    assert "readonly property int maximumVoiceprints: 10" in page
    assert "backend.moveVoiceprint" in page
    assert "声纹已满，请先删除一个声纹" in page
    assert "voiceprint.priority" in item
    assert "上移" in item and "下移" in item
