from pathlib import Path

from lidar_py.calibration_env import update_env_file


def test_update_env_file_preserves_comments_and_creates_backup(tmp_path):
    env_file = tmp_path / "dual_resolution_3d.env"
    original = (
        "# camera\n"
        "CAMERA_ROLL_DEG=0.0\n"
        "CAMERA_PITCH_DEG=25.04\n"
        "CAMERA_EXTRINSIC_CALIBRATED=false\n"
    )
    env_file.write_text(original, encoding="utf-8")

    backup = update_env_file(str(env_file), {
        "CAMERA_ROLL_DEG": "1.250",
        "CAMERA_Z": "0.3710",
    })

    rendered = env_file.read_text(encoding="utf-8")
    assert "# camera" in rendered
    assert "CAMERA_ROLL_DEG=1.250" in rendered
    assert "CAMERA_PITCH_DEG=25.04" in rendered
    assert "CAMERA_Z=0.3710" in rendered
    assert Path(backup).read_text(encoding="utf-8") == original


def test_no_change_does_not_create_backup(tmp_path):
    env_file = tmp_path / "dual_resolution_3d.env"
    env_file.write_text("CAMERA_YAW_DEG=2.000\n", encoding="utf-8")

    backup = update_env_file(str(env_file), {"CAMERA_YAW_DEG": "2.000"})

    assert backup == ""
    assert list(tmp_path.glob("*.bak.*")) == []
