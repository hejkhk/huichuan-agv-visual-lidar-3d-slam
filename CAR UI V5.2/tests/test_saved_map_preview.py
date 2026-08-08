from __future__ import annotations

import base64
from pathlib import Path
import struct
import threading

import yaml

from backend.map_preview import load_saved_map_preview, map_signature
from robot_api.team import TeamRobotApi
from robot_api.types import RobotSnapshot


def _write_map(directory: Path) -> Path:
    directory.mkdir(parents=True)
    (directory / "office.pgm").write_bytes(
        b"P5\n# saved map\n3 2\n255\n" + bytes([0, 254, 205, 254, 0, 254])
    )
    payload = {
        "image": "office.pgm",
        "resolution": 0.05,
        "origin": [-1.5, 2.0, 0.0],
        "negate": 0,
        "occupied_thresh": 0.65,
        "free_thresh": 0.196,
        "mode": "trinary",
    }
    yaml_path = directory / "office.yaml"
    yaml_path.write_text(
        yaml.safe_dump(payload, sort_keys=False), encoding="utf-8"
    )
    return yaml_path


def test_saved_map_preview_is_immediately_displayable(tmp_path):
    preview = load_saved_map_preview(_write_map(tmp_path / "Loc_MAP"))

    assert preview["map_width"] == 3
    assert preview["map_height"] == 2
    assert preview["map_resolution"] == 0.05
    assert preview["map_origin_x"] == -1.5
    assert preview["map_crc32"].startswith("0x")
    encoded = preview["map_image"]
    assert encoded.startswith("data:image/png;base64,")
    assert base64.b64decode(encoded.split(",", 1)[1]).startswith(
        b"\x89PNG\r\n\x1a\n"
    )


def test_saved_map_preview_signature_changes_with_pgm(tmp_path):
    yaml_path = _write_map(tmp_path / "Loc_MAP")
    first = load_saved_map_preview(yaml_path)["map_signature"]
    image = yaml_path.with_suffix(".pgm")
    raw = bytearray(image.read_bytes())
    raw[-1] = 0
    image.write_bytes(raw)

    second = load_saved_map_preview(yaml_path)["map_signature"]

    assert first != second


def test_map_signature_normalizes_ros_float32_resolution():
    ros_resolution = struct.unpack("f", struct.pack("f", 0.05))[0]

    from_yaml = map_signature([0, 100], 2, 1, 0.05, -1.5, 2.0)
    from_ros = map_signature([0, 100], 2, 1, ros_resolution, -1.5, 2.0)

    assert from_yaml == from_ros


def test_filesystem_preview_does_not_impersonate_ros_connection():
    api = TeamRobotApi.__new__(TeamRobotApi)
    api._snapshot_lock = threading.RLock()
    api.snapshot = RobotSnapshot()

    api.update_map(
        {
            "map_image": "data:image/png;base64,AA==",
            "map_width": 2,
            "map_height": 1,
            "map_revision": 1,
        },
        ros_source=False,
    )

    assert api.snapshot.map_available
    assert not api.snapshot.ros_connected
    api.update_ros_connection(True)
    assert api.snapshot.ros_connected
