import base64
import math

import pytest


pytest.importorskip("rclpy")

from robot_api.ros2_client import occupancy_grid_png, quaternion_to_yaw


def test_occupancy_grid_encodes_png_data_url():
    encoded = occupancy_grid_png([100, 0, -1, 50], 2, 2)
    payload = base64.b64decode(encoded.split(",", 1)[1])

    assert encoded.startswith("data:image/png;base64,")
    assert payload.startswith(b"\x89PNG\r\n\x1a\n")


def test_quaternion_to_yaw_for_planar_rotation():
    yaw = math.pi / 2

    assert quaternion_to_yaw(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)) == pytest.approx(yaw)
