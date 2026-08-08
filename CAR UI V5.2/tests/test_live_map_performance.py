from __future__ import annotations

from array import array
from types import SimpleNamespace

from robot_api.ros2_client import Ros2Client


class _Owner:
    def __init__(self) -> None:
        self.snapshot = SimpleNamespace(map_revision=0)
        self.updates: list[dict[str, object]] = []

    def update_map(self, payload: dict[str, object]) -> None:
        self.updates.append(payload)
        self.snapshot.map_revision = int(payload["map_revision"])


def _message(data: array) -> SimpleNamespace:
    origin = SimpleNamespace(position=SimpleNamespace(x=-2.0, y=3.0))
    info = SimpleNamespace(
        width=2,
        height=2,
        resolution=0.05,
        origin=origin,
    )
    return SimpleNamespace(info=info, data=data)


def test_republished_identical_map_does_not_rebuild_texture() -> None:
    client = Ros2Client.__new__(Ros2Client)
    client.owner = _Owner()
    client._last_map_signature = None
    message = _message(array("b", [100, 0, -1, 50]))

    client._map_callback(message)
    client._map_callback(message)

    assert len(client.owner.updates) == 1
    assert client.owner.updates[0]["map_revision"] == 1


def test_changed_map_still_publishes_a_new_revision() -> None:
    client = Ros2Client.__new__(Ros2Client)
    client.owner = _Owner()
    client._last_map_signature = None

    client._map_callback(_message(array("b", [100, 0, -1, 50])))
    client._map_callback(_message(array("b", [100, 0, -1, 51])))

    assert [item["map_revision"] for item in client.owner.updates] == [1, 2]
