from PySide6.QtCore import QSize

from backend.map_image_provider import MapImageProvider
from robot_api.ros2_client import occupancy_grid_png


def test_live_map_provider_decodes_and_returns_qimage():
    encoded = occupancy_grid_png([100, 0, -1, 50], 2, 2)
    provider = MapImageProvider()

    assert provider.update_data_url(encoded)
    size = QSize()
    image = provider.requestImage("current?revision=1", size, QSize())
    assert not image.isNull()
    assert (size.width(), size.height()) == (2, 2)


def test_live_map_provider_rejects_incomplete_data():
    provider = MapImageProvider()

    assert not provider.update_data_url("data:image/png;base64,broken")
