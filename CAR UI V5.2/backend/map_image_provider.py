from __future__ import annotations

import base64
import threading

from PySide6.QtCore import QSize, Qt
from PySide6.QtGui import QImage
from PySide6.QtQuick import QQuickImageProvider


class MapImageProvider(QQuickImageProvider):
    """Keep the current ROS map decoded once for all QML map views.

    ``RobotSnapshot.map_image`` remains the integration-facing PNG data URL.
    The UI backend hands a new value to this provider only when
    ``map_revision`` changes. Home and fullscreen views then share the
    ``image://live-map`` source instead of decoding separate base64 copies.
    """

    def __init__(self) -> None:
        super().__init__(QQuickImageProvider.Image)
        self._lock = threading.Lock()
        self._image = QImage()

    def update_data_url(self, data_url: str) -> bool:
        """Decode a PNG data URL and atomically publish a complete QImage."""

        try:
            _header, separator, payload = data_url.partition(",")
            if not separator or not payload:
                return False
            encoded = base64.b64decode(payload, validate=True)
            image = QImage.fromData(encoded, "PNG")
            if image.isNull():
                return False
        except (ValueError, TypeError):
            return False

        with self._lock:
            self._image = image
        return True

    def clear(self) -> None:
        with self._lock:
            self._image = QImage()

    def requestImage(
        self, _image_id: str, size: QSize, requested_size: QSize
    ) -> QImage:
        """Return an implicitly shared image without copying the full map.

        ``QImage`` uses copy-on-write storage. The provider never mutates a
        published image, so retaining a shallow reference is safe while also
        avoiding a full-frame memory copy on every QML texture request.
        """

        with self._lock:
            image = QImage(self._image)
        if image.isNull():
            size.setWidth(0)
            size.setHeight(0)
            return image

        size.setWidth(image.width())
        size.setHeight(image.height())
        if (
            requested_size.isValid()
            and requested_size.width() > 0
            and requested_size.height() > 0
            and (
                requested_size.width() < image.width()
                or requested_size.height() < image.height()
            )
        ):
            return image.scaled(
                requested_size,
                Qt.KeepAspectRatio,
                Qt.SmoothTransformation,
            )
        return image
