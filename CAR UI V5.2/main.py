from __future__ import annotations

import logging
import json
import os
import signal
import sys
from pathlib import Path

# The Ubuntu desktop session exports QT_IM_MODULE=ibus.  This application
# embeds Qt Virtual Keyboard, so it must override the desktop input context
# before QGuiApplication is constructed; otherwise Qt.inputMethod.show()
# talks to IBus and the in-app keyboard cannot be summoned reliably.
os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"
# Qt 6 does not support the embedded virtual keyboard as a client-side input
# context on Wayland. Ubuntu provides XWayland, so default to XCB for this
# standalone full-screen HMI. Tests and deployments can still override it
# explicitly (for example QT_QPA_PLATFORM=offscreen).
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

from PySide6.QtCore import QDir, QLockFile, QtMsgType, QTimer, QUrl, qInstallMessageHandler
from PySide6.QtGui import QGuiApplication
from PySide6.QtQml import QQmlApplicationEngine

from backend.map_image_provider import MapImageProvider
from backend.ui_backend import UiBackend
from robot_api import create_robot_api

_qt_shutting_down = False


def configure_logging() -> None:
    level = getattr(logging, os.getenv("ROBOT_UI_LOG_LEVEL", "INFO").upper(), logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s.%(msecs)03d [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )

    def qt_message_handler(kind: QtMsgType, _context: object, message: str) -> None:
        # Qt Virtual Keyboard 6.11.1 emits this internal qmltypes warning from
        # its bundled PopupList. It does not refer to application QML.
        if (
            "Member contentWidth of the object PopupList_" in message
            and "overrides a member of the base object" in message
        ):
            return
        if _qt_shutting_down and "Cannot read property" in message and "of null" in message:
            return
        if kind == QtMsgType.QtDebugMsg and level > logging.DEBUG:
            return
        logger = logging.getLogger("QT")
        if kind in (QtMsgType.QtCriticalMsg, QtMsgType.QtFatalMsg): logger.error(message)
        elif kind == QtMsgType.QtWarningMsg: logger.warning(message)
        else: logger.info(message)
    qInstallMessageHandler(qt_message_handler)


def configure_ros_domain_id(data_dir: Path) -> None:
    """Apply the persisted DDS domain before any ROS 2 node is constructed."""

    if "ROS_DOMAIN_ID" in os.environ:
        return
    domain_id = 88
    try:
        with (data_dir / "settings.json").open("r", encoding="utf-8") as handle:
            configured = int(json.load(handle).get("ros_domain_id", domain_id))
        if 0 <= configured <= 232:
            domain_id = configured
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        pass
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)


def create_engine() -> tuple[QGuiApplication, QQmlApplicationEngine]:
    configure_logging()
    # Use Qt's application-embedded keyboard. PySide6 bundles the Simplified
    # Chinese Pinyin plugin, so text input does not depend on a desktop IME.
    app = QGuiApplication(sys.argv)
    app.setApplicationName("Robot Touch UI")
    app.setOrganizationName("HongxinDeli")
    app.setOrganizationDomain("hongxindeli.local")
    app.setDesktopFileName("robot-touch-ui")
    instance_lock = QLockFile(QDir.temp().filePath("robot-touch-ui-single-instance.lock"))
    instance_lock.setStaleLockTime(0)
    if not instance_lock.tryLock(0):
        raise RuntimeError("机器人车载 UI 已在运行")
    app._single_instance_lock = instance_lock
    engine = QQmlApplicationEngine()
    root = Path(__file__).resolve().parent
    data_dir = Path(os.getenv("ROBOT_UI_DATA_DIR", str(root / "data"))).expanduser()
    project_root = Path(
        os.getenv("ROBOT_UI_PROJECT_ROOT", str(root))
    ).expanduser().resolve()
    configure_ros_domain_id(data_dir)
    logging.getLogger("ROS").info(
        "ROS 环境：domain=%s rmw=%s dds=%s map=%s reference_map=%s project=%s",
        os.getenv("ROS_DOMAIN_ID", "未设置"),
        os.getenv("RMW_IMPLEMENTATION", "默认"),
        os.getenv("CYCLONEDDS_URI", "未设置"),
        os.getenv("ROBOT_UI_MAP_TOPIC", "/map"),
        os.getenv("ROBOT_UI_REFERENCE_MAP_TOPIC", "/localization_reference_map"),
        os.getenv("HUICHUAN_SLAM_ROOT", "未设置"),
    )
    map_image_provider = MapImageProvider()
    engine.addImageProvider("live-map", map_image_provider)
    backend = UiBackend(
        create_robot_api(data_dir),
        data_dir,
        project_root,
        map_image_sink=map_image_provider,
    )
    engine._backend = backend  # keep the context object alive for the engine lifetime
    engine._map_image_provider = map_image_provider
    engine.rootContext().setContextProperty("backend", backend)
    engine.rootContext().setContextProperty("autoFullscreen", not os.getenv("ROBOT_UI_AUTOCLOSE_MS"))
    engine.addImportPath(str(root / "qml"))
    engine.load(QUrl.fromLocalFile(str(root / "qml" / "Main.qml")))
    if not engine.rootObjects():
        raise RuntimeError("QML failed to load")
    app.aboutToQuit.connect(backend.shutdown)
    return app, engine


def main() -> int:
    global _qt_shutting_down
    app, engine = create_engine()
    signal.signal(signal.SIGINT, lambda _signum, _frame: app.quit())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: app.quit())
    signal_timer = QTimer()
    signal_timer.setInterval(250)
    signal_timer.timeout.connect(lambda: None)
    signal_timer.start()
    logging.getLogger("UI").info("机器人车载 UI 已启动")
    auto_close_ms = int(os.getenv("ROBOT_UI_AUTOCLOSE_MS", "0"))
    if auto_close_ms > 0:
        QTimer.singleShot(auto_close_ms, app.quit)
    exit_code = app.exec()
    _qt_shutting_down = True
    engine.deleteLater()
    app.processEvents()
    logging.getLogger("UI").info("机器人车载 UI 已退出")
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
