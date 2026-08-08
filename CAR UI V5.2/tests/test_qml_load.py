import os
import time
from pathlib import Path

import pytest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["QT_IM_MODULE"] = "qtvirtualkeyboard"

PySide6 = pytest.importorskip("PySide6")


def test_main_qml_loads(tmp_path):
    from PySide6.QtCore import QUrl
    from PySide6.QtGui import QGuiApplication
    from PySide6.QtQml import QQmlApplicationEngine
    from PySide6.QtQuick import QQuickItem
    from PySide6.QtTest import QTest
    from backend.ui_backend import UiBackend
    from robot_api.mock import MockRobotApi
    map_dir = tmp_path / "map"
    map_dir.mkdir()
    for index, name in enumerate(("仓库", "office", "loading_zone")):
        (map_dir / f"{name}.pgm").write_bytes(
            b"P5\n2 2\n255\n" + bytes([index * 60, 80, 160, 240])
        )
        (map_dir / f"{name}.yaml").write_text(
            f"image: {name}.pgm\n"
            "resolution: 0.05\n"
            "origin: [0.0, 0.0, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n",
            encoding="utf-8",
        )
    app = QGuiApplication.instance() or QGuiApplication([])
    app.setOrganizationName("HongxinDeliTests")
    app.setOrganizationDomain("tests.local")
    app.setApplicationName("Robot Touch UI Tests")
    root = Path(__file__).resolve().parents[1]
    engine = QQmlApplicationEngine()
    backend = UiBackend(MockRobotApi(tmp_path), tmp_path, tmp_path)
    backend.setPerformanceMode(
        int(os.getenv("ROBOT_UI_TEST_PERFORMANCE_MODE", "1"))
    )
    backend.setLanguage("zh")
    engine.rootContext().setContextProperty("backend", backend)
    engine.addImportPath(str(root / "qml"))
    engine.load(QUrl.fromLocalFile(str(root / "qml" / "Main.qml")))
    assert engine.rootObjects()
    window = engine.rootObjects()[0]
    keyboard = window.findChild(
        PySide6.QtCore.QObject, "applicationVirtualKeyboard"
    )
    add_point_dialog = window.findChild(
        PySide6.QtCore.QObject, "addPointDialog"
    )
    assert keyboard is not None and add_point_dialog is not None
    window.setWidth(1920)
    window.setHeight(1080)
    window.setColorScheme(0)
    window.setDarkMode(False)
    for font_mode in range(3):
        window.setFontSizeMode(font_mode); app.processEvents()
        assert window.property("activeFontSizeMode") == font_mode
    for border_mode in range(3):
        window.setBorderMode(border_mode); app.processEvents()
        assert window.property("activeBorderMode") == border_mode
    window.setFontSizeMode(1)
    window.setBorderMode(0)
    window.show()
    QTest.qWait(620)
    home = window.findChild(PySide6.QtCore.QObject, "homeControlHub")
    left_panel = window.findChild(PySide6.QtCore.QObject, "homeInformationPanel")
    follow_loader = window.findChild(PySide6.QtCore.QObject, "embeddedFollowLoader")
    assert home is not None and left_panel is not None and follow_loader is not None
    window.startHomeTutorial()
    QTest.qWait(120)
    tutorial_loader = window.findChild(
        PySide6.QtCore.QObject, "homeTutorialLoader"
    )
    assert tutorial_loader is not None
    assert tutorial_loader.property("active") is True
    tutorial = tutorial_loader.property("item")
    assert tutorial is not None
    assert tutorial.property("stepIndex") == 0
    assert tutorial.property("manualNavigation") is False
    QTest.qWait(4200)
    assert tutorial.property("stepIndex") == 1
    previous_index = tutorial.property("stepIndex")
    tutorial.nextStep(True)
    QTest.qWait(520)
    assert tutorial.property("stepIndex") == previous_index + 1
    assert tutorial.property("manualNavigation") is True
    manual_index = tutorial.property("stepIndex")
    QTest.qWait(3500)
    assert tutorial.property("stepIndex") == manual_index
    window.goBack(); app.processEvents()
    assert tutorial_loader.property("active") is False
    home.setProperty("followControlExpanded", True)
    QTest.qWait(120)
    assert follow_loader.property("active") is True
    assert follow_loader.property("item") is not None
    embedded_follow = window.findChild(PySide6.QtCore.QObject, "embeddedFollowControl")
    follow_slider = window.findChild(PySide6.QtCore.QObject, "followControlDistanceSlider")
    assert embedded_follow is not None and follow_slider is not None
    assert left_panel.property("visible") is True
    window.goBack(); QTest.qWait(120)
    assert home.property("followControlExpanded") is False
    assert follow_loader.property("item") is None
    voice_loader = window.findChild(
        PySide6.QtCore.QObject, "embeddedVoiceLoader"
    )
    assert voice_loader is not None
    home.setProperty("voiceControlExpanded", True)
    QTest.qWait(120)
    assert voice_loader.property("item") is not None
    assert window.findChild(
        PySide6.QtCore.QObject, "embeddedVoiceControl"
    ) is not None
    window.goBack(); QTest.qWait(120)
    assert home.property("voiceControlExpanded") is False
    assert voice_loader.property("item") is None
    deadline = time.monotonic() + 2
    while len(backend.maps) < 3 and time.monotonic() < deadline:
        app.processEvents()
        time.sleep(0.02)
    assert len(backend.maps) == 3
    add_point_dialog.open()
    # Offscreen validates state; XCB runs may additionally request a real
    # InputPanel visibility assertion with ROBOT_UI_EXPECT_KEYBOARD_VISIBLE=1.
    QTest.qWait(300)
    keyboard = window.findChild(
        PySide6.QtCore.QObject, "applicationVirtualKeyboard"
    )
    assert keyboard is not None
    input_field = window.findChild(
        PySide6.QtCore.QObject, "addPointNameField"
    )
    assert input_field is not None
    assert add_point_dialog.property("visible")
    assert window.property("inputDialogActive") is True
    assert window.property("keyboardRequested") is False
    window.showKeyboardFor(input_field); app.processEvents()
    assert window.property("keyboardRequested") is True
    assert keyboard.property("allowed") is True
    QTest.qWait(400)
    if os.getenv("ROBOT_UI_EXPECT_KEYBOARD_VISIBLE") == "1":
        assert keyboard.property("visible") is True
    window.dismissKeyboard(); app.processEvents()
    assert window.property("keyboardRequested") is False
    assert keyboard.property("allowed") is False
    window.showKeyboardFor(input_field); app.processEvents()
    assert window.property("keyboardRequested") is True
    assert keyboard.property("allowed") is True
    add_point_dialog.close()
    QTest.qWait(50)
    keyboard = window.findChild(
        PySide6.QtCore.QObject, "applicationVirtualKeyboard"
    )
    assert keyboard is not None
    assert not keyboard.property("visible")
    assert window.property("inputDialogActive") is False
    assert window.property("keyboardRequested") is False
    app.processEvents()

    for scheme in range(4):
        for dark_mode in (False, True):
            window.setColorScheme(scheme)
            window.setDarkMode(dark_mode)
            app.processEvents()
            assert window.property("activeColorScheme") == scheme
            assert window.property("activeDarkMode") is dark_mode
    window.setColorScheme(0)
    window.setDarkMode(False)
    # The production panel contract is intentionally fixed at 1920×1080.
    for width, height in ((1920, 1080),):
        window.setWidth(width); window.setHeight(height); app.processEvents()
        assert window.width() == width and window.height() == height
        home.setProperty("followControlExpanded", True); app.processEvents()
        assert follow_loader.property("item") is not None
        assert left_panel.property("visible") is True
        window.goBack(); app.processEvents()
        assert follow_loader.property("item") is None
    for language in ("zh", "en", "ru"):
        backend.setLanguage(language); app.processEvents()
        assert backend.language == language
        assert window.property("title") == {"zh": "机器人车载触控屏", "en": "Robot Vehicle Touchscreen", "ru": "Сенсорная панель робота"}[language]
        for page in ("SettingsPage.qml", "PointManagerPage.qml", "RobotStatusPage.qml", "FollowPage.qml", "VoiceControlPage.qml", "VoiceprintManagerPage.qml", "RvizFullscreenPage.qml", "MapSelectorPage.qml", "GamepadTutorialPage.qml"):
            window.pushPage(page); app.processEvents()
            assert engine.rootObjects()
            window.goHome(); app.processEvents()
    window.pushPage("MapSelectorPage.qml")
    QTest.qWait(350)
    stack = window.findChild(PySide6.QtCore.QObject, "pageStack")
    current_page = stack.property("currentItem")
    carousel = current_page.findChild(
        PySide6.QtCore.QObject, "mapCarousel"
    )
    assert carousel is not None
    assert carousel.property("currentIndex") == 0
    assert getattr(carousel, "next")()
    assert not getattr(carousel, "next")()
    QTest.qWait(600)
    assert carousel.property("currentIndex") == 1
    assert getattr(carousel, "next")()
    QTest.qWait(600)
    assert carousel.property("currentIndex") == 2
    assert getattr(carousel, "next")()
    assert not getattr(carousel, "next")()
    QTest.qWait(600)
    assert carousel.property("currentIndex") == 0
    assert getattr(carousel, "previous")()
    QTest.qWait(600)
    assert carousel.property("currentIndex") == 2
    window.goHome()
    window.pushPage("RvizFullscreenPage.qml"); app.processEvents()
    status_bar = window.findChild(PySide6.QtCore.QObject, "statusBar")
    busy_overlay = window.findChild(PySide6.QtCore.QObject, "busyOverlay")
    assert stack.property("depth") > 1
    fullscreen_map = window.findChild(
        PySide6.QtCore.QObject, "fullscreenMap"
    )
    assert fullscreen_map.property("fullscreenPage")
    fullscreen_map.setProperty("headingUp", True)
    app.processEvents()
    assert fullscreen_map.property("headingUp")
    fullscreen_map.resetView()
    assert not fullscreen_map.property("headingUp")
    fullscreen_map.fullscreenRequested.emit()
    app.processEvents()
    assert stack.property("depth") == 1
    window.pushPage("RvizFullscreenPage.qml")
    app.processEvents()
    backend._set_busy(True); app.processEvents()
    assert not busy_overlay.property("visible")
    deadline = time.monotonic() + 0.35
    while not busy_overlay.property("visible") and time.monotonic() < deadline:
        app.processEvents(); time.sleep(0.01)
    assert busy_overlay.property("visible")
    backend._set_busy(False); app.processEvents()
    assert not busy_overlay.property("visible")
    status_bar.homeRequested.emit(); app.processEvents()
    assert stack.property("depth") == 1
    window.pushPage("SettingsPage.qml"); app.processEvents()
    status_bar.backRequested.emit(); app.processEvents()
    assert stack.property("depth") == 1
    from PySide6.QtGui import QWindow
    status_bar.fullscreenRequested.emit(); app.processEvents()
    assert window.visibility() == QWindow.FullScreen
    status_bar.fullscreenRequested.emit(); app.processEvents()
    assert window.visibility() != QWindow.FullScreen

    pose_results = []
    backend.currentPoseReady.connect(lambda pose: pose_results.append(pose))
    backend.requestCurrentPose()
    deadline = time.monotonic() + 2
    while (backend.busy or not pose_results) and time.monotonic() < deadline:
        app.processEvents(); time.sleep(0.01)
    assert pose_results and not backend.busy and not backend._workers

    backend.beginVoiceprint("测试声纹")
    deadline = time.monotonic() + 2
    while backend.busy and time.monotonic() < deadline:
        app.processEvents(); time.sleep(0.01)
    assert not backend.busy and not backend._workers

    window.pushPage("SettingsPage.qml"); app.processEvents()
    settings_page = window.findChild(PySide6.QtCore.QObject, "settingsPage")
    if settings_page:
        settings_page.setProperty("activePanel", 6); app.processEvents()
        settings_page.setProperty("activePanel", 7); app.processEvents()
        settings_page.setProperty("activePanel", 8); app.processEvents()
        assert settings_page.property("activePanel") == 8
    status_bar.backRequested.emit(); app.processEvents()

    backend.shutdown()
