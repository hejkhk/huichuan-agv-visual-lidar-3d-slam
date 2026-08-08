from backend.ui_backend import UiBackend
from robot_api.mock import MockRobotApi


def test_performance_modes_persist_and_adjust_only_ui_polling(tmp_path):
    backend = UiBackend(MockRobotApi(tmp_path), tmp_path, tmp_path)
    try:
        expected = {0: 1500, 1: 750, 2: 400}
        for mode, interval in expected.items():
            backend.setPerformanceMode(mode)
            assert backend.settings["performance_mode"] == mode
            assert backend._poll_timer.interval() == interval
        backend.setPerformanceMode(99)
        assert backend.settings["performance_mode"] == 2
        assert backend._poll_timer.interval() == 400
    finally:
        backend.shutdown()
