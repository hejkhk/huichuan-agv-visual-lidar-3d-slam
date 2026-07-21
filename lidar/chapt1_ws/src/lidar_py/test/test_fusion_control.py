import math

from lidar_py.fusion_control import (
    DepthSample,
    FusionConfig,
    FusionController,
    build_virtual_scan_ranges,
)


def test_avoidance_never_raises_nav_speed():
    controller = FusionController()
    sample = DepthSample(level=1, center_far=True, nearest_mm=700)
    result = controller.update(0.04, 0.0, sample, now=1.0, dt=0.05)
    assert 0.0 < result.linear_x <= 0.04


def test_stopped_nav_does_not_receive_vision_rotation():
    controller = FusionController()
    sample = DepthSample(
        level=2,
        center_danger=True,
        nearest_mm=380,
        preferred_dir=-1,
    )
    result = controller.update(0.0, 0.0, sample, now=1.0, dt=0.05)
    assert result.linear_x == 0.0
    assert result.angular_z == 0.0


def test_nav_recovery_spin_is_not_reversed_by_vision():
    controller = FusionController()
    sample = DepthSample(level=1, center_far=True, nearest_mm=700)
    result = controller.update(0.0, -0.3, sample, now=1.0, dt=0.05)
    assert result.angular_z == -0.3


def test_direction_is_latched_against_small_score_changes():
    config = FusionConfig(direction_switch_margin=80, direction_switch_frames=3)
    controller = FusionController(config)
    left_obstacle = DepthSample(
        level=1,
        center_far=True,
        left_score_x1000=150,
        right_score_x1000=20,
    )
    first = controller.update(0.15, 0.0, left_obstacle, now=1.0, dt=0.05)
    assert first.direction == -1

    noisy_opposite = DepthSample(
        level=1,
        center_far=True,
        left_score_x1000=80,
        right_score_x1000=110,
    )
    second = controller.update(0.15, 0.0, noisy_opposite, now=1.1, dt=0.05)
    assert second.direction == -1


def test_level_release_has_hysteresis():
    config = FusionConfig(level_release_hold_sec=0.4)
    controller = FusionController(config)
    obstacle = DepthSample(level=2, center_danger=True, nearest_mm=400)
    clear = DepthSample()
    assert controller.update(0.1, 0.0, obstacle, 1.0, 0.05).level == 2
    assert controller.update(0.1, 0.0, clear, 1.2, 0.05).level == 2
    assert controller.update(0.1, 0.0, clear, 1.7, 0.05).level == 0


def test_virtual_scan_marks_center_and_sides():
    sample = DepthSample(
        level=2,
        center_danger=True,
        left_blocked=True,
        nearest_mm=420,
        center_min_mm=380,
    )
    ranges = build_virtual_scan_ranges(sample, stable_level=2)
    assert len(ranges) == 61
    assert math.isclose(ranges[30], 0.68)
    assert any(math.isclose(value, 0.68) for value in ranges[40:])
    assert math.isinf(ranges[0])
