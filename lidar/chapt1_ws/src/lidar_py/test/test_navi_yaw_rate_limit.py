import math
from types import SimpleNamespace

from lidar_py.chassis_node import ChassisNode


def make_filter(initial_deg):
    state = SimpleNamespace(
        navi_unwrapped_yaw_rad=math.radians(initial_deg),
        navi_last_raw_yaw_rad=math.radians(initial_deg),
        navi_max_yaw_rate_rad_s=math.radians(120.0),
        navi_yaw_limited_count=0,
    )
    state._wrap_pi = ChassisNode._wrap_pi
    return state


def update(state, target_deg, dt=0.02, gyro_deg_s=0.0):
    return ChassisNode._update_navi_yaw(
        state, math.radians(target_deg), dt, math.radians(gyro_deg_s))


def test_large_yaw_step_is_rejected_without_rebasing_to_corrupt_sample():
    state = make_filter(0.0)

    accepted, raw_delta, accepted_delta, limited = update(state, 45.0)

    assert limited
    assert math.isclose(math.degrees(raw_delta), 45.0, abs_tol=1e-6)
    assert accepted_delta == 0.0
    assert math.isclose(math.degrees(accepted), 0.0, abs_tol=1e-6)

    # A stable corrupt absolute offset must remain rejected.
    accepted, raw_delta, accepted_delta, limited = update(state, 45.0)
    assert limited
    assert math.isclose(math.degrees(raw_delta), 45.0, abs_tol=1e-6)
    assert accepted_delta == 0.0
    assert math.isclose(math.degrees(accepted), 0.0, abs_tol=1e-6)


def test_bad_absolute_yaw_uses_gyro_prediction_during_real_turn():
    state = make_filter(0.0)

    accepted, _, accepted_delta, limited = update(
        state, 40.0, gyro_deg_s=30.0)

    assert limited
    assert math.isclose(math.degrees(accepted_delta), 0.6, abs_tol=1e-6)
    assert math.isclose(math.degrees(accepted), 0.6, abs_tol=1e-6)


def test_slow_return_from_corrupt_offset_cannot_leak_into_heading():
    state = make_filter(0.0)

    for target in (40.0, 35.0, 30.0, 20.0, 10.0, 5.0):
        accepted, _, _, limited = update(state, target)
        assert limited
        assert math.isclose(math.degrees(accepted), 0.0, abs_tol=1e-6)

    accepted, _, _, limited = update(state, 0.0)
    assert not limited
    assert math.isclose(math.degrees(accepted), 0.0, abs_tol=1e-6)


def test_wraparound_uses_shortest_continuous_delta():
    state = make_filter(179.0)

    accepted, raw_delta, accepted_delta, limited = update(state, -179.0)

    assert not limited
    assert math.isclose(math.degrees(raw_delta), 2.0, abs_tol=1e-6)
    assert math.isclose(math.degrees(accepted_delta), 2.0, abs_tol=1e-6)
    assert math.isclose(math.degrees(accepted), 181.0, abs_tol=1e-6)


def test_invalid_time_does_not_apply_unbounded_jump():
    state = make_filter(10.0)

    accepted, _, accepted_delta, limited = ChassisNode._update_navi_yaw(
        state, math.radians(90.0), None)

    assert not limited
    assert accepted_delta == 0.0
    assert math.isclose(math.degrees(accepted), 10.0, abs_tol=1e-6)
