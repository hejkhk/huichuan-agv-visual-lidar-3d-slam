from lidar_py.lidar_timing import (
    MonotonicMinimumDelayMapper,
    WrappingMillisecondClock,
)


def test_ld14p_30000ms_rollover_stays_monotonic():
    clock = WrappingMillisecondClock(modulus=30000)

    first = clock.update(29980)
    before_wrap = clock.update(29990)
    after_wrap = clock.update(5)
    following = clock.update(15)

    assert first.reset
    assert before_wrap.unwrapped_ms == 29990
    assert after_wrap.wrapped
    assert not after_wrap.reset
    assert after_wrap.delta_ms == 15
    assert after_wrap.unwrapped_ms == 30005
    assert following.unwrapped_ms == 30015


def test_small_backward_jump_is_treated_as_clock_reset():
    clock = WrappingMillisecondClock(modulus=30000)
    clock.update(1000)

    update = clock.update(900)

    assert update.reset
    assert not update.wrapped
    assert update.unwrapped_ms == 900


def test_implausibly_large_forward_jump_is_treated_as_reset():
    clock = WrappingMillisecondClock(modulus=30000, max_step_ms=2000)
    clock.update(1000)

    update = clock.update(5000)

    assert update.reset
    assert update.unwrapped_ms == 5000


def test_clock_mapper_tracks_fast_device_clock_without_future_stamps():
    mapper = MonotonicMinimumDelayMapper(max_adjustment_ns=100_000)
    wire_ns = 4_100_000
    host_ns = 10_000_000_000
    device_ms = 0
    previous = None
    lags = []

    # V5 measured a device clock about 0.36 percent faster than the host.
    for index in range(75_000):
        device_ms += 4
        host_ns += 3_985_600
        usb_jitter_ns = (index % 7) * 80_000
        receipt_ns = host_ns + wire_ns + 2_000_000 + usb_jitter_ns
        mapped_ns = mapper.map_ms(device_ms, receipt_ns, wire_ns)
        if previous is not None:
            assert mapped_ns > previous
        previous = mapped_ns
        lag_ns = receipt_ns - mapped_ns
        assert lag_ns >= 0
        lags.append(lag_ns)

    assert max(lags) < 10_000_000
    assert mapper.total_adjustment_ns > 900_000_000


def test_clock_mapper_ignores_late_usb_packet():
    mapper = MonotonicMinimumDelayMapper(max_adjustment_ns=100_000)
    first = mapper.map_ms(1000, 2_000_010_000, 4_000_000)
    delayed = mapper.map_ms(1004, 2_100_014_000, 4_000_000)

    assert delayed > first
    assert mapper.total_adjustment_ns == 0
