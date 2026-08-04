from lidar_py.lidar_timing import AdaptiveMinimumDelayMapper


def test_mapper_rejects_positive_receipt_jitter():
    mapper = AdaptiveMinimumDelayMapper(window_size=5, max_adjustment_ns=20_000)
    base = 1_000_000_000
    first = mapper.map_ms(100, base + 100_000_000)
    delayed = mapper.map_ms(120, base + 120_000_000 + 8_000_000)

    assert first == base + 100_000_000
    assert delayed == base + 120_000_000


def test_mapper_slews_when_recent_minimum_moves_forward():
    mapper = AdaptiveMinimumDelayMapper(window_size=3, max_adjustment_ns=20_000)
    receipt_offset = 2_000_000_000
    mapper.map_ms(100, receipt_offset + 100_000_000)

    for tick in (120, 140, 160):
        mapper.map_ms(tick, receipt_offset + tick * 1_000_000 + 100_000)

    assert mapper.offset_ns == receipt_offset + 20_000


def test_mapper_output_is_strictly_monotonic():
    mapper = AdaptiveMinimumDelayMapper(window_size=5, max_adjustment_ns=500_000)
    stamps = [
        mapper.map_ms(100, 5_100_000_000),
        mapper.map_ms(101, 5_100_500_000),
        mapper.map_ms(102, 5_101_000_000),
    ]
    assert stamps[0] < stamps[1] < stamps[2]
