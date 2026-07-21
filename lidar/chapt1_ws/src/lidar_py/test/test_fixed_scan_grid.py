import math

from lidar_py.fixed_scan_grid import FixedScanGridBuilder


def _start_at_boundary(builder, timestamp_ns=0):
    builder.add_ray(350.0, 2.0, 10.0, timestamp_ns - 1_000_000)
    assert builder.add_ray(0.0, 2.0, 10.0, timestamp_ns) is None


def _finish_revolution(builder, point_count, start_ns, duration_ns, ranges=None):
    for index in range(1, point_count):
        angle_deg = index * 360.0 / point_count
        distance_m = 2.0 if ranges is None else ranges(angle_deg)
        builder.add_ray(
            angle_deg,
            distance_m,
            20.0,
            start_ns + index * duration_ns // point_count,
        )
    return builder.add_ray(0.0, 2.0, 10.0, start_ns + duration_ns)


def test_point_count_changes_do_not_change_output_geometry():
    builder = FixedScanGridBuilder(bins=360, angle_sign=-1.0)
    _start_at_boundary(builder)

    scan_384 = _finish_revolution(builder, 384, 0, 167_000_000)
    scan_396 = _finish_revolution(
        builder, 396, 167_000_000, 166_000_000)

    assert scan_384 is not None
    assert scan_396 is not None
    assert scan_384.raw_point_count == 384
    assert scan_396.raw_point_count == 396
    assert len(scan_384.ranges) == len(scan_396.ranges) == 360
    assert scan_384.filled_bin_count == scan_396.filled_bin_count == 360
    assert scan_384.ranges == scan_396.ranges
    assert math.isclose(scan_384.angle_increment, math.radians(-1.0))
    assert scan_384.angle_increment == scan_396.angle_increment
    assert scan_384.angle_min == scan_396.angle_min


def test_measured_angle_selects_the_grid_cell():
    builder = FixedScanGridBuilder(bins=360, angle_sign=1.0)
    _start_at_boundary(builder)

    def landmark(angle_deg):
        return 1.0 if abs(angle_deg - 90.0) < 0.48 else float("inf")

    scan = _finish_revolution(
        builder, 396, 0, 166_000_000, ranges=landmark)

    assert scan is not None
    assert math.isclose(scan.ranges[90], 1.0)
    assert math.isclose(scan.ranges[0], 2.0)
    assert all(
        not math.isfinite(value)
        for index, value in enumerate(scan.ranges)
        if index not in (0, 90)
    )


def test_malformed_revolution_is_rejected():
    builder = FixedScanGridBuilder(bins=360, angle_sign=-1.0)
    _start_at_boundary(builder)

    scan = _finish_revolution(builder, 120, 0, 167_000_000)

    assert scan is None
    assert builder.dropped_count == 1
    assert "raw_point_count=120" in builder.last_drop_reason


def test_sparse_revolution_is_rejected_by_optional_valid_point_gate():
    builder = FixedScanGridBuilder(
        bins=360, angle_sign=-1.0, min_valid_points=180)
    _start_at_boundary(builder)

    def sparse(angle_deg):
        return 2.0 if angle_deg < 120.0 else float("inf")

    scan = _finish_revolution(
        builder, 396, 0, 166_000_000, ranges=sparse)

    assert scan is None
    assert builder.dropped_count == 1
    assert "valid_point_count=" in builder.last_drop_reason


def test_valid_point_gate_accepts_well_observed_revolution():
    builder = FixedScanGridBuilder(
        bins=360, angle_sign=-1.0, min_valid_points=180)
    _start_at_boundary(builder)

    scan = _finish_revolution(builder, 396, 0, 166_000_000)

    assert scan is not None
    assert scan.valid_point_count == 360
