"""Build a stable, uniformly indexed LaserScan from LD14P rays.

The LD14P reports the measured angle of every packet.  A ROS LaserScan,
however, describes a uniform angular grid.  This module keeps those two
concepts separate: physical angles select grid cells, while array order
describes acquisition order.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Sequence, Tuple


Ray = Tuple[float, float, float]


@dataclass(frozen=True)
class FixedGridScan:
    """One validated revolution represented on a fixed angular grid."""

    start_time_ns: int
    end_time_ns: int
    raw_point_count: int
    valid_point_count: int
    filled_bin_count: int
    angle_min: float
    angle_max: float
    angle_increment: float
    scan_time: float
    time_increment: float
    ranges: Sequence[float]
    intensities: Sequence[float]


class FixedScanGridBuilder:
    """Split raw rays at point-level wraps and resample complete turns."""

    def __init__(
            self,
            bins: int = 360,
            angle_sign: float = -1.0,
            angle_offset_deg: float = 0.0,
            min_raw_points: int = 300,
            max_raw_points: int = 480,
            min_valid_points: int = 0,
            min_scan_time: float = 0.10,
            max_scan_time: float = 0.25,
            max_bin_error_deg: Optional[float] = None):
        if bins < 10:
            raise ValueError("bins must be at least 10")
        if angle_sign == 0.0:
            raise ValueError("angle_sign must be positive or negative")
        if min_raw_points < 1 or max_raw_points < min_raw_points:
            raise ValueError("invalid raw point limits")
        if min_valid_points < 0 or min_valid_points > bins:
            raise ValueError("invalid minimum valid point count")
        if min_scan_time <= 0.0 or max_scan_time <= min_scan_time:
            raise ValueError("invalid scan time limits")

        self.bins = int(bins)
        self.angle_sign = 1.0 if angle_sign > 0.0 else -1.0
        self.angle_offset_deg = float(angle_offset_deg) % 360.0
        self.min_raw_points = int(min_raw_points)
        self.max_raw_points = int(max_raw_points)
        self.min_valid_points = int(min_valid_points)
        self.min_scan_time = float(min_scan_time)
        self.max_scan_time = float(max_scan_time)
        self.bin_width_deg = 360.0 / self.bins
        self.max_bin_error_deg = (
            0.75 * self.bin_width_deg
            if max_bin_error_deg is None
            else float(max_bin_error_deg)
        )

        self._previous_raw_angle_deg: Optional[float] = None
        self._have_start_boundary = False
        self._start_time_ns: Optional[int] = None
        self._rays: List[Ray] = []

        self.published_count = 0
        self.dropped_count = 0
        self.partial_count = 0
        self.last_drop_reason = ""

    @staticmethod
    def _normalize_deg(angle_deg: float) -> float:
        return float(angle_deg) % 360.0

    @staticmethod
    def _angular_error_deg(a_deg: float, b_deg: float) -> float:
        return abs((a_deg - b_deg + 180.0) % 360.0 - 180.0)

    def add_ray(
            self,
            raw_angle_deg: float,
            distance_m: float,
            intensity: float,
            timestamp_ns: int) -> Optional[FixedGridScan]:
        """Add one ray and return a scan when this ray starts a new turn."""
        raw_angle_deg = self._normalize_deg(raw_angle_deg)
        timestamp_ns = int(timestamp_ns)

        wrapped = (
            self._previous_raw_angle_deg is not None
            and self._previous_raw_angle_deg > 300.0
            and raw_angle_deg < 60.0
        )

        completed = None
        if wrapped:
            if self._have_start_boundary:
                completed = self._finish(timestamp_ns)
            else:
                self.partial_count += 1
            self._rays = []
            self._start_time_ns = timestamp_ns
            self._have_start_boundary = True

        if self._have_start_boundary:
            self._rays.append((
                raw_angle_deg,
                float(distance_m),
                float(intensity),
            ))

        self._previous_raw_angle_deg = raw_angle_deg
        return completed

    def _finish(self, end_time_ns: int) -> Optional[FixedGridScan]:
        if self._start_time_ns is None:
            return None

        raw_count = len(self._rays)
        scan_time = (int(end_time_ns) - self._start_time_ns) / 1e9

        if not self.min_raw_points <= raw_count <= self.max_raw_points:
            return self._drop(
                f"raw_point_count={raw_count} outside "
                f"[{self.min_raw_points}, {self.max_raw_points}]"
            )
        if not self.min_scan_time <= scan_time <= self.max_scan_time:
            return self._drop(
                f"scan_time={scan_time:.6f}s outside "
                f"[{self.min_scan_time:.3f}, {self.max_scan_time:.3f}]"
            )

        ranges = [float("inf")] * self.bins
        intensities = [0.0] * self.bins
        selected_error = [float("inf")] * self.bins
        valid_count = 0

        for raw_angle_deg, distance_m, intensity in self._rays:
            index = int(round(raw_angle_deg / self.bin_width_deg)) % self.bins
            target_angle_deg = index * self.bin_width_deg
            error_deg = self._angular_error_deg(raw_angle_deg, target_angle_deg)
            if error_deg > self.max_bin_error_deg:
                continue

            is_valid = math.isfinite(distance_m) and distance_m > 0.0
            current_valid = math.isfinite(ranges[index])
            should_replace = error_deg < selected_error[index]
            if math.isclose(error_deg, selected_error[index], abs_tol=1e-9):
                should_replace = is_valid and not current_valid

            if should_replace:
                ranges[index] = distance_m if is_valid else float("inf")
                intensities[index] = intensity if is_valid else 0.0
                selected_error[index] = error_deg

        for distance_m in ranges:
            if math.isfinite(distance_m):
                valid_count += 1

        if valid_count < self.min_valid_points:
            return self._drop(
                f"valid_point_count={valid_count} below "
                f"minimum {self.min_valid_points}"
            )

        filled_count = sum(math.isfinite(error) for error in selected_error)
        angle_increment = math.radians(
            self.angle_sign * self.bin_width_deg)
        angle_min = math.radians(self.angle_offset_deg)
        angle_max = angle_min + (self.bins - 1) * angle_increment

        self.published_count += 1
        self.last_drop_reason = ""
        return FixedGridScan(
            start_time_ns=self._start_time_ns,
            end_time_ns=int(end_time_ns),
            raw_point_count=raw_count,
            valid_point_count=valid_count,
            filled_bin_count=filled_count,
            angle_min=angle_min,
            angle_max=angle_max,
            angle_increment=angle_increment,
            scan_time=scan_time,
            time_increment=scan_time / self.bins,
            ranges=tuple(ranges),
            intensities=tuple(intensities),
        )

    def _drop(self, reason: str) -> None:
        self.dropped_count += 1
        self.last_drop_reason = reason
        return None
