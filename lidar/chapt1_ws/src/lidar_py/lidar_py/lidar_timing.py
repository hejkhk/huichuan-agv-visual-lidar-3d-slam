"""Timing helpers for LDROBOT LiDAR packet clocks."""

from dataclasses import dataclass
from collections import deque
from typing import Deque, Optional


@dataclass(frozen=True)
class ClockUpdate:
    unwrapped_ms: int
    delta_ms: Optional[int]
    reset: bool
    wrapped: bool


class WrappingMillisecondClock:
    """Unwrap a bounded millisecond counter without resetting at rollover."""

    def __init__(self, modulus: int, max_step_ms: int = 2000):
        if modulus <= 0:
            raise ValueError("modulus must be positive")
        if max_step_ms <= 0 or max_step_ms >= modulus:
            raise ValueError("invalid maximum step")
        self.modulus = int(modulus)
        self.max_step_ms = int(max_step_ms)
        self.last_raw: Optional[int] = None
        self.unwrapped_ms: Optional[int] = None

    def update(self, raw_value: int) -> ClockUpdate:
        raw = int(raw_value)
        if raw < 0 or raw > self.modulus:
            raise ValueError("counter value outside configured range")

        if self.last_raw is None:
            self.last_raw = raw
            self.unwrapped_ms = raw
            return ClockUpdate(raw, None, True, False)

        delta = raw - self.last_raw
        wrapped = (
            delta < 0
            and self.last_raw >= 3 * self.modulus // 4
            and raw <= self.modulus // 4
        )
        if wrapped:
            delta += self.modulus

        reset = delta < 0 or delta > self.max_step_ms
        if reset:
            self.unwrapped_ms = raw
        else:
            self.unwrapped_ms += delta

        self.last_raw = raw
        return ClockUpdate(
            unwrapped_ms=int(self.unwrapped_ms),
            delta_ms=int(delta),
            reset=reset,
            wrapped=wrapped,
        )


class MonotonicMinimumDelayMapper:
    """Map a device clock into host time while following slow clock drift.

    Serial receipt time contains a non-negative and variable transport delay.
    Only an earlier offset observation is therefore allowed to move the device
    timeline, and each adjustment is bounded so published stamps stay smooth.
    """

    def __init__(self, max_adjustment_ns: int = 100_000):
        if max_adjustment_ns <= 0:
            raise ValueError("max_adjustment_ns must be positive")
        self.max_adjustment_ns = int(max_adjustment_ns)
        self.offset_ns: Optional[int] = None
        self.last_mapped_ns: Optional[int] = None
        self.total_adjustment_ns = 0

    def reset(self) -> None:
        self.offset_ns = None
        self.last_mapped_ns = None
        self.total_adjustment_ns = 0

    def map_ms(self, device_ms: int, receipt_ns: int, wire_ns: int = 0) -> int:
        device_ns = int(device_ms) * 1_000_000
        candidate_offset_ns = int(receipt_ns) - device_ns - int(wire_ns)

        if self.offset_ns is None:
            self.offset_ns = candidate_offset_ns
        elif candidate_offset_ns < self.offset_ns:
            adjustment = min(
                self.offset_ns - candidate_offset_ns,
                self.max_adjustment_ns,
            )
            self.offset_ns -= adjustment
            self.total_adjustment_ns += adjustment

        mapped_ns = self.offset_ns + device_ns
        if self.last_mapped_ns is not None and mapped_ns <= self.last_mapped_ns:
            mapped_ns = self.last_mapped_ns + 1
        self.last_mapped_ns = mapped_ns
        return mapped_ns


class AdaptiveMinimumDelayMapper:
    """Map a device tick to host time while tracking slow oscillator drift.

    Receipt timestamps contain non-negative scheduling and serial delay.  The
    minimum offset in a recent window is therefore a robust clock-offset
    estimate.  Slew limiting lets that estimate move in either direction
    without making ROS timestamps jump when the MCU and host oscillators drift.
    """

    def __init__(self, window_size: int = 250, max_adjustment_ns: int = 20_000):
        if window_size < 2:
            raise ValueError("window_size must be at least 2")
        if max_adjustment_ns <= 0:
            raise ValueError("max_adjustment_ns must be positive")
        self.window_size = int(window_size)
        self.max_adjustment_ns = int(max_adjustment_ns)
        self.candidates: Deque[int] = deque(maxlen=self.window_size)
        self.offset_ns: Optional[int] = None
        self.last_mapped_ns: Optional[int] = None

    def reset(self) -> None:
        self.candidates.clear()
        self.offset_ns = None
        self.last_mapped_ns = None

    def map_ms(self, device_ms: int, receipt_ns: int, wire_ns: int = 0) -> int:
        device_ns = int(device_ms) * 1_000_000
        candidate = int(receipt_ns) - device_ns - int(wire_ns)
        self.candidates.append(candidate)
        target = min(self.candidates)

        if self.offset_ns is None:
            self.offset_ns = target
        else:
            error = target - self.offset_ns
            adjustment = max(
                -self.max_adjustment_ns,
                min(self.max_adjustment_ns, error),
            )
            self.offset_ns += adjustment

        mapped_ns = self.offset_ns + device_ns
        # The sample cannot be newer than the end of its serial transmission.
        mapped_ns = min(mapped_ns, int(receipt_ns) - int(wire_ns))
        if self.last_mapped_ns is not None and mapped_ns <= self.last_mapped_ns:
            mapped_ns = self.last_mapped_ns + 1
        self.last_mapped_ns = mapped_ns
        return mapped_ns
