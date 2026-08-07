"""Pure pose-correction detection used by the ROS guard node."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Deque, Optional


def wrap_angle(angle: float) -> float:
    """Wrap an angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Pose2D:
    """A planar map-to-odom sample on a monotonic time base."""

    time_sec: float
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class PoseCorrection:
    """Measured discontinuity which exceeded an instant or window limit."""

    instant_translation: float
    instant_yaw: float
    window_translation: float
    window_yaw: float


class SlamCorrectionDetector:
    """Detect abrupt and short-window changes in map-to-odom."""

    def __init__(
        self,
        translation_threshold: float,
        yaw_threshold: float,
        window_sec: float,
        window_translation_threshold: float,
        window_yaw_threshold: float,
        max_sample_gap_sec: float,
    ) -> None:
        self.translation_threshold = max(0.0, translation_threshold)
        self.yaw_threshold = max(0.0, yaw_threshold)
        self.window_sec = max(0.01, window_sec)
        self.window_translation_threshold = max(
            self.translation_threshold,
            window_translation_threshold,
        )
        self.window_yaw_threshold = max(
            self.yaw_threshold,
            window_yaw_threshold,
        )
        self.max_sample_gap_sec = max(
            self.window_sec,
            max_sample_gap_sec,
        )
        self.samples: Deque[Pose2D] = deque()

    def reset(self, sample: Optional[Pose2D] = None) -> None:
        """Forget prior motion while optionally retaining a new baseline."""
        self.samples.clear()
        if sample is not None:
            self.samples.append(sample)

    @staticmethod
    def _delta(first: Pose2D, second: Pose2D) -> tuple[float, float]:
        translation = math.hypot(second.x - first.x, second.y - first.y)
        yaw = abs(wrap_angle(second.yaw - first.yaw))
        return translation, yaw

    def update(self, sample: Pose2D) -> Optional[PoseCorrection]:
        """Add one sample and return a correction when a limit is crossed."""
        if not self.samples:
            self.samples.append(sample)
            return None

        previous = self.samples[-1]
        gap = sample.time_sec - previous.time_sec
        if gap <= 0.0 or gap > self.max_sample_gap_sec:
            self.reset(sample)
            return None

        instant_translation, instant_yaw = self._delta(previous, sample)
        self.samples.append(sample)
        cutoff = sample.time_sec - self.window_sec
        while len(self.samples) > 1 and self.samples[1].time_sec <= cutoff:
            self.samples.popleft()

        window_translation, window_yaw = self._delta(
            self.samples[0], sample)
        instant_trigger = (
            instant_translation >= self.translation_threshold
            or instant_yaw >= self.yaw_threshold
        )
        window_trigger = (
            window_translation >= self.window_translation_threshold
            or window_yaw >= self.window_yaw_threshold
        )
        if not instant_trigger and not window_trigger:
            return None

        correction = PoseCorrection(
            instant_translation=instant_translation,
            instant_yaw=instant_yaw,
            window_translation=window_translation,
            window_yaw=window_yaw,
        )
        self.reset(sample)
        return correction
