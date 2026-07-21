"""Pure control helpers for Nav2 and depth-obstacle velocity fusion."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import List, Optional, Tuple


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def slew(current: float, target: float, max_delta: float) -> float:
    if target > current + max_delta:
        return current + max_delta
    if target < current - max_delta:
        return current - max_delta
    return target


@dataclass
class DepthSample:
    level: int = 0
    preferred_dir: int = 0
    nearest_mm: int = 9999
    center_danger: bool = False
    center_far: bool = False
    left_blocked: bool = False
    right_blocked: bool = False
    center_area_x1000: int = 0
    total_area_x1000: int = 0
    left_score_x1000: int = 0
    right_score_x1000: int = 0
    center_min_mm: int = 9999

    @property
    def has_obstacle(self) -> bool:
        return bool(
            self.level > 0
            or self.center_danger
            or self.center_far
            or self.left_blocked
            or self.right_blocked
        )

    def effective_nearest(self) -> int:
        candidates = [
            value
            for value in (self.nearest_mm, self.center_min_mm)
            if 0 < int(value) < 9999
        ]
        return min(candidates) if candidates else 9999


@dataclass
class FusionConfig:
    max_v: float = 0.23
    max_w: float = 0.80
    warning_distance_mm: int = 950
    danger_distance_mm: int = 450
    critical_distance_mm: int = 300
    level_release_hold_sec: float = 0.40
    direction_switch_margin: int = 80
    direction_switch_frames: int = 6
    forward_threshold: float = 0.015
    warning_w_min: float = 0.05
    warning_w_max: float = 0.20
    danger_w_min: float = 0.12
    danger_w_max: float = 0.32
    avoid_w_slew_rate: float = 2.2
    return_w_slew_rate: float = 1.4


@dataclass
class FusionResult:
    linear_x: float
    angular_z: float
    mode: str
    level: int
    risk: float
    direction: int


class FusionController:
    """Stateful, deterministic arbiter that is independent of ROS."""

    def __init__(self, config: Optional[FusionConfig] = None):
        self.config = config or FusionConfig()
        self.stable_level = 0
        self.release_since: Optional[float] = None
        self.avoid_direction = 1
        self.direction_candidate = 0
        self.direction_candidate_count = 0
        self.bias_w = 0.0
        self.risk_ema = 0.0
        self.last_obstacle_sample = DepthSample()

    def reset(self) -> None:
        self.stable_level = 0
        self.release_since = None
        self.direction_candidate = 0
        self.direction_candidate_count = 0
        self.bias_w = 0.0
        self.risk_ema = 0.0
        self.last_obstacle_sample = DepthSample()

    def _update_level(self, sample: DepthSample, now: float) -> int:
        raw_level = int(clamp(sample.level, 0, 2))
        if raw_level >= self.stable_level:
            self.stable_level = raw_level
            self.release_since = None
        else:
            if self.release_since is None:
                self.release_since = now
            elif now - self.release_since >= self.config.level_release_hold_sec:
                self.stable_level = raw_level
                self.release_since = None

        if sample.has_obstacle:
            self.last_obstacle_sample = sample
        elif self.stable_level == 0:
            self.last_obstacle_sample = DepthSample()
        return self.stable_level

    @staticmethod
    def _raw_direction(sample: DepthSample, nav_w: float) -> Tuple[int, int]:
        score_delta = sample.left_score_x1000 - sample.right_score_x1000
        if abs(score_delta) >= 18:
            return (-1 if score_delta > 0 else 1), abs(score_delta)
        if sample.preferred_dir in (-1, 1):
            # Vision uses -1=prefer left and +1=prefer right. ROS angular.z
            # uses the opposite sign for that encoded direction.
            return -sample.preferred_dir, abs(score_delta)
        if sample.left_blocked and not sample.right_blocked:
            return -1, 1000
        if sample.right_blocked and not sample.left_blocked:
            return 1, 1000
        if abs(nav_w) > 0.05:
            return (1 if nav_w > 0.0 else -1), 0
        return 0, 0

    def _update_direction(self, sample: DepthSample, nav_w: float) -> int:
        candidate, strength = self._raw_direction(sample, nav_w)
        if candidate == 0:
            return self.avoid_direction

        if self.stable_level == 0:
            self.avoid_direction = candidate
            self.direction_candidate = 0
            self.direction_candidate_count = 0
            return self.avoid_direction

        if candidate == self.avoid_direction:
            self.direction_candidate = 0
            self.direction_candidate_count = 0
            return self.avoid_direction

        if strength < self.config.direction_switch_margin:
            return self.avoid_direction

        if candidate != self.direction_candidate:
            self.direction_candidate = candidate
            self.direction_candidate_count = 1
        else:
            self.direction_candidate_count += 1

        if self.direction_candidate_count >= self.config.direction_switch_frames:
            self.avoid_direction = candidate
            self.direction_candidate = 0
            self.direction_candidate_count = 0
        return self.avoid_direction

    def _distance_risk(self, nearest_mm: int) -> float:
        if nearest_mm <= 0 or nearest_mm >= 9999:
            return 0.25
        near = float(max(180, self.config.critical_distance_mm))
        far = float(max(
            self.config.warning_distance_mm,
            self.config.danger_distance_mm + 100,
        ))
        return clamp((far - float(nearest_mm)) / max(1.0, far - near), 0.0, 1.0)

    @staticmethod
    def _width_risk(sample: DepthSample) -> float:
        blocked = int(sample.left_blocked) + int(sample.right_blocked)
        blocked_risk = 0.55 if blocked == 2 else (0.28 if blocked == 1 else 0.0)
        center_area = clamp(float(sample.center_area_x1000) / 35.0, 0.0, 1.0)
        total_area = clamp(float(sample.total_area_x1000) / 65.0, 0.0, 1.0)
        side_score = clamp(
            float(max(sample.left_score_x1000, sample.right_score_x1000)) / 150.0,
            0.0,
            1.0,
        )
        return clamp(max(
            blocked_risk,
            0.45 * center_area + 0.30 * total_area + 0.25 * side_score,
        ), 0.0, 1.0)

    def update(
        self,
        nav_v: float,
        nav_w: float,
        sample: DepthSample,
        now: float,
        dt: float,
        depth_alive: bool = True,
    ) -> FusionResult:
        cfg = self.config
        nav_v = clamp(nav_v, -cfg.max_v, cfg.max_v)
        nav_w = clamp(nav_w, -cfg.max_w, cfg.max_w)
        if not depth_alive:
            sample = DepthSample()

        previous_level = self.stable_level
        level = self._update_level(sample, now)
        active_sample = sample if sample.has_obstacle else self.last_obstacle_sample
        if previous_level == 0 and level > 0:
            initial_direction, _ = self._raw_direction(active_sample, nav_w)
            if initial_direction in (-1, 1):
                self.avoid_direction = initial_direction
                self.direction_candidate = 0
                self.direction_candidate_count = 0
        direction = self._update_direction(active_sample, nav_w)
        distance_risk = self._distance_risk(active_sample.effective_nearest())
        width_risk = self._width_risk(active_sample)
        raw_risk = max(distance_risk, width_risk) if level > 0 else 0.0

        # A short time-domain filter removes risk jumps while preserving an
        # immediate rise for genuinely close obstacles.
        if raw_risk >= self.risk_ema:
            alpha = 0.65
        else:
            alpha = clamp(dt / 0.30, 0.0, 1.0)
        self.risk_ema += alpha * (raw_risk - self.risk_ema)
        risk = clamp(self.risk_ema, 0.0, 1.0)

        target_v = nav_v
        target_bias = 0.0
        mode = "clear"

        # The front camera must not invent a rotation while Nav2 is stopped,
        # aligning to the path, or executing a recovery spin. The virtual
        # depth scan gives Nav2 collision information for those maneuvers.
        moving_forward = nav_v > cfg.forward_threshold
        critical_close = (
            level >= 2
            and 0 < active_sample.effective_nearest() <= cfg.critical_distance_mm
        )
        corridor_closed = (
            level >= 2
            and active_sample.left_blocked
            and active_sample.right_blocked
        )

        if moving_forward and (critical_close or corridor_closed):
            target_v = 0.0
            target_bias = 0.0
            mode = "blocked"
        elif moving_forward and level >= 2:
            speed_scale = clamp(0.55 - 0.32 * risk - 0.10 * width_risk, 0.12, 0.55)
            target_v = nav_v * speed_scale
            desired_w = cfg.danger_w_min + (
                cfg.danger_w_max - cfg.danger_w_min
            ) * max(risk, width_risk)
            target_bias = self._minimum_turn_bias(nav_w, direction, desired_w)
            mode = "avoid"
        elif moving_forward and level == 1:
            speed_scale = clamp(0.88 - 0.38 * risk - 0.12 * width_risk, 0.42, 0.88)
            target_v = nav_v * speed_scale
            desired_w = cfg.warning_w_min + (
                cfg.warning_w_max - cfg.warning_w_min
            ) * max(risk, width_risk)
            target_bias = self._minimum_turn_bias(nav_w, direction, desired_w)
            mode = "caution"

        slew_rate = (
            cfg.avoid_w_slew_rate
            if abs(target_bias) > abs(self.bias_w)
            else cfg.return_w_slew_rate
        )
        if mode == "blocked":
            self.bias_w = 0.0
        else:
            self.bias_w = slew(
                self.bias_w,
                target_bias,
                max(0.0, slew_rate * clamp(dt, 0.0, 0.2)),
            )

        # Never raise a slow Nav2 command to a fixed minimum. Output speed is
        # always less than or equal to the source command during avoidance.
        out_v = clamp(target_v, -cfg.max_v, cfg.max_v)
        if mode == "blocked":
            out_w = 0.0
        elif not moving_forward:
            out_w = nav_w
        else:
            out_w = clamp(nav_w + self.bias_w, -cfg.max_w, cfg.max_w)
        return FusionResult(out_v, out_w, mode, level, risk, direction)

    @staticmethod
    def _minimum_turn_bias(nav_w: float, direction: int, desired_w: float) -> float:
        if direction not in (-1, 1):
            return 0.0
        signed_desired = float(direction) * desired_w
        if nav_w * direction >= 0.0:
            # Nav2 is already turning toward the safe side. Establish a
            # minimum curvature instead of stacking another full turn rate.
            return signed_desired - nav_w if abs(nav_w) < desired_w else 0.0
        # If Nav2 is still turning toward the newly observed obstacle, request
        # a safe-side total turn instead of adding two opposing controllers.
        return signed_desired - nav_w


def build_virtual_scan_ranges(
    sample: DepthSample,
    stable_level: int,
    ray_count: int = 61,
    angle_min: float = -math.pi / 3.0,
    angle_max: float = math.pi / 3.0,
    min_range_m: float = 0.18,
    max_range_m: float = 1.30,
    origin_offset_m: float = 0.30,
) -> List[float]:
    """Build a coarse forward LaserScan for Nav2 obstacle layers."""
    ranges = [math.inf] * ray_count
    if ray_count < 2 or not sample.has_obstacle:
        return ranges

    increment = (angle_max - angle_min) / float(ray_count - 1)
    nearest = sample.effective_nearest()
    distance = (
        max_range_m
        if nearest <= 0 or nearest >= 9999
        else clamp(
            float(nearest) / 1000.0 + origin_offset_m,
            min_range_m,
            max_range_m,
        )
    )

    def mark(start_deg: float, end_deg: float, value: float) -> None:
        for index in range(ray_count):
            angle_deg = math.degrees(angle_min + index * increment)
            if start_deg <= angle_deg <= end_deg:
                ranges[index] = min(ranges[index], value)

    if stable_level > 0 or sample.center_danger or sample.center_far:
        center_nearest = sample.center_min_mm
        center_distance = (
            distance
            if center_nearest <= 0 or center_nearest >= 9999
            else clamp(
                float(center_nearest) / 1000.0 + origin_offset_m,
                min_range_m,
                max_range_m,
            )
        )
        mark(-22.0, 22.0, center_distance)
    if sample.left_blocked:
        mark(18.0, 52.0, distance)
    if sample.right_blocked:
        mark(-52.0, -18.0, distance)
    return ranges
