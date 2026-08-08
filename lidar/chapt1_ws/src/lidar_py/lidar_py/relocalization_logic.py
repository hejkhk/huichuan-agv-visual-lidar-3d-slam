"""Pure helpers for deterministic Cartographer startup relocalization."""

from dataclasses import dataclass
import math
import struct
import zlib


Candidate = tuple[float, int, int, float]


class ImmutableCrcLock:
    """Accept the first CRC and only identical transient-local republishes."""

    def __init__(self) -> None:
        self.crc: int | None = None

    def accept(self, crc: int) -> bool:
        """Lock the first value and reject every later mutation."""

        crc = int(crc) & 0xFFFFFFFF
        if self.crc is None:
            self.crc = crc
            return True
        return crc == self.crc


def wrap_angle(angle: float) -> float:
    """Wrap one angle to [-pi, pi]."""

    return math.atan2(math.sin(angle), math.cos(angle))


def occupancy_grid_crc(
    width: int,
    height: int,
    resolution: float,
    origin: tuple[float, float, float, float, float, float, float],
    data,
) -> int:
    """Return a CRC covering immutable OccupancyGrid metadata and cells."""

    header = struct.pack(
        "<II8d",
        int(width),
        int(height),
        float(resolution),
        *(float(value) for value in origin),
    )
    cells = bytes((int(value) & 0xFF) for value in data)
    return zlib.crc32(cells, zlib.crc32(header)) & 0xFFFFFFFF


def candidates_share_cluster(
    first: Candidate,
    second: Candidate,
    resolution: float,
    translation_m: float,
    yaw_rad: float,
) -> bool:
    """Return whether two scan-match candidates describe one pose basin."""

    dx = (first[2] - second[2]) * resolution
    dy = (first[1] - second[1]) * resolution
    dyaw = abs(wrap_angle(first[3] - second[3]))
    return math.hypot(dx, dy) <= translation_m and dyaw <= yaw_rad


def select_distinct_candidates(
    candidates: list[Candidate],
    resolution: float,
    translation_m: float,
    yaw_rad: float,
    limit: int,
) -> list[Candidate]:
    """Keep the strongest candidate from each spatial/yaw cluster."""

    selected: list[Candidate] = []
    for candidate in sorted(candidates, reverse=True, key=lambda item: item[0]):
        if any(
            candidates_share_cluster(
                candidate, existing, resolution, translation_m, yaw_rad
            )
            for existing in selected
        ):
            continue
        selected.append(candidate)
        if len(selected) >= limit:
            break
    return selected


def refine_distinct_candidates(
    coarse_candidates: list[Candidate],
    refine,
    resolution: float,
    translation_m: float,
    yaw_rad: float,
    limit: int,
) -> list[Candidate]:
    """Refine multiple independent basins before ranking best and second."""

    seeds = select_distinct_candidates(
        coarse_candidates,
        resolution,
        translation_m,
        yaw_rad,
        limit,
    )
    refined = [refine(seed) for seed in seeds]
    return select_distinct_candidates(
        refined,
        resolution,
        translation_m,
        yaw_rad,
        limit,
    )


@dataclass(frozen=True)
class ConsensusResult:
    """Result of observing one timestamped scan-match pose."""

    count: int
    ready: bool
    reset: bool
    duplicate: bool
    pose: tuple[float, float, float] | None


class PoseConsensus:
    """Require several distinct scans to agree before trajectory restart."""

    def __init__(
        self,
        required_count: int,
        translation_m: float,
        yaw_rad: float,
        extended_required_count: int | None = None,
        allow_extended: bool = False,
    ) -> None:
        self.required_count = max(2, int(required_count))
        self.extended_required_count = max(
            self.required_count,
            int(extended_required_count or self.required_count),
        )
        self.translation_m = max(0.01, float(translation_m))
        self.yaw_rad = max(math.radians(1.0), float(yaw_rad))
        self.allow_extended = bool(allow_extended)
        self.reset()

    def reset(self) -> None:
        """Discard all accumulated pose evidence."""

        self.count = 0
        self.last_stamp_ns: int | None = None
        self.x_sum = 0.0
        self.y_sum = 0.0
        self.sin_sum = 0.0
        self.cos_sum = 0.0
        self.requires_extended = False

    @property
    def active_required_count(self) -> int:
        """Return the evidence count required by the current pose cluster."""

        if self.requires_extended:
            return self.extended_required_count
        return self.required_count

    @property
    def ready(self) -> bool:
        """Return true only for a sufficiently distinct pose cluster."""

        enough = self.count >= self.active_required_count
        return enough and (self.allow_extended or not self.requires_extended)

    @property
    def pose(self) -> tuple[float, float, float] | None:
        """Return the circular mean of the accepted pose cluster."""

        if self.count <= 0:
            return None
        return (
            self.x_sum / self.count,
            self.y_sum / self.count,
            math.atan2(self.sin_sum, self.cos_sum),
        )

    def observe(
        self,
        x: float,
        y: float,
        yaw: float,
        stamp_ns: int,
        *,
        extended: bool = False,
    ) -> ConsensusResult:
        """Add one pose only when its scan timestamp and cluster are valid."""

        stamp_ns = int(stamp_ns)
        if self.last_stamp_ns == stamp_ns:
            return ConsensusResult(
                self.count,
                self.ready,
                False,
                True,
                self.pose,
            )

        current = self.pose
        reset = False
        if current is not None:
            distance = math.hypot(x - current[0], y - current[1])
            yaw_error = abs(wrap_angle(yaw - current[2]))
            if distance > self.translation_m or yaw_error > self.yaw_rad:
                self.reset()
                reset = True

        self.requires_extended = self.requires_extended or bool(extended)
        self.count += 1
        self.last_stamp_ns = stamp_ns
        self.x_sum += float(x)
        self.y_sum += float(y)
        self.sin_sum += math.sin(float(yaw))
        self.cos_sum += math.cos(float(yaw))
        return ConsensusResult(
            self.count,
            self.ready,
            reset,
            False,
            self.pose,
        )


@dataclass(frozen=True)
class BootstrapGateResult:
    """Decision from one Cartographer bootstrap pose observation."""

    count: int
    ready: bool
    reset: bool
    duplicate: bool
    duration_sec: float
    pose: tuple[float, float, float] | None


class BootstrapPoseGate:
    """Require a fresh, high-score, stable Cartographer pose over time."""

    def __init__(
        self,
        min_score: float,
        hold_sec: float,
        min_observations: int,
        translation_m: float,
        yaw_rad: float,
    ) -> None:
        self.min_score = float(min_score)
        self.hold_sec = max(0.1, float(hold_sec))
        self.min_observations = max(2, int(min_observations))
        self.translation_m = max(0.01, float(translation_m))
        self.yaw_rad = max(math.radians(1.0), float(yaw_rad))
        self.reset()

    def reset(self) -> None:
        """Discard accumulated bootstrap evidence."""

        self.count = 0
        self.first_time_sec: float | None = None
        self.last_stamp_ns: int | None = None
        self.x_sum = 0.0
        self.y_sum = 0.0
        self.sin_sum = 0.0
        self.cos_sum = 0.0

    @property
    def pose(self) -> tuple[float, float, float] | None:
        """Return the mean pose of the current stable interval."""

        if self.count <= 0:
            return None
        return (
            self.x_sum / self.count,
            self.y_sum / self.count,
            math.atan2(self.sin_sum, self.cos_sum),
        )

    def observe(
        self,
        x: float,
        y: float,
        yaw: float,
        stamp_ns: int,
        now_sec: float,
        score: float,
    ) -> BootstrapGateResult:
        """Accept only distinct TF samples that remain stable and well matched."""

        stamp_ns = int(stamp_ns)
        now_sec = float(now_sec)
        if score < self.min_score:
            had_evidence = self.count > 0
            self.reset()
            return BootstrapGateResult(0, False, had_evidence, False, 0.0, None)
        if self.last_stamp_ns == stamp_ns:
            duration = (
                now_sec - self.first_time_sec
                if self.first_time_sec is not None else 0.0
            )
            return BootstrapGateResult(
                self.count, False, False, True, duration, self.pose)

        current = self.pose
        reset = False
        if current is not None:
            distance = math.hypot(x - current[0], y - current[1])
            yaw_error = abs(wrap_angle(yaw - current[2]))
            if distance > self.translation_m or yaw_error > self.yaw_rad:
                self.reset()
                reset = True

        if self.first_time_sec is None:
            self.first_time_sec = now_sec
        self.count += 1
        self.last_stamp_ns = stamp_ns
        self.x_sum += float(x)
        self.y_sum += float(y)
        self.sin_sum += math.sin(float(yaw))
        self.cos_sum += math.cos(float(yaw))
        duration = max(0.0, now_sec - self.first_time_sec)
        ready = self.count >= self.min_observations and duration >= self.hold_sec
        return BootstrapGateResult(
            self.count, ready, reset, False, duration, self.pose)
