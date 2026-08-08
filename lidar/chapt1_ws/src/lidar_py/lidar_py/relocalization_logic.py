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
    ) -> None:
        self.required_count = max(2, int(required_count))
        self.translation_m = max(0.01, float(translation_m))
        self.yaw_rad = max(math.radians(1.0), float(yaw_rad))
        self.reset()

    def reset(self) -> None:
        """Discard all accumulated pose evidence."""

        self.count = 0
        self.last_stamp_ns: int | None = None
        self.x_sum = 0.0
        self.y_sum = 0.0
        self.sin_sum = 0.0
        self.cos_sum = 0.0

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
        self, x: float, y: float, yaw: float, stamp_ns: int
    ) -> ConsensusResult:
        """Add one pose only when its scan timestamp and cluster are valid."""

        stamp_ns = int(stamp_ns)
        if self.last_stamp_ns == stamp_ns:
            return ConsensusResult(
                self.count,
                self.count >= self.required_count,
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

        self.count += 1
        self.last_stamp_ns = stamp_ns
        self.x_sum += float(x)
        self.y_sum += float(y)
        self.sin_sum += math.sin(float(yaw))
        self.cos_sum += math.cos(float(yaw))
        return ConsensusResult(
            self.count,
            self.count >= self.required_count,
            reset,
            False,
            self.pose,
        )
