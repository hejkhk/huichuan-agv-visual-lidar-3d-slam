from __future__ import annotations

from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Generic, TypeVar

T = TypeVar("T")


@dataclass(slots=True)
class ApiResult(Generic[T]):
    """Serializable success/failure envelope used by every public API method.

    ``message`` is safe to show to the operator. ``error_code`` is the stable
    machine-readable identifier used by logs/tests. A failed result never
    carries data; a successful list operation should carry an empty list
    instead of ``None``.
    """

    success: bool
    message: str = ""
    data: T | None = None
    error_code: str = ""

    @classmethod
    def ok(cls, data: T | None = None, message: str = "操作成功") -> "ApiResult[T]":
        return cls(True, message, data)

    @classmethod
    def fail(cls, message: str, error_code: str = "API_ERROR") -> "ApiResult[T]":
        return cls(False, message, None, error_code)


class NavigationState(str, Enum):
    IDLE = "IDLE"
    TARGET_SELECTED = "TARGET_SELECTED"
    STARTING = "STARTING"
    NAVIGATING = "NAVIGATING"
    PAUSED = "PAUSED"
    ARRIVED = "ARRIVED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(slots=True)
class Pose:
    """Map-frame pose in metres and radians."""

    x: float = 6.46
    y: float = 65.64
    yaw: float = 0.0


@dataclass(slots=True)
class RobotSnapshot:
    """Complete UI-facing state cache.

    This dataclass is the only supported way for integrations to publish
    status to QML. Fields are copied by ``UiBackend._poll_snapshot`` and must
    remain cheap to read. Linear velocity is m/s, angular velocity is rad/s,
    acceleration is g, gyroscope values are degrees/s, and map coordinates
    are metres. See ``INTERFACE_CONTRACT.md`` before adding fields.
    """

    timestamp: float = 0.0
    battery_percent: int = 0
    remaining_range_km: float = 0.0
    upload_kbps: float = 0.0
    download_kbps: float = 0.0
    network_connected: bool = False
    bluetooth_connected: bool = False
    system_status: str = "未连接"
    navigation_state: str = NavigationState.IDLE.value
    navigation_target: str = ""
    navigation_progress: int = 0
    navigation_message: str = "请选择目标点"
    navigation_pause_supported: bool = True
    voice_control_enabled: bool = True
    visual_follow_enabled: bool = True
    charging: bool = False
    cpu_percent: float = 18.0
    memory_percent: float = 43.0
    cpu_temperature: float = 52.0
    encoder_status: str = "UNKNOWN"
    lidar_status: str = "UNKNOWN"
    voice_module_status: str = "UNKNOWN"
    battery_voltage: float = 0.0
    charging_status: str = "未在充电"
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0
    ax: float = 0.0
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0
    gy: float = 0.0
    gz: float = 0.0
    current_pose: Pose = field(default_factory=Pose)
    pose_available: bool = False
    ros_connected: bool = False
    ros_error: str = ""
    map_available: bool = False
    map_image: str = ""
    map_width: int = 0
    map_height: int = 0
    map_resolution: float = 0.0
    map_origin_x: float = 0.0
    map_origin_y: float = 0.0
    map_revision: int = 0
    mapping_state: str = "IDLE"
    mapping_active: bool = False
    mapping_message: str = ""
    slam_running: bool = False
    slam_mode: str = "stopped"
    slam_message: str = "SLAM 系统未运行"
    localization_ready: bool = False
    localization_state: str = "inactive"
    localization_detail: str = ""
    recovery_stage: str = "tracking"
    recovery_reason: str = ""
    recovery_count: int = 0
    laser_points: list[list[float]] = field(default_factory=list)
    path_points: list[list[float]] = field(default_factory=list)
    detected_actors: list[dict[str, Any]] = field(default_factory=list)
    follow_state: str = "IDLE"
    follow_target: str = ""
    # Voice UI contract:
    # LISTENING -> 倾听中, SPEAKING -> 播报中, READY -> 您请说.
    voice_state: str = "LISTENING"
    speaker_name: str = "声纹3"
    speaker_voiceprint: str = "声纹3"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)
