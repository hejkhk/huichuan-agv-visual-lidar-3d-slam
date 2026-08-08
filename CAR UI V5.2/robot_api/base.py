from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

from .types import ApiResult, RobotSnapshot


class RobotApiBase(ABC):
    """Stable public boundary between Qt and robot/device integrations.

    Implementations may talk to ROS 2, serial devices, SDKs, or local mock
    storage, but callers must see the same typed methods on every platform.
    Every method returns :class:`ApiResult`; expected device or validation
    failures must not escape as exceptions into ``UiBackend``.

    Threading contract:
      * ``get_robot_snapshot`` should return a cached snapshot quickly.
      * Mutating calls may block because ``UiBackend`` dispatches them through
        ``QThreadPool``; implementations must still protect shared caches.
      * Values stored in ``RobotSnapshot`` use the units documented in
        ``INTERFACE_CONTRACT.md``.

    Compatibility contract:
      * Do not rename methods or change their positional parameters.
      * Unsupported features return a failed ``ApiResult`` with
        ``NOT_IMPLEMENTED`` or ``NOT_SUPPORTED``.
      * Public dictionaries must remain JSON/QVariant serializable.
    """

    # Snapshot and localization -------------------------------------------------
    @abstractmethod
    def get_robot_snapshot(self) -> ApiResult[RobotSnapshot]: """Return the latest cached robot state."""
    @abstractmethod
    def get_current_pose(self) -> ApiResult[dict[str, float]]: """Return the current map pose."""

    # Navigation ---------------------------------------------------------------
    @abstractmethod
    def start_single_navigation(self, point_id: str) -> ApiResult[None]: """Start navigation to one point."""
    @abstractmethod
    def start_pose_navigation(self, x: float, y: float, yaw: float = 0.0) -> ApiResult[None]: """Start navigation to an unsaved map pose."""
    @abstractmethod
    def start_route_navigation(self, point_ids: list[str], ordered: bool = True) -> ApiResult[None]: """Start multi-point navigation."""
    @abstractmethod
    def pause_navigation(self) -> ApiResult[None]: """Pause navigation."""
    @abstractmethod
    def resume_navigation(self) -> ApiResult[None]: """Resume navigation."""
    @abstractmethod
    def cancel_navigation(self) -> ApiResult[None]: """Cancel navigation."""
    @abstractmethod
    def start_slam_navigation(self) -> ApiResult[None]: """Start the full SLAM/Nav2 stack."""
    @abstractmethod
    def stop_slam_system(self) -> ApiResult[None]: """Stop the registered SLAM stack."""

    # Map/RViz-compatible presentation commands ------------------------------
    @abstractmethod
    def rviz_zoom_in(self) -> ApiResult[None]: """Request RViz zoom in."""
    @abstractmethod
    def rviz_zoom_out(self) -> ApiResult[None]: """Request RViz zoom out."""
    @abstractmethod
    def rviz_reset_view(self) -> ApiResult[None]: """Reset the RViz camera."""
    @abstractmethod
    def open_rviz_fullscreen(self) -> ApiResult[None]: """Record opening of fullscreen RViz."""

    # Mapping lifecycle --------------------------------------------------------
    @abstractmethod
    def start_mapping(self) -> ApiResult[None]: """Begin building a new map."""
    @abstractmethod
    def stop_mapping(self) -> ApiResult[None]: """Stop the mapping session."""
    @abstractmethod
    def save_map(self, name: str) -> ApiResult[dict[str, Any]]: """Save the current map."""
    @abstractmethod
    def load_map(self, yaml_path: str) -> ApiResult[dict[str, Any]]: """Load one main-directory YAML map."""

    # Saved points and charging ------------------------------------------------
    @abstractmethod
    def list_points(self) -> ApiResult[list[dict[str, Any]]]: """List saved points."""
    @abstractmethod
    def save_point(self, name: str, x: float, y: float, yaw: float, is_charging_point: bool = False) -> ApiResult[dict[str, Any]]: """Persist a point."""
    @abstractmethod
    def rename_point(self, point_id: str, new_name: str) -> ApiResult[None]: """Rename a point."""
    @abstractmethod
    def update_point_yaw(self, point_id: str, yaw: float) -> ApiResult[None]:
        """Persist a saved point's final heading in map-frame radians."""
    @abstractmethod
    def delete_point(self, point_id: str) -> ApiResult[None]: """Delete a point."""
    @abstractmethod
    def set_charging_point(self, point_id: str) -> ApiResult[None]: """Make one point the sole charging point."""
    @abstractmethod
    def get_charging_point(self) -> ApiResult[dict[str, Any]]: """Return the charging point."""
    @abstractmethod
    def start_charging(self) -> ApiResult[None]: """Start return-to-charge."""
    @abstractmethod
    def cancel_charging(self) -> ApiResult[None]: """Cancel charging navigation."""

    # Voice and visual-follow controls ----------------------------------------
    @abstractmethod
    def set_voice_control_enabled(self, enabled: bool) -> ApiResult[None]: """Enable or disable voice control."""
    @abstractmethod
    def set_visual_follow_enabled(self, enabled: bool) -> ApiResult[None]: """Enable or disable visual following."""
    @abstractmethod
    def set_unknown_voice_control_allowed(self, enabled: bool) -> ApiResult[None]: """Set unknown-voice permission."""
    @abstractmethod
    def list_detected_actors(self) -> ApiResult[list[dict[str, Any]]]: """List camera detections."""
    @abstractmethod
    def select_follow_target(self, actor_id: str) -> ApiResult[None]: """Select an actor."""
    @abstractmethod
    def start_following(self, actor_id: str) -> ApiResult[None]: """Start following an actor."""
    @abstractmethod
    def stop_following(self) -> ApiResult[None]: """Stop following."""

    # Manual gamepad hand-off -------------------------------------------------
    @abstractmethod
    def release_control_to_gamepad(self) -> ApiResult[None]:
        """Send the one-way command that releases host control to the gamepad.

        The gamepad is connected to the lower controller. Implementations must
        only transmit the hand-off command; no acknowledgement or ownership
        feedback is required by this interface.
        """

    # Voiceprint lifecycle -----------------------------------------------------
    @abstractmethod
    def list_voiceprints(self) -> ApiResult[list[dict[str, Any]]]: """List saved voiceprints."""
    @abstractmethod
    def begin_voiceprint_recording(self, name: str) -> ApiResult[None]: """Start voiceprint capture."""
    @abstractmethod
    def cancel_voiceprint_recording(self) -> ApiResult[None]: """Cancel voiceprint capture."""
    @abstractmethod
    def save_voiceprint(self, name: str) -> ApiResult[dict[str, Any]]: """Save a recorded voiceprint."""
    @abstractmethod
    def rename_voiceprint(self, voiceprint_id: str, new_name: str) -> ApiResult[None]: """Rename a voiceprint."""
    @abstractmethod
    def delete_voiceprint(self, voiceprint_id: str) -> ApiResult[None]: """Delete a voiceprint."""
    @abstractmethod
    def move_voiceprint(self, voiceprint_id: str, direction: int) -> ApiResult[None]: """Move a voiceprint one priority position; direction is -1 or 1."""

    # User settings and system services ---------------------------------------
    @abstractmethod
    def get_settings(self) -> ApiResult[dict[str, Any]]: """Return user settings."""
    @abstractmethod
    def set_volume(self, value: int) -> ApiResult[None]: """Set output volume."""
    @abstractmethod
    def set_parameter(self, name: str, value: Any) -> ApiResult[None]: """Set one robot parameter."""
    @abstractmethod
    def list_wifi_networks(self) -> ApiResult[list[dict[str, Any]]]: """List visible Wi-Fi networks."""
    @abstractmethod
    def connect_wifi(self, ssid: str, password: str) -> ApiResult[None]: """Connect to a Wi-Fi network."""
    @abstractmethod
    def get_ota_status(self) -> ApiResult[dict[str, Any]]: """Return OTA version/status."""
    @abstractmethod
    def start_ota_upgrade(self) -> ApiResult[None]: """Start OTA update."""
