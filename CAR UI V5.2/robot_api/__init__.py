from __future__ import annotations

import os
from pathlib import Path

from .base import RobotApiBase
from .mock import MockRobotApi
from .team import TeamRobotApi


def create_robot_api(data_dir: str | Path | None = None) -> RobotApiBase:
    """Create the configured public API implementation.

    ``ROBOT_API_MODE=mock`` selects the offline deterministic adapter.
    ``ROBOT_API_MODE=team`` (default) selects ROS/serial integration with mock
    fallbacks. ``data_dir`` controls persistent JSON placement and should point
    outside the installed application for production packages.
    """

    mode = os.getenv("ROBOT_API_MODE", "team").strip().lower()
    if mode == "mock":
        return MockRobotApi(data_dir)
    if mode == "team":
        return TeamRobotApi(data_dir)
    raise ValueError(f"不支持的 ROBOT_API_MODE：{mode}")


__all__ = ["RobotApiBase", "MockRobotApi", "TeamRobotApi", "create_robot_api"]
