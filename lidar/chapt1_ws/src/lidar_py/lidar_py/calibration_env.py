"""Atomic, comment-preserving updates for the dual-resolution calibration env."""

from __future__ import annotations

import os
import re
import shutil
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Mapping


_KEY_PATTERN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_RESTART_MARKER_NAME = ".camera_calibration_restart_required"


def calibration_restart_marker(env_file: str) -> Path:
    return Path(env_file).expanduser().resolve().parent / _RESTART_MARKER_NAME


def mark_calibration_restart_required(env_file: str, stage: str) -> str:
    """Atomically record that the running static camera TF is stale."""
    marker = calibration_restart_marker(env_file)
    marker.write_text(f"{stage}\n", encoding="utf-8")
    return str(marker)


def update_env_file(env_file: str, updates: Mapping[str, object]) -> str:
    """Back up and atomically update existing KEY=value entries.

    Returns the backup path. Unknown keys are appended so this helper remains
    usable when a newer calibration field is introduced.
    """
    path = Path(env_file).expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f"Calibration env file does not exist: {path}")
    if not updates:
        raise ValueError("No calibration values were supplied")
    for key in updates:
        if not _KEY_PATTERN.fullmatch(key):
            raise ValueError(f"Invalid env key: {key!r}")

    original = path.read_text(encoding="utf-8")
    lines = original.splitlines(keepends=True)
    pending = {key: str(value) for key, value in updates.items()}
    updated_lines = []

    for line in lines:
        stripped = line.lstrip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            updated_lines.append(line)
            continue
        key = stripped.split("=", 1)[0].strip()
        if key not in pending:
            updated_lines.append(line)
            continue
        newline = "\r\n" if line.endswith("\r\n") else "\n"
        if not line.endswith(("\n", "\r\n")):
            newline = ""
        updated_lines.append(f"{key}={pending.pop(key)}{newline}")

    if pending:
        if updated_lines and not updated_lines[-1].endswith(("\n", "\r\n")):
            updated_lines[-1] += "\n"
        updated_lines.append("\n# Added automatically by camera calibration.\n")
        updated_lines.extend(f"{key}={value}\n" for key, value in pending.items())

    rendered = "".join(updated_lines)
    if rendered == original:
        return ""

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    backup = path.with_name(f"{path.name}.bak.{timestamp}")
    shutil.copy2(path, backup)

    fd, temp_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8", newline="") as stream:
            stream.write(rendered)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    except Exception:
        try:
            os.unlink(temp_name)
        except FileNotFoundError:
            pass
        raise
    return str(backup)
