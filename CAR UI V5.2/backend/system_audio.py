from __future__ import annotations

import logging
import re
import shutil
import subprocess


class SystemAudio:
    """Read and change the Ubuntu default output volume.

    Ubuntu 24.04 normally exposes PipeWire through ``wpctl``. ``amixer`` is a
    conservative fallback for minimal Jetson images. Commands are executed as
    argument lists (never through a shell), bounded by a short timeout, and do
    not depend on ROS or any voice-team integration.
    """

    def __init__(self) -> None:
        self.log = logging.getLogger("AUDIO")

    @staticmethod
    def _run(arguments: list[str]) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            arguments,
            capture_output=True,
            check=False,
            text=True,
            timeout=3,
        )

    def read_volume(self) -> tuple[int | None, str]:
        """Return ``(percent, error)`` for the default output device."""

        if shutil.which("wpctl"):
            try:
                result = self._run(
                    ["wpctl", "get-volume", "@DEFAULT_AUDIO_SINK@"]
                )
                match = re.search(r"Volume:\s*([0-9.]+)", result.stdout)
                if result.returncode == 0 and match:
                    percent = round(float(match.group(1)) * 100)
                    return max(0, min(100, percent)), ""
                detail = result.stderr.strip() or result.stdout.strip()
                self.log.warning("wpctl 读取音量失败：%s", detail)
            except (OSError, subprocess.SubprocessError, ValueError) as exc:
                self.log.warning("wpctl 读取音量异常：%s", exc)

        if shutil.which("amixer"):
            try:
                result = self._run(["amixer", "get", "Master"])
                match = re.search(r"\[(\d{1,3})%\]", result.stdout)
                if result.returncode == 0 and match:
                    return max(0, min(100, int(match.group(1)))), ""
                detail = result.stderr.strip() or result.stdout.strip()
                self.log.warning("amixer 读取音量失败：%s", detail)
            except (OSError, subprocess.SubprocessError, ValueError) as exc:
                self.log.warning("amixer 读取音量异常：%s", exc)

        return None, "未找到可用的 PipeWire/ALSA 输出设备"

    def set_volume(self, percent: int) -> tuple[bool, str]:
        """Set default output volume, unmuting it when percent is non-zero."""

        percent = max(0, min(100, int(percent)))
        if shutil.which("wpctl"):
            try:
                result = self._run(
                    [
                        "wpctl",
                        "set-volume",
                        "@DEFAULT_AUDIO_SINK@",
                        f"{percent}%",
                    ]
                )
                if result.returncode == 0:
                    if percent > 0:
                        self._run(
                            [
                                "wpctl",
                                "set-mute",
                                "@DEFAULT_AUDIO_SINK@",
                                "0",
                            ]
                        )
                    return True, ""
                detail = result.stderr.strip() or result.stdout.strip()
                self.log.warning("wpctl 设置音量失败：%s", detail)
            except (OSError, subprocess.SubprocessError) as exc:
                self.log.warning("wpctl 设置音量异常：%s", exc)

        if shutil.which("amixer"):
            try:
                result = self._run(
                    ["amixer", "set", "Master", f"{percent}%", "unmute"]
                )
                if result.returncode == 0:
                    return True, ""
                detail = result.stderr.strip() or result.stdout.strip()
                self.log.warning("amixer 设置音量失败：%s", detail)
            except (OSError, subprocess.SubprocessError) as exc:
                self.log.warning("amixer 设置音量异常：%s", exc)

        return False, "系统音量设置失败，请检查 PipeWire 或 ALSA"
