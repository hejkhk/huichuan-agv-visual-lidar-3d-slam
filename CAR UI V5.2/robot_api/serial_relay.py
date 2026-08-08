"""Serial port relay bridge — forwards data between real serial and a PTY slave,
while parsing the 25-byte MCU frames for battery voltage and charging status."""

from __future__ import annotations

import logging
import os
import struct
import threading
from typing import Callable

import serial


class SerialRelay:
    """Create a PTY pair and relay data between the real serial port and the PTY slave.
    The PTY slave path is exposed so that base_node can connect to it instead of the
    real serial device.  Battery data is extracted from upstream 25-byte MCU frames."""

    def __init__(self, real_port: str, baud: int = 115200):
        self.real_port = real_port
        self.baud = baud
        self.log = logging.getLogger("RELAY")
        self._voltage_divisor = float(os.getenv("ROBOT_UI_VOLTAGE_DIVISOR", "1000"))
        self._closed = False
        self._listeners: list[Callable[[float, bool], None]] = []
        self._master_fd: int = -1
        self._slave_name: str = ""
        self._ser: serial.Serial | None = None
        self._threads: list[threading.Thread] = []
        self._open()

    # ── public API ─────────────────────────────────────────────────

    @property
    def slave_path(self) -> str:
        return self._slave_name

    def add_listener(self, callback: Callable[[float, bool], None]) -> None:
        """callback(voltage: float, charging: bool) — called on every valid MCU frame."""
        self._listeners.append(callback)

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        if self._ser and self._ser.is_open:
            self._ser.close()
        if self._master_fd >= 0:
            os.close(self._master_fd)
        for t in self._threads:
            t.join(timeout=0.5)
        self.log.info("串口中继桥已关闭")

    # ── internal ───────────────────────────────────────────────────

    def _open(self) -> None:
        master_fd, slave_name = os.openpty()
        self._master_fd = master_fd
        self._slave_name = slave_name
        self.log.info("PTY 已创建 → %s", slave_name)

        self._ser = serial.Serial(self.real_port, self.baud, timeout=0.01)
        self.log.info("真实串口已打开 → %s @ %d", self.real_port, self.baud)

        t1 = threading.Thread(target=self._relay_serial_to_pty, name="relay-serial2pty", daemon=True)
        t2 = threading.Thread(target=self._relay_pty_to_serial, name="relay-pty2serial", daemon=True)
        self._threads = [t1, t2]
        t1.start()
        t2.start()

    def _relay_serial_to_pty(self) -> None:
        """Read from real serial, parse frames for battery, forward to PTY."""
        buf = bytearray()
        try:
            while not self._closed:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if not chunk:
                    continue
                buf.extend(chunk)
                os.write(self._master_fd, chunk)
                buf = self._parse_frames(buf, max(len(buf), 0))
        except Exception:
            if not self._closed:
                self.log.exception("serial→pty 线程异常")

    def _relay_pty_to_serial(self) -> None:
        """Read from PTY master and forward to real serial."""
        try:
            while not self._closed:
                data = os.read(self._master_fd, 1024)
                if not data:
                    break
                self._ser.write(data)
        except Exception:
            if not self._closed:
                self.log.exception("pty→serial 线程异常")

    def _parse_frames(self, buf: bytearray, limit: int) -> bytearray:
        """Extract valid 25-byte frames from buffer, return remaining bytes."""
        while len(buf) >= 25:
            try:
                idx = buf.index(0x7B)
            except ValueError:
                return bytearray()
            if idx > 0:
                del buf[:idx]
            if len(buf) < 25:
                break
            if buf[24] != 0x7D:
                del buf[:1]
                continue
            payload = buf[1:24]
            bcc = 0x7B
            for b in payload[:22]:
                bcc ^= b
            if bcc != payload[22]:
                del buf[:1]
                continue
            voltage_raw = struct.unpack(">H", payload[19:21])[0]
            voltage = voltage_raw / self._voltage_divisor
            charging = payload[21] == 0x01
            for cb in self._listeners:
                try:
                    cb(voltage, charging)
                except Exception:
                    pass
            del buf[:25]
        return buf
