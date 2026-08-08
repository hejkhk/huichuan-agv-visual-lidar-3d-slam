"""Decode saved Nav2 maps into the same immutable preview used by ROS."""

from __future__ import annotations

import base64
import binascii
from pathlib import Path
import struct
from typing import Any
import zlib

import yaml


MapSignature = tuple[int, int, float, float, float, int]


def occupancy_grid_png(data: list[int], width: int, height: int) -> str:
    """Encode a ROS occupancy grid as a vertically corrected grayscale PNG."""

    if width <= 0 or height <= 0 or len(data) != width * height:
        return ""

    def shade(value: int) -> int:
        if value < 0:
            return 205
        if value >= 65:
            return 35
        if value <= 10:
            return 247
        return max(45, 247 - round(value * 2.1))

    raw = bytearray()
    for row in range(height - 1, -1, -1):
        raw.append(0)
        start = row * width
        raw.extend(shade(value) for value in data[start:start + width])

    def chunk(kind: bytes, payload: bytes) -> bytes:
        checksum = binascii.crc32(kind + payload) & 0xFFFFFFFF
        return (
            struct.pack(">I", len(payload))
            + kind
            + payload
            + struct.pack(">I", checksum)
        )

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(
        b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    )
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 6))
    png += chunk(b"IEND", b"")
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")


def map_signature(
    data: list[int],
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
) -> MapSignature:
    """Return the UI/ROS identity of one immutable occupancy grid."""

    raw = bytes((int(value) & 0xFF) for value in data)
    return (
        int(width),
        int(height),
        round(float(resolution), 6),
        round(float(origin_x), 6),
        round(float(origin_y), 6),
        zlib.crc32(raw) & 0xFFFFFFFF,
    )


def _token(data: bytes, index: int) -> tuple[bytes, int]:
    while True:
        while index < len(data) and data[index] in b" \t\r\n":
            index += 1
        if index < len(data) and data[index] == ord("#"):
            newline = data.find(b"\n", index)
            index = len(data) if newline < 0 else newline + 1
            continue
        break
    start = index
    while index < len(data) and data[index] not in b" \t\r\n#":
        index += 1
    if start == index:
        raise ValueError("unexpected end of PGM header")
    return data[start:index], index


def _read_pgm(path: Path) -> tuple[list[int], int, int]:
    raw = path.read_bytes()
    index = 0
    header: list[bytes] = []
    for _ in range(4):
        value, index = _token(raw, index)
        header.append(value)
    magic = header[0]
    width, height, maximum = (int(value) for value in header[1:])
    if magic not in {b"P2", b"P5"}:
        raise ValueError(f"unsupported PGM magic {magic!r}")
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 65535:
        raise ValueError("invalid PGM dimensions or maximum value")

    sample_count = width * height
    if magic == b"P2":
        samples: list[int] = []
        for _ in range(sample_count):
            value, index = _token(raw, index)
            samples.append(int(value))
    else:
        if index >= len(raw) or raw[index] not in b" \t\r\n":
            raise ValueError("PGM header has no binary separator")
        if raw[index:index + 2] == b"\r\n":
            index += 2
        else:
            index += 1
        bytes_per_sample = 1 if maximum < 256 else 2
        expected = sample_count * bytes_per_sample
        payload = raw[index:index + expected]
        if len(payload) != expected:
            raise ValueError("PGM pixel payload is truncated")
        if bytes_per_sample == 1:
            samples = list(payload)
        else:
            samples = [
                int.from_bytes(payload[offset:offset + 2], "big")
                for offset in range(0, len(payload), 2)
            ]
    if any(value < 0 or value > maximum for value in samples):
        raise ValueError("PGM pixel is outside the declared range")
    if maximum != 255:
        samples = [round(value * 255.0 / maximum) for value in samples]
    return samples, width, height


def load_saved_map_preview(yaml_path: str | Path) -> dict[str, Any]:
    """Load a YAML/PGM pair and return an immutable UI map payload."""

    path = Path(yaml_path).expanduser().resolve()
    payload = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(payload, dict):
        raise ValueError("map YAML root must be an object")
    image_value = str(payload.get("image", "")).strip()
    if not image_value:
        raise ValueError("map YAML has no image field")
    image_path = Path(image_value).expanduser()
    if not image_path.is_absolute():
        image_path = (path.parent / image_path).resolve()
    pixels, width, height = _read_pgm(image_path)

    resolution = float(payload["resolution"])
    origin = payload["origin"]
    if not isinstance(origin, (list, tuple)) or len(origin) < 2:
        raise ValueError("map YAML origin must contain x and y")
    origin_x = float(origin[0])
    origin_y = float(origin[1])
    negate = bool(int(payload.get("negate", 0)))
    occupied = float(payload.get("occupied_thresh", 0.65))
    free = float(payload.get("free_thresh", 0.196))
    mode = str(payload.get("mode", "trinary")).strip().lower()
    if resolution <= 0 or not 0 <= free < occupied <= 1:
        raise ValueError("map YAML resolution or thresholds are invalid")

    data: list[int] = []
    for row in range(height - 1, -1, -1):
        start = row * width
        for pixel in pixels[start:start + width]:
            probability = pixel / 255.0 if negate else (255 - pixel) / 255.0
            if probability > occupied:
                value = 100
            elif probability < free:
                value = 0
            elif mode == "scale":
                value = max(
                    1,
                    min(99, round((probability - free) * 100 / (occupied - free))),
                )
            else:
                value = -1
            data.append(value)

    signature = map_signature(
        data, width, height, resolution, origin_x, origin_y
    )
    image = occupancy_grid_png(data, width, height)
    if not image:
        raise ValueError("saved map PNG encoding failed")
    return {
        "map_image": image,
        "map_width": width,
        "map_height": height,
        "map_resolution": resolution,
        "map_origin_x": origin_x,
        "map_origin_y": origin_y,
        "map_signature": signature,
        "map_crc32": f"0x{signature[-1]:08x}",
        "map_yaml_path": str(path),
        "map_image_path": str(image_path),
    }
