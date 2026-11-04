"""Raspberry Pi -> ESP32 vision packet V1 (ESP_MAIN README section 10).

14 bytes, little-endian, no CRC (intended, see AGENTS.md):
  AA 55 01 0A | seq | flags | x | y | w | h   (x, y, w, h are uint16, 1920x1080 scale)
The checks here mirror ESP_MAIN/src/communication/raspberry_link.cpp validate(),
so a packet that leaves this module is one the ESP32 accepts.
"""
from __future__ import annotations

import struct
from typing import Any, Sequence

HEADER = bytes((0xAA, 0x55, 0x01, 0x0A))
FRAME_WIDTH, FRAME_HEIGHT = 1920, 1080

TARGET_VALID = 1 << 0
TRACK_STABLE = 1 << 1
MULTIPLE_TARGETS = 1 << 2
BOX_CLIPPED = 1 << 3
CAMERA_OK = 1 << 4
PIPELINE_OK = 1 << 5
TARGET_ONLY_FLAGS = TRACK_STABLE | MULTIPLE_TARGETS | BOX_CLIPPED

STABLE_FRAMES = 3  # consecutive frames with a target before TrackStable is set


def encode(sequence: int, flags: int, x: int = 0, y: int = 0, width: int = 0, height: int = 0) -> bytes:
    """Build one packet. Raises ValueError for anything the ESP32 would reject."""
    if not 0 <= sequence <= 255:
        raise ValueError(f"sequence {sequence}")
    if flags & ~0x3F:
        raise ValueError(f"reserved flag bits {flags:#x}")
    if flags & TARGET_VALID:
        if not (flags & CAMERA_OK and flags & PIPELINE_OK):
            raise ValueError("target without camera/pipeline ok")
        if not (0 <= x < FRAME_WIDTH and 0 <= y < FRAME_HEIGHT):
            raise ValueError(f"center {x},{y}")
        if not (1 <= width <= FRAME_WIDTH and 1 <= height <= FRAME_HEIGHT):
            raise ValueError(f"box {width}x{height}")
    elif flags & TARGET_ONLY_FLAGS or (x, y, width, height) != (0, 0, 0, 0):
        raise ValueError("no target but target data")
    return HEADER + struct.pack("<BBHHHH", sequence, flags, x, y, width, height)


def from_candidates(
    sequence: int,
    candidates: Sequence[dict[str, Any]],
    frame_size: tuple[int, int],
    stable_count: int,
) -> bytes:
    """Packet for one processed frame. candidates come from detect_frame_lightweight,
    largest first; bbox/centroid are in frame_size (width, height) pixels."""
    flags = CAMERA_OK | PIPELINE_OK
    if not candidates:
        return encode(sequence, flags)
    frame_w, frame_h = frame_size
    sx, sy = FRAME_WIDTH / frame_w, FRAME_HEIGHT / frame_h
    bx, by, bw, bh = candidates[0]["bbox"]
    cx, cy = candidates[0]["centroid"]
    flags |= TARGET_VALID
    if len(candidates) > 1:
        flags |= MULTIPLE_TARGETS
    if bx <= 0 or by <= 0 or bx + bw >= frame_w or by + bh >= frame_h:
        flags |= BOX_CLIPPED
    if stable_count >= STABLE_FRAMES:
        flags |= TRACK_STABLE
    return encode(
        sequence,
        flags,
        min(max(round(cx * sx), 0), FRAME_WIDTH - 1),
        min(max(round(cy * sy), 0), FRAME_HEIGHT - 1),
        min(max(round(bw * sx), 1), FRAME_WIDTH),
        min(max(round(bh * sy), 1), FRAME_HEIGHT),
    )
