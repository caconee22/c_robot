"""Shared-memory tap: hands a few small frames from the running sender to the
tracking viewer (scripts/vision_viewer.py) without slowing detection.

The viewer creates /dev/shm/c_robot_view and keeps an "alive" time in it. The
sender only writes while a viewer is alive, and then only every EVERY-th frame:
one copy of the small search frame (no encoding, no drawing) plus the boxes.
With no viewer the cost is one os.path.exists() per second.

Layout: 64-byte header, then a BGR frame of at most MAX_W x MAX_H.
A seqlock (odd while writing) lets the viewer skip a half-written frame.
"""
from __future__ import annotations

import mmap
import os
import struct
import time

import numpy as np

PATH = "/dev/shm/c_robot_view"
MAX_W, MAX_H = 960, 540
EVERY = 4                       # 1 frame in 4: ~12-15 fps in the viewer at 50-60 fps detection
MAX_BOXES = 4
# offset 0 seq (uint32), 4 alive (viewer, monotonic s, written only by the viewer),
# 12: written (monotonic s), w, h, full_w, full_h, n,
# fps, detect_ms, age_ms, boxes (x, y, w, h) * 4 in full-frame pixels
HEADER = struct.Struct("<d HHHH B fff 16h")   # at offset 12
HEADER_SIZE = 96
SIZE = HEADER_SIZE + MAX_W * MAX_H * 3
ALIVE_S = 2.0


def open_map(create: bool) -> mmap.mmap | None:
    try:
        if create:
            with open(PATH, "wb") as f:
                f.truncate(SIZE)
        fd = os.open(PATH, os.O_RDWR)
    except OSError:
        return None
    try:
        if os.fstat(fd).st_size < SIZE:
            return None
        return mmap.mmap(fd, SIZE)
    finally:
        os.close(fd)


class ViewTap:
    """Sender side. publish() is called once per detected frame."""

    def __init__(self) -> None:
        self.map: mmap.mmap | None = None
        self.next_check = 0.0
        self.count = 0
        self.seq = 0

    def publish(self, full_shape, search, candidates, stats) -> None:
        self.count += 1
        if self.count % EVERY:
            return
        now = time.monotonic()
        if self.map is None:
            if now < self.next_check:
                return
            self.next_check = now + 1.0
            if not os.path.exists(PATH):
                return
            self.map = open_map(create=False)
            if self.map is None:
                return
        try:
            alive = struct.unpack_from("<d", self.map, 4)[0]
            if now - alive > ALIVE_S:
                if not os.path.exists(PATH):      # viewer closed and removed it
                    self.map.close()
                    self.map = None
                return
            h, w = search.shape[:2]
            if w > MAX_W or h > MAX_H:
                return
            boxes = [0] * 16
            for i, c in enumerate(candidates[:MAX_BOXES]):
                boxes[i * 4:i * 4 + 4] = [int(v) for v in c["bbox"]]
            age = stats.get("age_ms") or 0.0
            self.seq += 1
            struct.pack_into("<I", self.map, 0, self.seq * 2 + 1)        # writing
            dst = np.frombuffer(self.map, np.uint8, h * w * 3, HEADER_SIZE).reshape(h, w, 3)
            dst[:] = search
            HEADER.pack_into(self.map, 12, now, w, h,
                             full_shape[1], full_shape[0], min(len(candidates), MAX_BOXES),
                             stats.get("fps", 0.0), stats.get("detect_ms", 0.0), age, *boxes)
            struct.pack_into("<I", self.map, 0, self.seq * 2)            # done
        except Exception:  # noqa: BLE001
            self.map = None   # never let the viewer break detection
