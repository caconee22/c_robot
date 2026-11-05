"""Green tower detector for the Pi: HSV proposal on a small frame, HSV check on
the full frame around each proposal (method 08 of cv_tests, 10/06), then pick
the tower instead of simply the largest green area.

Why the pick matters: in the 10/06 comparison almost every miss was a larger
green mat or background chosen over the tower, not a missing color.

Pick order:
  1. Not flat (height >= TALL_RATIO * width). A box touching the frame
     edge is exempt: a close or clipped tower can look wide.
  2. If the tower was seen recently, the nearest of the large candidates
     (>= TRACK_MIN_SHARE of the largest), so small noise never wins.
  3. Otherwise the largest tall candidate.

Stateful (remembers the last position), so use one TowerDetector per camera.

Several color ranges can be on at once (e.g. lit side, shadow side): their
masks are OR-ed before the blobs are found, so a tower split by shading is one
blob again (the 3 misses on the 10/06 video were such splits).

Settings (color ranges, size limits, flat limit, tracking): config/tower_hsv.json
when present (saved by scripts/tower_settings_app.py), otherwise the constants
below. Each detect() also lists the rejected blobs and why, for the settings app.
"""
from __future__ import annotations

import json
import os
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

# OpenCV H 0..179. Used when config/tower_hsv.json is missing: the range 학현
# saved with the calibrator on 11/04 (room light, real tower H~58 S~122).
# The older config/color_detector_last.json range (H 45-56, S>=138) missed it.
HSV_LOW = (51, 72, 32)
HSV_HIGH = (67, 255, 255)
RANGE_FILE = Path(__file__).resolve().parents[1] / "config" / "tower_hsv.json"
# Smallest blob, as a share of the frame (1080p: ~207 px, 720p: ~92 px). 11/04 Pi
# camera: 8 px let reflections and plants through; the smallest annotated far
# tower on the 10/06 videos was ~3000 px at 1080p.
MIN_AREA_FRACTION = 1e-4
MAX_AREA_FRACTION = 1.0     # largest blob, share of the frame; 1.0 = no limit
ROI_MARGIN = 12             # full-frame pixels around each proposal
TALL_RATIO = 0.5            # height / width; real tower boxes were 0.7-1.3, mats 0.2-0.47
TRACK_FRAMES = 10           # frames the last position is trusted after a miss
TRACK_GATE = 0.25           # max jump to the nearest candidate, fraction of frame width
TRACK_MIN_SHARE = 0.25      # tracking only picks among candidates >= this share of the largest
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


@dataclass
class Result:
    candidates: list = field(default_factory=list)   # chosen tower first


def valid_range(low, high) -> bool:
    try:
        return (len(low) == len(high) == 3 and all(0 <= int(a) <= int(b) for a, b in zip(low, high))
                and int(high[0]) <= 179 and int(high[1]) <= 255 and int(high[2]) <= 255)
    except (TypeError, ValueError):
        return False


def load_range(path: Path = RANGE_FILE) -> tuple[tuple, tuple]:
    """Saved range, or the fallback when the file is missing or broken."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        low, high = data["low"], data["high"]
        if valid_range(low, high):
            return tuple(int(v) for v in low), tuple(int(v) for v in high)
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return HSV_LOW, HSV_HIGH


def load_sizes(path: Path = RANGE_FILE) -> tuple[float, float]:
    """Saved tower size limits (share of the frame area), or the defaults."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        low, high = float(data["min_area"]), float(data["max_area"])
        if 0 <= low < high <= 1:
            return low, high
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return MIN_AREA_FRACTION, MAX_AREA_FRACTION


def load_ranges(path: Path = RANGE_FILE) -> list[dict]:
    """Saved color ranges [{name, low, high, on}]; a file from before 11/05 (one
    low/high pair) becomes a single range."""
    try:
        ranges = []
        for item in json.loads(path.read_text(encoding="utf-8"))["ranges"]:
            if valid_range(item["low"], item["high"]):
                ranges.append({"name": str(item.get("name", f"범위 {len(ranges) + 1}")),
                               "low": tuple(int(v) for v in item["low"]),
                               "high": tuple(int(v) for v in item["high"]),
                               "on": bool(item.get("on", True))})
        if ranges:
            return ranges
    except (OSError, ValueError, KeyError, TypeError):
        pass
    low, high = load_range(path)
    return [{"name": "범위 1", "low": low, "high": high, "on": True}]


def save_range(low, high, path: Path = RANGE_FILE,
               min_area: float = MIN_AREA_FRACTION, max_area: float = MAX_AREA_FRACTION,
               tall_ratio: float = TALL_RATIO, track: bool = True, ranges: list | None = None) -> None:
    if not valid_range(low, high):
        raise ValueError(f"bad HSV range {low} {high}")
    ranges = ranges or [{"name": "범위 1", "low": low, "high": high, "on": True}]
    if not all(valid_range(r["low"], r["high"]) for r in ranges):
        raise ValueError("bad HSV range in the list")
    if not 0 <= min_area < max_area <= 1:
        raise ValueError(f"bad size limits {min_area} {max_area}")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps({"low": [int(v) for v in low], "high": [int(v) for v in high],
                               "min_area": min_area, "max_area": max_area,
                               "tall_ratio": tall_ratio, "track": bool(track),
                               "ranges": [{"name": r["name"], "low": [int(v) for v in r["low"]],
                                           "high": [int(v) for v in r["high"]], "on": bool(r["on"])}
                                          for r in ranges]},
                              ensure_ascii=False), encoding="utf-8")
    os.replace(tmp, path)  # a power cut leaves the old file or the new one, never half


def load_shape(path: Path = RANGE_FILE) -> tuple[float, bool]:
    """Saved flat limit (height / width) and tracking switch, or the defaults."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
        ratio, track = float(data["tall_ratio"]), bool(data["track"])
        if 0 <= ratio <= 5:
            return ratio, track
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return TALL_RATIO, True


def save_settings(detector: "TowerDetector", path: Path = RANGE_FILE) -> None:
    save_range(detector.low, detector.high, path, detector.min_area, detector.max_area,
               detector.tall_ratio, detector.track, detector.ranges)


class TowerDetector:
    def __init__(self, range_file: Path = RANGE_FILE) -> None:
        self.last_center: tuple[float, float] | None = None
        self.missed = 0
        self.load(range_file)
        self.rejected: list = []   # (bbox, reason) of the last detect(), for the settings app

    def load(self, range_file: Path = RANGE_FILE) -> None:
        self.ranges = load_ranges(range_file)
        self.active = 0                                          # range the sliders edit
        self.min_area, self.max_area = load_sizes(range_file)   # share of the frame area
        self.tall_ratio, self.track = load_shape(range_file)

    # low/high = the active range, so single-range callers keep working.
    @property
    def low(self) -> tuple:
        return self.ranges[self.active]["low"]

    @low.setter
    def low(self, value) -> None:
        self.ranges[self.active]["low"] = tuple(int(v) for v in value)

    @property
    def high(self) -> tuple:
        return self.ranges[self.active]["high"]

    @high.setter
    def high(self, value) -> None:
        self.ranges[self.active]["high"] = tuple(int(v) for v in value)

    def range_mask(self, hsv: np.ndarray) -> np.ndarray:
        """OR of every range that is on (already-HSV image)."""
        mask = None
        for r in list(self.ranges):
            if r["on"]:
                part = cv2.inRange(hsv, r["low"], r["high"])
                mask = part if mask is None else cv2.bitwise_or(mask, part)
        return mask if mask is not None else np.zeros(hsv.shape[:2], np.uint8)

    def _hsv(self, image: np.ndarray) -> np.ndarray:
        return self.range_mask(cv2.cvtColor(image, cv2.COLOR_BGR2HSV))

    def mask(self, frame: np.ndarray, small: np.ndarray | None = None) -> np.ndarray:
        """Full-frame HSV mask, computed only around small-frame proposals."""
        height, width = frame.shape[:2]
        if small is None:
            small = cv2.resize(frame, (width // 2, height // 2), interpolation=cv2.INTER_AREA)
        proposal = self._hsv(small)
        sx, sy = width / small.shape[1], height / small.shape[0]
        full = np.zeros((height, width), np.uint8)
        contours, _ = cv2.findContours(proposal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            x0, y0 = max(0, int(x * sx) - ROI_MARGIN), max(0, int(y * sy) - ROI_MARGIN)
            x1 = min(width, int(np.ceil((x + w) * sx)) + ROI_MARGIN)
            y1 = min(height, int(np.ceil((y + h) * sy)) + ROI_MARGIN)
            full[y0:y1, x0:x1] |= self._hsv(frame[y0:y1, x0:x1])
        return cv2.morphologyEx(full, cv2.MORPH_CLOSE, KERNEL)

    def detect(self, frame: np.ndarray, small: np.ndarray | None = None) -> Result:
        height, width = frame.shape[:2]
        contours, _ = cv2.findContours(self.mask(frame, small), cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        min_area, max_area = self.min_area * width * height, self.max_area * width * height
        tall, rejected = [], []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                if area >= min_area / 4 and len(rejected) < 12:   # skip specks, keep near misses
                    rejected.append((cv2.boundingRect(contour), "small"))
                continue
            x, y, w, h = cv2.boundingRect(contour)
            if area > max_area:
                rejected.append(((x, y, w, h), "big"))
                continue
            edge = x <= 0 or y <= 0 or x + w >= width or y + h >= height
            if h < self.tall_ratio * w and not edge:
                rejected.append(((x, y, w, h), "flat"))
                continue
            m = cv2.moments(contour)
            tall.append({"bbox": (x, y, w, h), "area": area,
                         "centroid": (m["m10"] / m["m00"], m["m01"] / m["m00"])})
        tall.sort(key=lambda c: c["area"], reverse=True)
        self.rejected = rejected
        if not tall:
            self.missed += 1
            if self.missed > TRACK_FRAMES:
                self.last_center = None
            return Result()
        if self.track and self.last_center is not None:
            lx, ly = self.last_center
            big = [c for c in tall if c["area"] >= TRACK_MIN_SHARE * tall[0]["area"]]
            near = min(big, key=lambda c: (c["centroid"][0] - lx) ** 2 + (c["centroid"][1] - ly) ** 2)
            if np.hypot(near["centroid"][0] - lx, near["centroid"][1] - ly) <= TRACK_GATE * width:
                tall.remove(near)
                tall.insert(0, near)
        self.last_center, self.missed = tall[0]["centroid"], 0
        return Result(tall)
