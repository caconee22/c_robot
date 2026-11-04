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

Color range and tower size limits: config/tower_hsv.json when present (saved
by the calibrator in scripts/vision_preview.py), otherwise the constants below.
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


def save_range(low, high, path: Path = RANGE_FILE,
               min_area: float = MIN_AREA_FRACTION, max_area: float = MAX_AREA_FRACTION) -> None:
    if not valid_range(low, high):
        raise ValueError(f"bad HSV range {low} {high}")
    if not 0 <= min_area < max_area <= 1:
        raise ValueError(f"bad size limits {min_area} {max_area}")
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps({"low": [int(v) for v in low], "high": [int(v) for v in high],
                               "min_area": min_area, "max_area": max_area}), encoding="utf-8")
    os.replace(tmp, path)  # a power cut leaves the old file or the new one, never half


class TowerDetector:
    def __init__(self, range_file: Path = RANGE_FILE) -> None:
        self.last_center: tuple[float, float] | None = None
        self.missed = 0
        self.low, self.high = load_range(range_file)
        self.min_area, self.max_area = load_sizes(range_file)   # share of the frame area

    def _hsv(self, image: np.ndarray) -> np.ndarray:
        return cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), self.low, self.high)

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
        tall = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area or area > max_area:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            edge = x <= 0 or y <= 0 or x + w >= width or y + h >= height
            if h < TALL_RATIO * w and not edge:
                continue
            m = cv2.moments(contour)
            tall.append({"bbox": (x, y, w, h), "area": area,
                         "centroid": (m["m10"] / m["m00"], m["m01"] / m["m00"])})
        tall.sort(key=lambda c: c["area"], reverse=True)
        if not tall:
            self.missed += 1
            if self.missed > TRACK_FRAMES:
                self.last_center = None
            return Result()
        if self.last_center is not None:
            lx, ly = self.last_center
            big = [c for c in tall if c["area"] >= TRACK_MIN_SHARE * tall[0]["area"]]
            near = min(big, key=lambda c: (c["centroid"][0] - lx) ** 2 + (c["centroid"][1] - ly) ** 2)
            if np.hypot(near["centroid"][0] - lx, near["centroid"][1] - ly) <= TRACK_GATE * width:
                tall.remove(near)
                tall.insert(0, near)
        self.last_center, self.missed = tall[0]["centroid"], 0
        return Result(tall)
