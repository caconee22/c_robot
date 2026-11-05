"""Live camera window for bench tests, with the tower color calibrator.

Shown by pi_vision_sender.py --preview (desktop icon "C_ROBOT 카메라 보기").
  - Drag a box over the tower: its color becomes the detection range.
  - Six sliders (H/S/V min and max) change the range live; the detector that
    sends packets to the ESP32 uses it at once.
  - Two size sliders: smallest tower (0.01 % steps of the frame area) and
    largest tower (% of the frame). Green blobs outside them are ignored.
  - Pink tint = pixels inside the range, green box = chosen tower.
  - s: save to config/tower_hsv.json (used from then on, also at boot)
    r: reload the saved range   q: quit
Drawn every 3rd frame at 640x360: the Pi 5 kept 50 fps detection (11/04).
"""
from __future__ import annotations

import cv2
import numpy as np

from tower_detector_hsv import TowerDetector, load_range, load_sizes, save_settings

WINDOW = "C_ROBOT vision"
SIZE = (640, 360)
EVERY = 3
BARS = (("H min", 179), ("H max", 179), ("S min", 255), ("S max", 255), ("V min", 255), ("V max", 255))
SIZE_BARS = (("Min size x0.01%", 300), ("Max size %", 100))
H_MARGIN, SV_MARGIN = 4, 30   # widen the dragged color a little for light changes


def pick_range(hsv_pixels: np.ndarray) -> tuple[tuple, tuple]:
    """Range from the dragged pixels: 5th-95th percentile, widened; S/V max stay
    open so highlights on the tower still count."""
    pixels = hsv_pixels.reshape(-1, 3).astype(np.int32)
    lo = np.percentile(pixels, 5, axis=0)
    hi = np.percentile(pixels, 95, axis=0)
    low = (max(0, int(lo[0]) - H_MARGIN), max(0, int(lo[1]) - SV_MARGIN), max(0, int(lo[2]) - SV_MARGIN))
    high = (min(179, int(hi[0]) + H_MARGIN), 255, 255)
    return low, high


class Preview:
    def __init__(self, detector: TowerDetector) -> None:
        self.detector = detector
        self.raw = None          # last shown frame without drawings, for the drag
        self.start = None
        self.box = None
        self.message = "drag a box on the tower"
        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
        for name, top in BARS + SIZE_BARS:
            cv2.createTrackbar(name, WINDOW, 0, top, lambda _value: None)
        self._set_bars(detector.low, detector.high)
        self._set_sizes(detector.min_area, detector.max_area)
        cv2.setMouseCallback(WINDOW, self._mouse)

    def _set_bars(self, low, high) -> None:
        for (name, _top), value in zip(BARS, (low[0], high[0], low[1], high[1], low[2], high[2])):
            cv2.setTrackbarPos(name, WINDOW, int(value))

    def _set_sizes(self, min_area, max_area) -> None:
        cv2.setTrackbarPos(SIZE_BARS[0][0], WINDOW, round(min_area * 1e4))
        cv2.setTrackbarPos(SIZE_BARS[1][0], WINDOW, max(1, round(max_area * 100)))

    def _read_sizes(self):
        low = cv2.getTrackbarPos(SIZE_BARS[0][0], WINDOW) * 1e-4
        high = max(1, cv2.getTrackbarPos(SIZE_BARS[1][0], WINDOW)) / 100
        return low, max(high, low + 1e-4)

    def _read_bars(self):
        v = [cv2.getTrackbarPos(name, WINDOW) for name, _top in BARS]
        return (v[0], v[2], v[4]), (max(v[0], v[1]), max(v[2], v[3]), max(v[4], v[5]))

    def _mouse(self, event, x, y, flags, _param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            self.start, self.box = (x, y), None
        elif event == cv2.EVENT_MOUSEMOVE and self.start and flags & cv2.EVENT_FLAG_LBUTTON:
            self.box = (*self.start, x, y)
        elif event == cv2.EVENT_LBUTTONUP and self.start:
            x0, x1 = sorted((self.start[0], x))
            y0, y1 = sorted((self.start[1], y))
            self.start, self.box = None, None
            if self.raw is None or (x1 - x0) * (y1 - y0) < 9:
                self.message = "box too small, drag again"
                return
            region = self.raw[max(0, y0):y1, max(0, x0):x1]
            low, high = pick_range(cv2.cvtColor(region, cv2.COLOR_BGR2HSV))
            self._set_bars(low, high)
            self.message = "picked from box - s to save"

    def show(self, frame, search_frame, candidates, frame_number) -> None:
        low, high = self._read_bars()
        min_area, max_area = self._read_sizes()
        self.detector.low, self.detector.high = low, high   # live: the sender uses it too
        self.detector.min_area, self.detector.max_area = min_area, max_area
        view = search_frame if search_frame is not None else frame
        if (view.shape[1], view.shape[0]) != SIZE:
            view = cv2.resize(view, SIZE, interpolation=cv2.INTER_AREA)
        self.raw = view.copy()
        mask = cv2.inRange(cv2.cvtColor(view, cv2.COLOR_BGR2HSV), low, high)
        tint = view.copy()
        tint[mask > 0] = (255, 0, 255)
        view = cv2.addWeighted(view, 0.6, tint, 0.4, 0)
        k = SIZE[0] / frame.shape[1]
        for i, c in enumerate(candidates[:2]):
            x, y, w, h = (int(v * k) for v in c["bbox"])
            cv2.rectangle(view, (x, y), (x + w, y + h), (0, 255, 0) if i == 0 else (0, 200, 255), 2)
        cv2.line(view, (SIZE[0] // 2, 0), (SIZE[0] // 2, SIZE[1]), (255, 255, 255), 1)
        if self.box:
            cv2.rectangle(view, self.box[:2], self.box[2:], (0, 255, 255), 1)
        # Squares of the smallest and largest tower area, for scale (bottom right).
        for share, color in ((min_area, (0, 255, 255)), (min(max_area, 0.25), (255, 255, 0))):
            side = max(2, int((share * SIZE[0] * SIZE[1]) ** 0.5))
            cv2.rectangle(view, (SIZE[0] - 8 - side, SIZE[1] - 30 - side), (SIZE[0] - 8, SIZE[1] - 30), color, 1)
        text = (f"#{frame_number} targets {len(candidates)}  H {low[0]}-{high[0]} "
                f"S {low[1]}-{high[1]} V {low[2]}-{high[2]}  size {min_area * 100:.2f}-{max_area * 100:.0f}%")
        cv2.putText(view, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        cv2.putText(view, self.message + "   s save  r reload  q quit", (8, SIZE[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            raise KeyboardInterrupt  # same clean exit as Ctrl+C
        if key == ord("s"):
            save_settings(self.detector)   # keeps the settings app's other ranges
            self.message = "saved"
        elif key == ord("r"):
            self._set_bars(*load_range())
            self._set_sizes(*load_sizes())
            self.message = "reloaded saved range"
