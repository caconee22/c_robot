"""Display the benchmark CV tracker on the synthetic video in an endless loop."""

from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np


VIDEO = Path(__file__).resolve().parent / "data" / "synthetic.avi"
TARGET_FPS = 60.0


def track(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 100, 80), (10, 255, 255))
    for low, high in (
        ((170, 100, 80), (179, 255, 255)),
        ((35, 100, 80), (85, 255, 255)),
        ((90, 100, 80), (135, 255, 255)),
    ):
        mask |= cv2.inRange(hsv, low, high)

    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    contours = [c for c in contours if cv2.contourArea(c) >= 100]
    if not contours:
        return mask, contours, None

    largest = max(contours, key=cv2.contourArea)
    moments = cv2.moments(largest)
    if not moments["m00"]:
        return mask, contours, None
    center = (
        int(moments["m10"] / moments["m00"]),
        int(moments["m01"] / moments["m00"]),
    )
    return mask, contours, center


def main():
    cap = cv2.VideoCapture(str(VIDEO))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {VIDEO}")

    window = "Color Tower CV Tracker"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    frame_times = deque(maxlen=120)
    next_frame = time.perf_counter()

    while True:
        ok, frame = cap.read()
        if not ok:
            cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            continue

        started = time.perf_counter()
        mask, contours, center = track(frame)
        processing_ms = (time.perf_counter() - started) * 1000

        cv2.drawContours(frame, contours, -1, (0, 255, 255), 2)
        if center is not None:
            cv2.drawMarker(
                frame,
                center,
                (255, 255, 255),
                cv2.MARKER_CROSS,
                28,
                3,
            )
            cv2.putText(
                frame,
                f"center=({center[0]}, {center[1]})",
                (20, 100),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

        now = time.perf_counter()
        frame_times.append(now)
        display_fps = 0.0
        if len(frame_times) > 1:
            display_fps = (len(frame_times) - 1) / (frame_times[-1] - frame_times[0])

        status = (
            f"CV TRACKING  target={TARGET_FPS:.0f} FPS  "
            f"display={display_fps:.1f} FPS  process={processing_ms:.2f} ms"
        )
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 58), (0, 0, 0), -1)
        cv2.putText(
            frame,
            status,
            (20, 38),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (80, 255, 80),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(window, frame)

        next_frame += 1.0 / TARGET_FPS
        delay = max(1, int((next_frame - time.perf_counter()) * 1000))
        if next_frame < time.perf_counter() - 0.25:
            next_frame = time.perf_counter()
        key = cv2.waitKey(delay) & 0xFF
        if key in (27, ord("q")):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
