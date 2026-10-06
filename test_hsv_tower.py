"""Standalone HSV-only green tower video test. No UART or old tracker imports."""

import argparse
from collections import deque
from pathlib import Path
import time

import cv2
import numpy as np


# OpenCV hue: 0..179. Tune these three lines first.
HSV_LOW = (35, 60, 35)
HSV_HIGH = (90, 255, 255)
MIN_AREA = 8
DEFAULT_VIDEO = Path(__file__).parent / "img" / "KakaoTalk_20261006_201439113.mp4"
WINDOW = "HSV Tower Test | SPACE pause | M mask | Q quit"
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))


def detect(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, HSV_LOW, HSV_HIGH)
    # Close small gaps, but do not open/erode away distant small targets.
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest = max(contours, key=cv2.contourArea, default=None)
    if largest is None or cv2.contourArea(largest) < MIN_AREA:
        return mask, None
    x, y, w, h = cv2.boundingRect(largest)
    moments = cv2.moments(largest)
    center = (round(moments["m10"] / moments["m00"]),
              round(moments["m01"] / moments["m00"]))
    return mask, (x, y, w, h, center)


def annotate(frame, target, cv_ms, cv_fps, playback_fps, source_fps, index, misses):
    if target is not None:
        x, y, w, h, center = target
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3)
        cv2.drawMarker(frame, center, (0, 0, 255), cv2.MARKER_CROSS, 20, 3)
    lines = [
        f"HSV {'DETECTED' if target else 'NO TARGET'} | frame {index} | empty {misses}",
        f"CV {cv_ms:.2f} ms | CV capacity {cv_fps:.1f} FPS",
        f"Playback {playback_fps:.1f} FPS | source {source_fps:.2f} FPS",
        f"Center {target[4] if target else '-'} | SPACE pause / M mask / Q quit",
    ]
    cv2.rectangle(frame, (0, 0), (frame.shape[1], 170), (0, 0, 0), -1)
    for row, line in enumerate(lines):
        cv2.putText(frame, line, (15, 32 + row * 38), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 2, cv2.LINE_AA)
    return frame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("video", nargs="?", default=str(DEFAULT_VIDEO))
    parser.add_argument("--headless", action="store_true", help="Check entire video without a window")
    parser.add_argument("--snapshot", help="Save annotated frame 300 for inspection")
    args = parser.parse_args()
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open video: {args.video}")
    source_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    timings = []
    recent = deque(maxlen=30)
    displayed = deque(maxlen=30)
    index = misses = 0
    show_mask = False
    if not args.headless:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 608, 1080)
    try:
        while True:
            loop_start = time.perf_counter()
            ok, frame = cap.read()
            if not ok:
                if args.headless or index == 0:
                    break
                print(f"Loop complete: {index} frames, {misses} empty detections", flush=True)
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                index = misses = 0
                displayed.clear()
                continue
            index += 1
            started = time.perf_counter()
            mask, target = detect(frame)
            cv_ms = (time.perf_counter() - started) * 1000
            timings.append(cv_ms)
            recent.append(cv_ms)
            misses += target is None
            now = time.perf_counter()
            displayed.append(now)
            playback_fps = ((len(displayed) - 1) / (displayed[-1] - displayed[0])
                            if len(displayed) > 1 else 0.0)
            if not args.headless or (args.snapshot and index == 300):
                preview = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if show_mask else frame
                annotate(preview, target, cv_ms, 1000 / np.mean(recent),
                         playback_fps, source_fps, index, misses)
                if args.snapshot and index == 300:
                    if not cv2.imwrite(args.snapshot, preview):
                        raise RuntimeError("Could not save snapshot")
                if not args.headless:
                    scale = min(1.0, 900 / preview.shape[0], 1400 / preview.shape[1])
                    cv2.imshow(WINDOW, cv2.resize(preview, None, fx=scale, fy=scale))
            if args.headless:
                continue
            delay = max(1, round((1 / source_fps - (time.perf_counter() - loop_start)) * 1000))
            key = cv2.waitKey(delay) & 0xFF
            if key == ord(" "):
                key = cv2.waitKey(0) & 0xFF
                displayed.clear()
            if key in (27, ord("q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key == ord("m"):
                show_mask = not show_mask
    finally:
        cap.release()
        if not args.headless:
            cv2.destroyAllWindows()
    if timings:
        print(f"Frames={len(timings)} empty={misses} | "
              f"CV mean={np.mean(timings):.2f}ms p95={np.percentile(timings, 95):.2f}ms "
              f"max={max(timings):.2f}ms", flush=True)
        print("Detection counts do NOT prove that the selected object is the tower.", flush=True)


if __name__ == "__main__":
    main()
