"""Single-file green target tracker for Raspberry Pi 5 + Camera Module 3 Wide."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import cv2
import numpy as np


# Camera and performance settings
MAIN_SIZE = (1920, 1080)
SEARCH_SIZE = (960, 540)
CAMERA_FPS = 50.0
OPENCV_THREADS = 4

# ROI proposal settings
MAX_ROIS = 12
ROI_EXPAND = 2.0
ROI_PADDING = 24
PROPOSAL_MIN_AREA = 2

# Final target settings. Tiny targets are accepted only with stricter color agreement.
MIN_AREA = 8
NORMAL_AREA = 300
TINY_MIN_CONFIDENCE = 0.90
TINY_MIN_VOTE4_RATIO = 0.60
SECOND_TINY_MIN_CONFIDENCE = 0.97
SECOND_TINY_MIN_VOTE4_RATIO = 0.85
SECOND_SMALL_AREA_RATIO = 0.15
MAX_TARGETS = 2

# Tuned green thresholds
G_MIN = 40
GR_MIN = 20
GB_MIN = 20
EXG_MIN = 45
EXG_G_MIN = 35
H_MIN, H_MAX = 45, 56
S_MIN, S_MAX = 138, 255
V_MIN, V_MAX = 61, 255
LAB_L_MIN, LAB_L_MAX = 50, 255
LAB_A_MIN, LAB_A_MAX = 64, 110
LAB_B_MIN, LAB_B_MAX = 147, 206


def direct_proposal(frame: np.ndarray) -> np.ndarray:
    """Cheap BGR + ExG proposal mask."""
    b, g, r = cv2.split(frame)
    gr = cv2.subtract(g, r)
    gb = cv2.subtract(g, b)
    dominance = cv2.bitwise_and(
        cv2.inRange(gr, GR_MIN, 255),
        cv2.inRange(gb, GB_MIN, 255),
    )
    bgr = cv2.bitwise_and(cv2.inRange(g, G_MIN, 255), dominance)

    exg = cv2.addWeighted(g, 2.0, r, -1.0, 0.0, dtype=cv2.CV_16S)
    exg = cv2.subtract(exg, b, dtype=cv2.CV_16S)
    exg = cv2.bitwise_and(cv2.inRange(exg, EXG_MIN, 32767), cv2.inRange(g, EXG_G_MIN, 255))
    exg = cv2.bitwise_and(exg, dominance)
    proposal = cv2.bitwise_or(bgr, exg)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(proposal, cv2.MORPH_CLOSE, kernel, iterations=1)


def merge_rois(rois: list[tuple[int, int, int, int]]) -> list[tuple[int, int, int, int]]:
    merged: list[tuple[int, int, int, int]] = []
    for x, y, w, h in rois:
        right, bottom = x + w, y + h
        changed = True
        while changed:
            changed = False
            keep: list[tuple[int, int, int, int]] = []
            for ox, oy, ow, oh in merged:
                oright, obottom = ox + ow, oy + oh
                if x < oright and ox < right and y < obottom and oy < bottom:
                    x, y = min(x, ox), min(y, oy)
                    right, bottom = max(right, oright), max(bottom, obottom)
                    changed = True
                else:
                    keep.append((ox, oy, ow, oh))
            merged = keep
        merged.append((x, y, right - x, bottom - y))
    return merged


def proposal_rois(mask: np.ndarray, full_shape: tuple[int, int]) -> list[tuple[int, int, int, int]]:
    full_h, full_w = full_shape
    low_h, low_w = mask.shape
    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    components: list[tuple[int, int, int, int, int]] = []
    for label in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[label])
        if area >= PROPOSAL_MIN_AREA:
            components.append((area, x, y, w, h))
    components.sort(reverse=True)

    sx, sy = full_w / low_w, full_h / low_h
    rois: list[tuple[int, int, int, int]] = []
    for _area, x, y, w, h in components[:MAX_ROIS]:
        cx, cy = (x + w / 2) * sx, (y + h / 2) * sy
        rw = max(1, round(w * sx * ROI_EXPAND)) + ROI_PADDING * 2
        rh = max(1, round(h * sy * ROI_EXPAND)) + ROI_PADDING * 2
        left = max(0, round(cx - rw / 2))
        top = max(0, round(cy - rh / 2))
        right = min(full_w, left + rw)
        bottom = min(full_h, top + rh)
        left, top = max(0, right - rw), max(0, bottom - rh)
        rois.append((left, top, right - left, bottom - top))
    return merge_rois(rois)


def four_filter_masks(roi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Return Vote-3 mask and per-pixel agreement count (0..4)."""
    b, g, r = cv2.split(roi)
    gr = cv2.subtract(g, r)
    gb = cv2.subtract(g, b)
    dominance = cv2.bitwise_and(cv2.inRange(gr, GR_MIN, 255), cv2.inRange(gb, GB_MIN, 255))
    bgr = cv2.bitwise_and(cv2.inRange(g, G_MIN, 255), dominance)

    exg_value = cv2.addWeighted(g, 2.0, r, -1.0, 0.0, dtype=cv2.CV_16S)
    exg_value = cv2.subtract(exg_value, b, dtype=cv2.CV_16S)
    exg = cv2.bitwise_and(cv2.inRange(exg_value, EXG_MIN, 32767), cv2.inRange(g, EXG_G_MIN, 255))
    exg = cv2.bitwise_and(exg, dominance)

    hsv_image = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    hsv = cv2.inRange(
        hsv_image,
        np.array([H_MIN, S_MIN, V_MIN], dtype=np.uint8),
        np.array([H_MAX, S_MAX, V_MAX], dtype=np.uint8),
    )
    lab_image = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    lab = cv2.inRange(
        lab_image,
        np.array([LAB_L_MIN, LAB_A_MIN, LAB_B_MIN], dtype=np.uint8),
        np.array([LAB_L_MAX, LAB_A_MAX, LAB_B_MAX], dtype=np.uint8),
    )

    votes = cv2.add(bgr, exg, dtype=cv2.CV_16U)
    votes = cv2.add(votes, hsv, dtype=cv2.CV_16U)
    votes = cv2.add(votes, lab, dtype=cv2.CV_16U)
    agreement = (votes // 255).astype(np.uint8)
    fused = cv2.inRange(votes, 3 * 255, 4 * 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    return cv2.morphologyEx(fused, cv2.MORPH_CLOSE, kernel, iterations=1), agreement


def roi_candidates(
    mask: np.ndarray,
    agreement: np.ndarray,
    offset_x: int,
    offset_y: int,
) -> list[dict]:
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    candidates: list[dict] = []
    for label in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[label])
        if area < MIN_AREA:
            continue
        component = (labels[y : y + h, x : x + w] == label).astype(np.uint8) * 255
        local_agreement = agreement[y : y + h, x : x + w]
        confidence = float(cv2.mean(local_agreement, mask=component)[0] / 4.0)
        vote4 = cv2.inRange(local_agreement, 4, 4)
        vote4_pixels = cv2.countNonZero(cv2.bitwise_and(vote4, component))
        vote4_ratio = vote4_pixels / max(area, 1)

        if area < NORMAL_AREA and (
            confidence < TINY_MIN_CONFIDENCE or vote4_ratio < TINY_MIN_VOTE4_RATIO
        ):
            continue
        cx, cy = centroids[label]
        candidates.append(
            {
                "area": area,
                "bbox": (x + offset_x, y + offset_y, w, h),
                "centroid": (float(cx + offset_x), float(cy + offset_y)),
                "confidence": confidence,
                "vote4_ratio": vote4_ratio,
            }
        )
    return candidates


def detect(frame: np.ndarray, search_frame: np.ndarray | None = None) -> tuple[list[dict], dict[str, float]]:
    started = time.perf_counter()
    full_h, full_w = frame.shape[:2]
    if search_frame is None:
        search_frame = cv2.resize(frame, SEARCH_SIZE, interpolation=cv2.INTER_AREA)
    proposal_started = time.perf_counter()
    proposal = direct_proposal(search_frame)
    rois = proposal_rois(proposal, (full_h, full_w))
    proposal_ms = (time.perf_counter() - proposal_started) * 1000

    verify_started = time.perf_counter()
    candidates: list[dict] = []
    for x, y, w, h in rois:
        mask, agreement = four_filter_masks(frame[y : y + h, x : x + w])
        candidates.extend(roi_candidates(mask, agreement, x, y))
    candidates.sort(key=lambda item: item["area"], reverse=True)
    selected: list[dict] = []
    for candidate in candidates:
        if selected:
            is_small_second = (
                candidate["area"] < NORMAL_AREA
                or candidate["area"] < selected[0]["area"] * SECOND_SMALL_AREA_RATIO
            )
            if is_small_second and (
                candidate["confidence"] < SECOND_TINY_MIN_CONFIDENCE
                or candidate["vote4_ratio"] < SECOND_TINY_MIN_VOTE4_RATIO
            ):
                continue
        selected.append(candidate)
        if len(selected) >= MAX_TARGETS:
            break
    candidates = selected
    verify_ms = (time.perf_counter() - verify_started) * 1000
    return candidates, {
        "proposal_ms": proposal_ms,
        "verify_ms": verify_ms,
        "total_ms": (time.perf_counter() - started) * 1000,
        "roi_count": float(len(rois)),
    }


class VideoInput:
    def __init__(self, path: str, start_frame: int) -> None:
        self.capture = cv2.VideoCapture(path)
        if not self.capture.isOpened():
            raise RuntimeError(f"Cannot open video: {path}")
        if start_frame > 0:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    def read(self):
        ok, frame = self.capture.read()
        return ok, frame, None

    def close(self) -> None:
        self.capture.release()


class PiCameraInput:
    def __init__(self) -> None:
        try:
            from picamera2 import Picamera2
        except ModuleNotFoundError as exc:
            raise RuntimeError("Picamera2 is required on Raspberry Pi OS.") from exc
        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={"size": MAIN_SIZE, "format": "RGB888"},
            lores={"size": SEARCH_SIZE, "format": "RGB888"},
            controls={"FrameRate": CAMERA_FPS},
            buffer_count=4,
        )
        self.camera.configure(config)
        self.camera.start()

    def read(self):
        request = self.camera.capture_request()
        try:
            frame = request.make_array("main")
            search = request.make_array("lores")
        finally:
            request.release()
        return True, frame, search

    def close(self) -> None:
        self.camera.stop()
        self.camera.close()


def print_result(frame_number: int, candidates: list[dict], timings: dict[str, float], as_json: bool) -> None:
    targets = [
        {
            "id": index + 1,
            "x": round(item["centroid"][0], 1),
            "y": round(item["centroid"][1], 1),
            "area": item["area"],
            "confidence": round(item["confidence"], 3),
        }
        for index, item in enumerate(candidates)
    ]
    if as_json:
        print(json.dumps({"frame": frame_number, "process_ms": round(timings["total_ms"], 2), "targets": targets}), flush=True)
    else:
        text = " ".join(
            f"#{target['id']} x={target['x']:.1f} y={target['y']:.1f} area={target['area']} conf={target['confidence']:.2f}"
            for target in targets
        )
        print(f"frame={frame_number} {timings['total_ms']:.2f}ms targets={len(targets)} {text}".rstrip(), flush=True)


def draw_preview(frame: np.ndarray, candidates: list[dict], timings: dict[str, float], fps: float) -> np.ndarray:
    output = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
    sx, sy = 1280 / frame.shape[1], 720 / frame.shape[0]
    colors = [(0, 255, 255), (255, 0, 255)]
    for index, item in enumerate(candidates):
        x, y, w, h = item["bbox"]
        cx, cy = item["centroid"]
        x, y, w, h = round(x * sx), round(y * sy), round(w * sx), round(h * sy)
        cx, cy = round(cx * sx), round(cy * sy)
        color = colors[index]
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 3)
        cv2.circle(output, (cx, cy), 6, color, -1)
        cv2.putText(output, f"#{index + 1} ({cx},{cy})", (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
    cv2.rectangle(output, (0, 0), (580, 82), (0, 0, 0), -1)
    cv2.putText(output, f"DETECT {timings['total_ms']:.2f} ms   {fps:.1f} FPS", (15, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (0, 255, 255), 2, cv2.LINE_AA)
    cv2.putText(output, f"ROIs {int(timings['roi_count'])}   Q quit", (15, 67), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 255), 2, cv2.LINE_AA)
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Single-file Raspberry Pi green tracker")
    parser.add_argument("--source", default="camera", help="camera or video path")
    parser.add_argument("--preview", action="store_true")
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--quiet", action="store_true")
    parser.add_argument("--print-every", type=int, default=1)
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()

    cv2.setUseOptimized(True)
    cv2.setNumThreads(OPENCV_THREADS)
    if hasattr(cv2, "ocl"):
        cv2.ocl.setUseOpenCL(False)

    source = PiCameraInput() if args.source.lower() == "camera" else VideoInput(args.source, args.start_frame)
    frame_number = 0
    loop_times: list[float] = []
    if args.preview:
        cv2.namedWindow("Green tracker", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Green tracker", 1280, 720)
    try:
        while True:
            loop_started = time.perf_counter()
            ok, frame, search = source.read()
            if not ok:
                break
            candidates, timings = detect(frame, search)
            frame_number += 1
            loop_ms = (time.perf_counter() - loop_started) * 1000
            loop_times.append(loop_ms)
            recent = loop_times[-60:]
            fps = 1000.0 / max(sum(recent) / len(recent), 1e-9)
            if not args.quiet and frame_number % max(1, args.print_every) == 0:
                print_result(frame_number, candidates, timings, args.json)
            if args.preview:
                cv2.imshow("Green tracker", draw_preview(frame, candidates, timings, fps))
                key = cv2.pollKey() if hasattr(cv2, "pollKey") else cv2.waitKey(1)
                if key & 0xFF == ord("q"):
                    break
            if args.max_frames > 0 and frame_number >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
