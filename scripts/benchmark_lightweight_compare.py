from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from color_detection_core import DEFAULT_PARAMS, detect_frame
    from color_detection_lightweight import LightweightOptions, detect_frame_lightweight
except ModuleNotFoundError:
    from scripts.color_detection_core import DEFAULT_PARAMS, detect_frame
    from scripts.color_detection_lightweight import LightweightOptions, detect_frame_lightweight


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SOURCE = ROOT / "dataset" / "videos" / "irc_highlight_1080p50.mp4"
DEFAULT_CONFIG = ROOT / "config" / "color_detector_last.json"
OUTPUT_DIR = ROOT / "output" / "color_detector" / "benchmarks"


def load_params(path: Path) -> dict[str, Any]:
    params = dict(DEFAULT_PARAMS)
    if path.exists():
        params.update(json.loads(path.read_text(encoding="utf-8")))
    return params


def mask_iou(left: np.ndarray, right: np.ndarray) -> float:
    union = cv2.countNonZero(cv2.bitwise_or(left, right))
    if union == 0:
        return 1.0
    intersection = cv2.countNonZero(cv2.bitwise_and(left, right))
    return intersection / union


def center_distance(left: dict[str, Any] | None, right: dict[str, Any] | None) -> float | None:
    if left is None or right is None:
        return None
    lx, ly = left["centroid"]
    rx, ry = right["centroid"]
    return float(np.hypot(lx - rx, ly - ry))


def summarize(name: str, values: list[float]) -> dict[str, float]:
    data = np.asarray(values, dtype=np.float64)
    result = {
        "avg_ms": float(data.mean()),
        "p50_ms": float(np.percentile(data, 50)),
        "p95_ms": float(np.percentile(data, 95)),
        "p99_ms": float(np.percentile(data, 99)),
        "throughput_fps": float(1000.0 / max(data.mean(), 1e-9)),
        "over_20ms": int(np.count_nonzero(data > 20.0)),
        "over_33_33ms": int(np.count_nonzero(data > 33.3333)),
        "over_41_67ms": int(np.count_nonzero(data > 41.6667)),
    }
    print(f"\n{name}")
    for key, value in result.items():
        if isinstance(value, int):
            print(f"  {key}: {value}")
        else:
            print(f"  {key}: {value:.3f}")
    return result


def run(args: argparse.Namespace) -> None:
    params = load_params(args.config)
    options = LightweightOptions(
        search_width=args.search_width,
        search_height=args.search_height,
        max_rois=args.max_rois,
        roi_expand=args.roi_expand,
        roi_padding=args.roi_padding,
    )
    cap = cv2.VideoCapture(str(args.source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {args.source}")
    if args.start_frame:
        cap.set(cv2.CAP_PROP_POS_FRAMES, args.start_frame)

    rows: list[dict[str, Any]] = []
    baseline_times: list[float] = []
    light_times: list[float] = []
    frame_index = args.start_frame
    started_all = time.perf_counter()
    while len(rows) < args.max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        # Alternate order to reduce cache/thermal order bias.
        if len(rows) % 2 == 0:
            baseline = detect_frame(frame, params)
            light = detect_frame_lightweight(frame, params, options)
        else:
            light = detect_frame_lightweight(frame, params, options)
            baseline = detect_frame(frame, params)

        baseline_ms = float(baseline.timings_ms["total"])
        light_ms = float(light.timings_ms["total"])
        distance = center_distance(baseline.best, light.best)
        row = {
            "frame": frame_index,
            "baseline_ms": baseline_ms,
            "lightweight_ms": light_ms,
            "baseline_count": len(baseline.candidates),
            "lightweight_count": len(light.candidates),
            "presence_match": int(bool(baseline.best) == bool(light.best)),
            "count_match": int(len(baseline.candidates) == len(light.candidates)),
            "center_distance_px": "" if distance is None else distance,
            "selected_mask_iou": mask_iou(baseline.masks["selected"], light.masks["selected"]),
            "roi_count": int(light.timings_ms["roi_count"]),
        }
        rows.append(row)
        baseline_times.append(baseline_ms)
        light_times.append(light_ms)
        frame_index += args.frame_step
        if args.frame_step > 1:
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        if len(rows) % 25 == 0:
            print(f"processed {len(rows)}/{args.max_frames}", flush=True)
    cap.release()

    if not rows:
        raise RuntimeError("No frames were processed.")
    baseline_summary = summarize("BASELINE 1080p four-filter", baseline_times)
    light_summary = summarize("LIGHTWEIGHT low-res proposal + full-res ROI", light_times)
    speedup = baseline_summary["avg_ms"] / max(light_summary["avg_ms"], 1e-9)
    distances = [float(row["center_distance_px"]) for row in rows if row["center_distance_px"] != ""]
    accuracy = {
        "presence_match_percent": 100.0 * sum(row["presence_match"] for row in rows) / len(rows),
        "candidate_count_match_percent": 100.0 * sum(row["count_match"] for row in rows) / len(rows),
        "avg_selected_mask_iou": float(np.mean([row["selected_mask_iou"] for row in rows])),
        "avg_center_distance_px": float(np.mean(distances)) if distances else None,
        "p95_center_distance_px": float(np.percentile(distances, 95)) if distances else None,
        "avg_roi_count": float(np.mean([row["roi_count"] for row in rows])),
    }
    print("\nCOMPARISON")
    print(f"  speedup: {speedup:.3f}x")
    for key, value in accuracy.items():
        print(f"  {key}: {'n/a' if value is None else f'{value:.3f}'}")

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    csv_path = OUTPUT_DIR / f"lightweight_compare_{stamp}.csv"
    json_path = OUTPUT_DIR / f"lightweight_compare_{stamp}.json"
    with csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "source": str(args.source),
        "config": str(args.config),
        "start_frame": args.start_frame,
        "frame_step": args.frame_step,
        "frames": len(rows),
        "options": options.__dict__,
        "baseline": baseline_summary,
        "lightweight": light_summary,
        "speedup": speedup,
        "accuracy_vs_baseline": accuracy,
        "wall_seconds": time.perf_counter() - started_all,
    }
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"\nCSV: {csv_path}")
    print(f"JSON: {json_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare full-frame and lightweight green detectors.")
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--start-frame", type=int, default=12900)
    parser.add_argument("--max-frames", type=int, default=200)
    parser.add_argument("--frame-step", type=int, default=1, help="Sample every Nth input frame.")
    parser.add_argument("--search-width", type=int, default=960)
    parser.add_argument("--search-height", type=int, default=540)
    parser.add_argument("--max-rois", type=int, default=8)
    parser.add_argument("--roi-expand", type=float, default=2.0)
    parser.add_argument("--roi-padding", type=int, default=20)
    run(parser.parse_args())


if __name__ == "__main__":
    main()
