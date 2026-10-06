"""Validate saved metrics/videos and rebuild a summary without rerunning detection."""

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from comparison_engine import METHODS, ROOT, VIDEO, HSV_LOW, HSV_HIGH, LAB_LOW, LAB_HIGH, MIN_AREA


def finalize(output, video, threads=1, wall_seconds=None):
    timings = {name: [] for name in METHODS}
    found = {name: 0 for name in METHODS}
    indices = {name: [] for name in METHODS}
    with (output / "metrics.csv").open(encoding="utf-8") as file:
        for row in csv.DictReader(file):
            name = row["method"]
            timings[name].append(float(row["detect_ms"]))
            found[name] += int(row["found"])
            indices[name].append(int(row["frame"]))
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open source {video}")
    fps = cap.get(cv2.CAP_PROP_FPS)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    source_width, source_height = int(cap.get(3)), int(cap.get(4))
    cap.release()
    processed = len(timings[next(iter(METHODS))])
    if not processed:
        raise RuntimeError("No complete metrics")
    summaries = {}
    for name in METHODS:
        if indices[name] != list(range(processed)):
            raise RuntimeError(f"Missing/duplicate frame metrics: {name}")
        values = np.asarray(timings[name][30:] or timings[name])
        if not np.isfinite(values).all() or (values <= 0).any():
            raise RuntimeError(f"Invalid timing data: {name}")
        check = cv2.VideoCapture(str(output / f"{name}.mp4"))
        count = int(check.get(cv2.CAP_PROP_FRAME_COUNT))
        width, height = int(check.get(3)), int(check.get(4))
        output_fps = check.get(cv2.CAP_PROP_FPS)
        first, _ = check.read()
        check.set(cv2.CAP_PROP_POS_FRAMES, processed - 1)
        last, _ = check.read()
        check.release()
        if count != processed or not first or not last or (width, height) != (810, 960) or abs(output_fps - fps) > .01:
            raise RuntimeError(f"Output verification failed: {name}")
        summaries[name] = {"frames": processed, "found": found[name], "empty": processed - found[name],
                           "mean_ms": float(values.mean()), "p95_ms": float(np.percentile(values, 95)),
                           "p99_ms": float(np.percentile(values, 99)), "max_ms": float(values.max()),
                           "over20_percent": float((values > 20).mean() * 100),
                           "cv_capacity_fps": float(1000 / values.mean()), "video_verified": True}
    info = {"source": str(video.resolve()), "source_sha256": hashlib.sha256(video.read_bytes()).hexdigest(),
            "source_fps": fps, "source_frames": total, "processed_frames": processed,
            "complete_source": processed == total,
            "source_width": source_width, "source_height": source_height, "opencv": cv2.__version__,
            "opencv_threads": threads, "warmup_excluded": 30 if processed > 30 else 0,
            "wall_seconds": wall_seconds, "methods": summaries,
            "config": {"hsv_low": HSV_LOW, "hsv_high": HSV_HIGH, "lab_low": LAB_LOW,
                       "lab_high": LAB_HIGH, "min_area": MIN_AREA},
            "note": "Found does not prove correct tower detection. See independent annotations."}
    (output / "summary.json").write_text(json.dumps(info, indent=2), encoding="utf-8")
    return info


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "cv_comparison_phone")
    parser.add_argument("--video", type=Path, default=VIDEO)
    parser.add_argument("--threads", type=int, default=1)
    args = parser.parse_args()
    print(json.dumps(finalize(args.output, args.video, args.threads), indent=2), flush=True)


if __name__ == "__main__":
    main()
