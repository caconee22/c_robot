"""Process every source frame with all methods; save masks, videos and raw metrics."""

import argparse
import csv
import json
from pathlib import Path
import time

import cv2
import numpy as np

from comparison_engine import METHODS, ROOT, VIDEO, detect, render
from finalize_results import finalize
from video_frames import selected_frames


def contact_sheets(video, output):
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {video}")
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.release()
    indices = np.linspace(0, total - 1, min(48, total), dtype=int).tolist()
    output.mkdir(parents=True, exist_ok=True)
    sheet = None
    for j, (index, frame) in enumerate(selected_frames(video, indices)):
        if j % 6 == 0:
            sheet = np.zeros((1280, 1080, 3), np.uint8)
        preview = cv2.resize(frame, (360, 640))
        cv2.rectangle(preview, (0, 0), (359, 35), (0, 0, 0), -1)
        cv2.putText(preview, f"frame={index}  original {frame.shape[1]}x{frame.shape[0]}", (5, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
        row, col = (j % 6) // 3, j % 3
        sheet[row * 640:(row + 1) * 640, col * 360:(col + 1) * 360] = preview
        if j % 6 == 5 or j == len(indices) - 1:
            cv2.imwrite(str(output / f"reference_{j // 6:02d}.jpg"), sheet)
    (output / "reference_indices.json").write_text(json.dumps(indices), encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--video", type=Path, default=VIDEO)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "cv_comparison_phone")
    parser.add_argument("--references", action="store_true")
    parser.add_argument("--limit", type=int, default=0, help="0 means entire video")
    parser.add_argument("--threads", type=int, default=1)
    args = parser.parse_args()
    if args.references:
        contact_sheets(args.video, args.output / "references")
        return
    args.output.mkdir(parents=True, exist_ok=True)
    # Explicit opt-in to a fresh directory avoids overwriting previous evidence.
    if (args.output / "metrics.csv").exists():
        raise SystemExit("Output already contains metrics.csv; use a new --output directory")
    cv2.setNumThreads(args.threads)
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise SystemExit(f"Cannot open {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    names = list(METHODS)
    writers = {}
    start = time.perf_counter()
    index = 0
    fields = ["frame", "time_s", "method", "detect_ms", "render_encode_ms", "found",
              "x", "y", "w", "h", "cx", "cy", "area", "mask_pixels"]
    try:
        for name in names:
            writer = cv2.VideoWriter(str(args.output / f"{name}.mp4"),
                                     cv2.VideoWriter_fourcc(*"mp4v"), fps, (810, 960))
            if not writer.isOpened():
                raise RuntimeError(f"Cannot create video for {name}")
            writers[name] = writer
        with (args.output / "metrics.csv").open("w", newline="", encoding="utf-8") as file:
            rows = csv.DictWriter(file, fieldnames=fields)
            rows.writeheader()
            while args.limit == 0 or index < args.limit:
                ok, frame = cap.read()
                if not ok:
                    break
                # Rotate execution order; no shared precomputed color spaces or masks.
                order = names[index % len(names):] + names[:index % len(names)]
                for name in order:
                    started = time.perf_counter()
                    masks, fused, target = detect(frame, name)
                    elapsed = (time.perf_counter() - started) * 1000
                    started = time.perf_counter()
                    preview = render(frame, masks, fused, target, name, index, elapsed, fps)
                    writers[name].write(preview)
                    render_ms = (time.perf_counter() - started) * 1000
                    row = dict(frame=index, time_s=index / fps, method=name, detect_ms=elapsed,
                               render_encode_ms=render_ms, found=int(target is not None),
                               mask_pixels=int(cv2.countNonZero(fused)))
                    if target:
                        row.update(zip(("x", "y", "w", "h"), target["box"]))
                        row.update(zip(("cx", "cy"), target["center"]))
                        row["area"] = target["area"]
                    rows.writerow(row)
                    if index in (0, 300, 1500, 3000):
                        cv2.imwrite(str(args.output / f"{name}_frame{index}.jpg"), preview)
                index += 1
                if index % 100 == 0:
                    file.flush()
                    print(f"{index}/{min(total, args.limit) if args.limit else total} "
                          f"frames | elapsed {time.perf_counter() - start:.1f}s", flush=True)
    finally:
        cap.release()
        for writer in writers.values():
            writer.release()
    if not index:
        raise RuntimeError("Source contained no decodable frames")
    info = finalize(args.output, args.video, cv2.getNumThreads(), time.perf_counter() - start)
    print(json.dumps(info["methods"], indent=2), flush=True)


if __name__ == "__main__":
    main()
