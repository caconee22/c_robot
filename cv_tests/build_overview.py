"""Render saved detections side by side, without rerunning any filter."""

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

from comparison_engine import METHODS, ROOT
from analyze_comparison import matches, read_annotations, row_target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "cv_comparison_phone")
    parser.add_argument("--annotations", type=Path, default=Path(__file__).with_name("annotations_phone.json"))
    args = parser.parse_args()
    info = json.loads((args.output / "summary.json").read_text(encoding="utf-8"))
    names = list(METHODS)
    rows = {name: {} for name in names}
    with (args.output / "metrics.csv").open(encoding="utf-8") as file:
        for row in csv.DictReader(file):
            rows[row["method"]][int(row["frame"])] = row
    annotation_info = json.loads(args.annotations.read_text(encoding="utf-8"))
    if annotation_info["source_sha256"] != info["source_sha256"]:
        raise RuntimeError("Annotations belong to another source video")
    labels = read_annotations(args.annotations,
                              info["source_width"], info["source_height"])
    cap = cv2.VideoCapture(info["source"])
    if not cap.isOpened():
        raise RuntimeError("Cannot read source")
    destination = args.output / "00_all_methods_overview.mp4"
    writer = cv2.VideoWriter(str(destination), cv2.VideoWriter_fourcc(*"mp4v"),
                             info["source_fps"], (960, 960))
    if not writer.isOpened():
        raise RuntimeError("Cannot create overview")
    cv2.setNumThreads(1)
    tile_w, tile_h = 240, 480
    index = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            scale = min(tile_w / frame.shape[1], 405 / frame.shape[0])
            source = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
            canvas = np.zeros((960, 960, 3), np.uint8)
            for tile, name in enumerate(names):
                row = rows[name][index]
                target = row_target(row)
                image = source.copy()
                if index in labels and labels[index]["box"]:
                    x, y, w, h = [round(v * scale) for v in labels[index]["box"]]
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 255), 1)
                if target:
                    x, y, w, h = [round(v * scale) for v in target["box"]]
                    cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    center = tuple(round(v * scale) for v in target["center"])
                    cv2.drawMarker(image, center, (0, 0, 255), cv2.MARKER_CROSS, 8, 1)
                if index in labels:
                    truth = labels[index]["box"]
                    status = ("CORRECT" if matches(target, truth) else "MISS/WRONG") if truth else (
                        "TRUE ABSENT" if target is None else "FALSE POSITIVE")
                else:
                    status = "FOUND - UNLABELED" if target else "NO DETECTION"
                tx, ty = (tile % 4) * tile_w, (tile // 4) * tile_h
                canvas[ty + 75:ty + 75 + image.shape[0], tx:tx + image.shape[1]] = image
                ms = float(row["detect_ms"])
                text = [name, f"CV {ms:.2f}ms | {1000 / max(ms, .0001):.1f} FPS",
                        f"{index / info['source_fps']:.2f}s #{index} {status}"]
                for line, value in enumerate(text):
                    cv2.putText(canvas, value, (tx + 4, ty + 19 + line * 22),
                                cv2.FONT_HERSHEY_SIMPLEX, .34, (255, 255, 255), 1, cv2.LINE_AA)
            writer.write(canvas)
            if index in (182, 1092, 2458, 3095, 4005, 4187):
                cv2.imwrite(str(args.output / f"overview_frame{index}.jpg"), canvas)
            index += 1
            if index % 1000 == 0:
                print(f"Overview {index}/{info['processed_frames']}", flush=True)
    finally:
        writer.release()
        cap.release()
    check = cv2.VideoCapture(str(destination))
    count = int(check.get(cv2.CAP_PROP_FRAME_COUNT))
    check.set(cv2.CAP_PROP_POS_FRAMES, index - 1)
    ok, _ = check.read()
    check.release()
    if count != info["processed_frames"] or index != count or not ok:
        raise RuntimeError(f"Incomplete overview: {index}/{count}")
    print(f"Verified overview: {destination}", flush=True)


if __name__ == "__main__":
    main()
