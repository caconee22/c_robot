from __future__ import annotations

import argparse
import json
import time
from datetime import datetime
from pathlib import Path
from threading import Event
from typing import Any, Callable

import cv2
import numpy as np

try:
    from color_detection_core import DEFAULT_PARAMS, DetectionResult, detect_frame
except ModuleNotFoundError:
    from scripts.color_detection_core import DEFAULT_PARAMS, DetectionResult, detect_frame


TILE_WIDTH = 640
TILE_HEIGHT = 360
EXPORT_SIZE = (TILE_WIDTH * 3, TILE_HEIGHT * 3)


def load_params(path: Path) -> dict[str, Any]:
    params = dict(DEFAULT_PARAMS)
    if path.exists():
        params.update(json.loads(path.read_text(encoding="utf-8")))
    return params


def default_source() -> Path:
    converted = Path("dataset/videos/irc_highlight_1080p50.mp4")
    if converted.exists():
        return converted
    videos = sorted(Path("img").glob("*.mp4"))
    if not videos:
        raise FileNotFoundError("No MP4 source video was found")
    return videos[0]


def labelled_image(image: np.ndarray, label: str) -> np.ndarray:
    tile = cv2.resize(image, (TILE_WIDTH, TILE_HEIGHT), interpolation=cv2.INTER_AREA)
    cv2.rectangle(tile, (0, 0), (TILE_WIDTH, 38), (0, 0, 0), -1)
    cv2.putText(tile, label, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 255, 255), 2, cv2.LINE_AA)
    return tile


def labelled_mask(mask: np.ndarray, label: str) -> np.ndarray:
    if mask.dtype != np.uint8:
        maximum = max(1, int(mask.max()))
        mask = np.clip(mask.astype(np.float32) * (255.0 / maximum), 0, 255).astype(np.uint8)
    return labelled_image(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), label)


def annotated_frame(frame: np.ndarray, result: DetectionResult, frame_index: int) -> np.ndarray:
    output = frame.copy()
    colors = [(0, 255, 255), (255, 0, 255)]
    for index, candidate in enumerate(result.candidates[:2]):
        color = colors[index]
        x, y, w, h = candidate["bbox"]
        cx, cy = candidate["centroid"]
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 3)
        cv2.circle(output, (int(cx), int(cy)), 8, color, -1)
        cv2.putText(
            output,
            f"#{index + 1} area={candidate['area']} conf={candidate['color_confidence']:.2f}",
            (x, max(26, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.70,
            color,
            2,
            cv2.LINE_AA,
        )
    cv2.putText(
        output,
        f"frame={frame_index} total={result.timings_ms['total']:.2f}ms targets={len(result.candidates)}",
        (16, output.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    return output


def build_export_frame(frame: np.ndarray, result: DetectionResult, frame_index: int) -> np.ndarray:
    row1 = cv2.hconcat(
        [
            labelled_image(annotated_frame(frame, result, frame_index), "DETECTION"),
            labelled_mask(result.masks["cleaned"], "FINAL CLEANED MASK"),
            labelled_mask(result.masks["selected"], "SELECTED TARGETS"),
        ]
    )
    row2 = cv2.hconcat(
        [
            labelled_mask(result.masks["bgr"], "BGR"),
            labelled_mask(result.masks["exg"], "ExG + DOMINANCE GUARD"),
            labelled_mask(result.masks["hsv"], "HSV"),
        ]
    )
    row3 = cv2.hconcat(
        [
            labelled_mask(result.masks["lab"], "Lab"),
            labelled_mask(result.masks["core"], "STRICT CORE"),
            labelled_mask(result.masks["support"], "SUPPORT / VOTES"),
        ]
    )
    return cv2.vconcat([row1, row2, row3])


def export_video(
    source: Path,
    output: Path,
    params: dict[str, Any],
    progress_callback: Callable[[int, int, float], None] | None = None,
    cancel_event: Event | None = None,
    max_frames: int | None = None,
) -> dict[str, Any]:
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {source}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 50.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    output.parent.mkdir(parents=True, exist_ok=True)
    temp = output.with_name(f"{output.stem}.partial{output.suffix}")
    if temp.exists():
        temp.unlink()
    writer = cv2.VideoWriter(str(temp), cv2.VideoWriter_fourcc(*"mp4v"), fps, EXPORT_SIZE)
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"Could not create output video: {temp}")

    started = time.perf_counter()
    frame_index = 0
    cancelled = False
    try:
        while True:
            if cancel_event is not None and cancel_event.is_set():
                cancelled = True
                break
            ok, frame = cap.read()
            if not ok:
                break
            frame_index += 1
            result = detect_frame(frame, params)
            writer.write(build_export_frame(frame, result, frame_index))
            if progress_callback and (frame_index % 25 == 0 or frame_index == total_frames):
                progress_callback(frame_index, total_frames, time.perf_counter() - started)
            if max_frames and frame_index >= max_frames:
                break
    finally:
        cap.release()
        writer.release()

    elapsed = time.perf_counter() - started
    if cancelled:
        cancelled_path = output.with_name(f"{output.stem}.cancelled{output.suffix}")
        if cancelled_path.exists():
            cancelled_path.unlink()
        temp.replace(cancelled_path)
        final_path = cancelled_path
    else:
        if output.exists():
            output.unlink()
        temp.replace(output)
        final_path = output

    metadata = {
        "source": str(source),
        "output": str(final_path),
        "width": EXPORT_SIZE[0],
        "height": EXPORT_SIZE[1],
        "fps": fps,
        "frames": frame_index,
        "elapsed_seconds": elapsed,
        "cancelled": cancelled,
        "params": params,
        "layout": [
            ["detection", "final_cleaned_mask", "selected_targets"],
            ["bgr", "exg", "hsv"],
            ["lab", "strict_core", "support_votes"],
        ],
    }
    final_path.with_suffix(".json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    return metadata


def main() -> None:
    parser = argparse.ArgumentParser(description="Export a 3x3 green-detector review video.")
    parser.add_argument("--source", type=Path, default=None)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path("config/color_detector_last.json"),
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--max-frames", type=int, default=None)
    args = parser.parse_args()

    source = args.source or default_source()
    output = args.output or Path("output/color_detector/exports") / (
        f"green_detector_all_masks_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
    )
    params = load_params(args.config if args.config.exists() else Path("config/color_detector_defaults.json"))

    def report(current: int, total: int, elapsed: float) -> None:
        percent = current * 100.0 / max(total, 1)
        speed = current / max(elapsed, 1e-6)
        print(f"{percent:6.2f}%  frame={current}/{total}  export_fps={speed:.2f}", flush=True)

    metadata = export_video(source, output, params, progress_callback=report, max_frames=args.max_frames)
    print(f"done: {metadata['output']}")
    print(f"elapsed_seconds: {metadata['elapsed_seconds']:.2f}")


if __name__ == "__main__":
    main()
