from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

try:
    from color_detection_lightweight import LightweightOptions, detect_frame_lightweight
    from pi_green_tracker import DEFAULT_CONFIG, DEFAULT_VIDEO, draw_preview, load_params
except ModuleNotFoundError:
    from scripts.color_detection_lightweight import LightweightOptions, detect_frame_lightweight
    from scripts.pi_green_tracker import DEFAULT_CONFIG, DEFAULT_VIDEO, draw_preview, load_params


ROOT = Path(__file__).resolve().parents[1]


def export_full(source: Path, config: Path, output: Path, max_frames: int = 0) -> None:
    params = load_params(config)
    options = LightweightOptions()
    capture = cv2.VideoCapture(str(source))
    if not capture.isOpened():
        raise RuntimeError(f"영상을 열 수 없습니다: {source}")
    fps = float(capture.get(cv2.CAP_PROP_FPS) or 50.0)
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(output),
        cv2.VideoWriter_fourcc(*"mp4v"),
        fps,
        (1280, 720),
    )
    if not writer.isOpened():
        capture.release()
        raise RuntimeError(f"출력 파일을 열 수 없습니다: {output}")

    cv2.setUseOptimized(True)
    cv2.setNumThreads(4)
    loop_values: list[float] = []
    frame_index = 0
    started_all = time.perf_counter()
    try:
        while True:
            loop_started = time.perf_counter()
            ok, frame = capture.read()
            if not ok:
                break
            result = detect_frame_lightweight(
                frame,
                params,
                options,
                debug_masks=True,
                build_output_masks=False,
            )
            frame_index += 1
            elapsed_ms = (time.perf_counter() - loop_started) * 1000.0
            loop_values.append(elapsed_ms)
            composed = draw_preview(
                frame,
                result,
                float(result.timings_ms["total"]),
                loop_values,
                f"EXPORT {frame_index}/{total}",
                True,
                True,
                frame_index,
                total,
            )
            writer.write(composed)
            if max_frames > 0 and frame_index >= max_frames:
                break
            if frame_index % 100 == 0 or frame_index == total:
                elapsed = time.perf_counter() - started_all
                print(
                    f"export {frame_index}/{total} ({100.0 * frame_index / max(total, 1):.1f}%) "
                    f"speed={frame_index / max(elapsed, 1e-9):.1f}fps",
                    flush=True,
                )
    finally:
        capture.release()
        writer.release()

    elapsed = time.perf_counter() - started_all
    metadata = {
        "source": str(source),
        "output": str(output),
        "width": 1280,
        "height": 720,
        "fps": fps,
        "frames": frame_index,
        "elapsed_seconds": elapsed,
        "layout": "lightweight detection with 10 side masks",
    }
    output.with_suffix(".json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    print(f"done: {output}")


def main() -> None:
    parser = argparse.ArgumentParser(description="전체 경량 마스크 영상을 하나의 MP4로 저장")
    parser.add_argument("--source", type=Path, default=DEFAULT_VIDEO)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-frames", type=int, default=0, help="시험용 프레임 제한")
    args = parser.parse_args()
    export_full(args.source, args.config, args.output, args.max_frames)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("export interrupted", file=sys.stderr)
