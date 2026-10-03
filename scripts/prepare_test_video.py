from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2


def find_default_source() -> Path:
    videos = sorted(Path("img").glob("*.mp4"))
    if not videos:
        raise FileNotFoundError("No MP4 video found in img/")
    return videos[0]


def open_writer(path: Path, fps: float, size: tuple[int, int]) -> tuple[cv2.VideoWriter, str]:
    # MP4V is tried first because some Windows OpenCV builds report an AV1/H.264
    # writer as open even when the external codec cannot build a seekable index.
    for codec in ("mp4v", "avc1"):
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*codec), fps, size)
        if writer.isOpened():
            return writer, codec
        writer.release()
    raise RuntimeError("Could not create an MP4 writer with avc1 or mp4v")


def convert_video(source: Path, output: Path, width: int, height: int, fps: float) -> None:
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open source video: {source}")

    source_fps = cap.get(cv2.CAP_PROP_FPS)
    source_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    source_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    source_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if source_fps <= 0 or source_frames <= 0:
        raise RuntimeError("Source video has invalid FPS or frame count")

    output.parent.mkdir(parents=True, exist_ok=True)
    temp_output = output.with_name(f"{output.stem}.partial{output.suffix}")
    if temp_output.exists():
        temp_output.unlink()
    writer, codec = open_writer(temp_output, fps, (width, height))

    duration = source_frames / source_fps
    output_frames = int(round(duration * fps))
    source_index = -1
    current_frame = None
    started = time.perf_counter()

    print(f"source: {source}")
    print(f"source: {source_width}x{source_height} @ {source_fps:.5f} fps, {source_frames} frames")
    print(f"output: {width}x{height} @ {fps:.2f} fps, {output_frames} frames")
    print(f"codec: {codec}")
    print("frame-rate conversion: timestamp-based frame repetition (no optical-flow interpolation)")

    try:
        for output_index in range(output_frames):
            wanted_source_index = min(source_frames - 1, int(math.floor(output_index * source_fps / fps)))
            while source_index < wanted_source_index:
                ok, frame = cap.read()
                if not ok:
                    raise RuntimeError(f"Source ended unexpectedly at frame {source_index + 1}")
                source_index += 1
                current_frame = frame

            if current_frame is None:
                ok, current_frame = cap.read()
                if not ok:
                    raise RuntimeError("Could not read the first source frame")
                source_index = 0

            if current_frame.shape[1] != width or current_frame.shape[0] != height:
                converted = cv2.resize(current_frame, (width, height), interpolation=cv2.INTER_LINEAR)
            else:
                converted = current_frame
            writer.write(converted)

            if output_index % 500 == 0 or output_index + 1 == output_frames:
                elapsed = time.perf_counter() - started
                progress = (output_index + 1) * 100.0 / output_frames
                rate = (output_index + 1) / max(elapsed, 1e-6)
                print(f"{progress:6.2f}%  frame={output_index + 1}/{output_frames}  encode_fps={rate:.1f}")
    finally:
        cap.release()
        writer.release()

    verify = cv2.VideoCapture(str(temp_output))
    metadata = {
        "source": str(source),
        "source_width": source_width,
        "source_height": source_height,
        "source_fps": source_fps,
        "source_frames": source_frames,
        "output_width": int(verify.get(cv2.CAP_PROP_FRAME_WIDTH)),
        "output_height": int(verify.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        "output_fps": verify.get(cv2.CAP_PROP_FPS),
        "output_frames": int(verify.get(cv2.CAP_PROP_FRAME_COUNT)),
        "codec": codec,
        "fps_method": "timestamp-based frame repetition",
    }
    verify.release()

    if metadata["output_width"] != width or metadata["output_height"] != height:
        raise RuntimeError(f"Converted video size verification failed: {metadata}")
    if abs(metadata["output_fps"] - fps) > 0.1:
        raise RuntimeError(f"Converted video FPS verification failed: {metadata}")
    if abs(metadata["output_frames"] - output_frames) > 1:
        raise RuntimeError(f"Converted video frame-count verification failed: {metadata}")

    if output.exists():
        output.unlink()
    temp_output.replace(output)
    metadata_path = output.with_suffix(".json")
    metadata_path.write_text(json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"done: {output}")
    print(f"metadata: {metadata_path}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Create a deterministic 1080p50 CV test video.")
    parser.add_argument("--source", type=Path, default=None)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("dataset/videos/irc_highlight_1080p50.mp4"),
    )
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--fps", type=float, default=50.0)
    args = parser.parse_args()
    convert_video(args.source or find_default_source(), args.output, args.width, args.height, args.fps)


if __name__ == "__main__":
    main()
