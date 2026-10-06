"""Decode every saved video frame, not merely the first and last."""

import argparse
import json
from pathlib import Path

import cv2

from comparison_engine import METHODS, ROOT


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "cv_comparison_phone")
    args = parser.parse_args()
    info = json.loads((args.output / "summary.json").read_text(encoding="utf-8"))
    files = [args.output / f"{name}.mp4" for name in METHODS]
    overview = args.output / "00_all_methods_overview.mp4"
    if overview.exists():
        files.append(overview)
    results = {}
    for path in files:
        cap = cv2.VideoCapture(str(path))
        if not cap.isOpened():
            raise RuntimeError(f"Cannot read {path}")
        count = 0
        expected = (960, 960) if path == overview else (960, 810)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if frame.shape[:2] != expected:
                raise RuntimeError(f"Wrong frame shape at {path.name}:{count}")
            count += 1
        cap.release()
        if count != info["processed_frames"]:
            raise RuntimeError(f"Incomplete decode {path.name}: {count}/{info['processed_frames']}")
        results[path.name] = {"decoded_frames": count, "bytes": path.stat().st_size}
        print(f"Verified ALL {count} frames: {path.name}", flush=True)
    (args.output / "video_verification.json").write_text(json.dumps(results, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
