"""Compare the Pi detectors on the annotated 10/06 videos (sequential decode,
every frame run so tracking works, scored on the sparse manual samples).

  legacy : what the Pi ran until 11/04 (lightweight 4-filter, config json)
  08     : cv_tests 08_hsv_cascade, largest green area
  new    : scripts/tower_detector_hsv.py (08 + tall shape + tracking)

--scale 0.667 shrinks frames first to mimic the Pi's 1280x720 camera.
Usage: .venv/Scripts/python.exe cv_tests/evaluate_tower_hsv.py [--scale 1.0]
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "scripts"))
from analyze_comparison import matches, read_annotations  # noqa: E402
from comparison_engine import ROOT, detect as detect_08  # noqa: E402
from color_detection_lightweight import LightweightOptions, detect_frame_lightweight  # noqa: E402
from pi_green_tracker import DEFAULT_CONFIG, load_params  # noqa: E402
from tower_detector_hsv import TowerDetector  # noqa: E402

VIDEOS = (("phone", ROOT / "img" / "KakaoTalk_20261006_201439113.mp4", HERE / "annotations_phone.json"),
          ("210419904", ROOT / "img" / "KakaoTalk_20261006_210419904.mp4", HERE / "annotations_210419904.json"))


def as_target(candidate):
    if not candidate:
        return None
    return {"box": list(candidate["bbox"]), "center": list(candidate["centroid"])}


def run(name, path, labels_path, scale):
    capture = cv2.VideoCapture(str(path))
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH) * scale)
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT) * scale)
    labels = read_annotations(labels_path, width, height)
    params = load_params(DEFAULT_CONFIG)
    options = LightweightOptions(width // 2, height // 2)
    new = TowerDetector()
    stats = {k: {"hit": 0, "wrong": 0, "none": 0, "ms": []} for k in ("legacy", "08", "new")}
    absent = {k: 0 for k in stats}
    index, last = 0, max(labels)
    while index <= last:
        ok, frame = capture.read()
        if not ok:
            break
        if scale != 1.0:
            frame = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        small = cv2.resize(frame, (width // 2, height // 2), interpolation=cv2.INTER_AREA)
        label = labels.get(index)
        outs = {}
        t = time.perf_counter()
        r = detect_frame_lightweight(frame, params, options, search_frame=small, build_output_masks=False)
        stats["legacy"]["ms"].append((time.perf_counter() - t) * 1000)
        outs["legacy"] = as_target(r.candidates[0] if r.candidates else None)
        t = time.perf_counter()
        _m, _f, target = detect_08(frame, "08_hsv_cascade")
        stats["08"]["ms"].append((time.perf_counter() - t) * 1000)
        outs["08"] = target
        t = time.perf_counter()
        r = new.detect(frame, small)
        stats["new"]["ms"].append((time.perf_counter() - t) * 1000)
        outs["new"] = as_target(r.candidates[0] if r.candidates else None)
        if label is not None:
            for k, target in outs.items():
                if label["box"] is None:
                    absent[k] += target is not None
                elif target is None:
                    stats[k]["none"] += 1
                elif matches(target, label["box"]):
                    stats[k]["hit"] += 1
                else:
                    stats[k]["wrong"] += 1
        index += 1
    capture.release()
    total = sum(1 for v in labels.values() if v["box"] is not None)
    n_absent = sum(1 for v in labels.values() if v["box"] is None)
    print(f"\n{name}  {width}x{height}  frames run {index}, tower samples {total}, no-tower samples {n_absent}")
    print(f"{'method':8} {'hit':>5} {'wrong':>6} {'none':>5} {'false+':>7} {'mean ms':>8} {'p99 ms':>7}")
    for k, s in stats.items():
        ms = np.array(s["ms"][30:])
        print(f"{k:8} {s['hit']:>5} {s['wrong']:>6} {s['none']:>5} {absent[k]:>4}/{n_absent:<2} "
              f"{ms.mean():8.2f} {np.percentile(ms, 99):7.2f}")
    return stats


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scale", type=float, default=1.0)
    args = parser.parse_args()
    cv2.setNumThreads(1)
    for video in VIDEOS:
        run(*video, args.scale)
