"""Five-frame end-to-end EOF, video, CSV and summary regression test."""

import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import cv2
import numpy as np

from comparison_engine import METHODS, ROOT
from video_frames import selected_frames


def main():
    output_root = ROOT / "outputs"
    output_root.mkdir(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix="cv_pipeline_test_", dir=output_root))
    source = folder / "source.avi"
    writer = cv2.VideoWriter(str(source), cv2.VideoWriter_fourcc(*"MJPG"), 30, (160, 120))
    if not writer.isOpened():
        raise RuntimeError("Cannot create smoke source")
    for number in range(5):
        frame = np.zeros((120, 160, 3), np.uint8)
        if number in (1, 2):
            cv2.rectangle(frame, (60, 30), (85, 80), (45, 150, 40), -1)
        writer.write(frame)
    writer.release()
    cap = cv2.VideoCapture(str(source))
    expected = []
    while True:
        ok, decoded = cap.read()
        if not ok:
            break
        expected.append(decoded)
    cap.release()
    selected = list(selected_frames(source, [4, 1, 3]))
    assert [index for index, _ in selected] == [1, 3, 4]
    assert all(np.array_equal(decoded, expected[index]) for index, decoded in selected)
    result = folder / "results"
    subprocess.run([sys.executable, str(Path(__file__).with_name("run_comparison.py")),
                    "--video", str(source), "--output", str(result)], check=True,
                   stdout=subprocess.DEVNULL)
    summary = json.loads((result / "summary.json").read_text(encoding="utf-8"))
    assert summary["processed_frames"] == 5
    assert summary["complete_source"]
    assert summary["source_width"] == 160 and summary["source_height"] == 120
    with (result / "metrics.csv").open(encoding="utf-8") as file:
        rows = list(csv.DictReader(file))
    assert len(rows) == 5 * len(METHODS)
    for name in METHODS:
        detections = [int(row["found"]) for row in rows if row["method"] == name]
        assert detections == [0, 1, 1, 0, 0], (name, detections)
        assert summary["methods"][name]["video_verified"]
    print(f"End-to-end EOF/video/CSV/summary check passed: {folder}")


if __name__ == "__main__":
    main()
