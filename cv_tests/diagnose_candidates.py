"""Distinguish losing the color signal from choosing the wrong largest blob."""

import csv
import argparse
import json
from pathlib import Path

import cv2

from analyze_comparison import matches, read_annotations
from comparison_engine import METHODS, MIN_AREA, ROOT, detect
from video_frames import selected_frames


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "cv_comparison_phone")
    parser.add_argument("--annotations", type=Path, default=Path(__file__).with_name("annotations_phone.json"))
    args = parser.parse_args()
    output = args.output
    info = json.loads((output / "summary.json").read_text(encoding="utf-8"))
    annotation_info = json.loads(args.annotations.read_text(encoding="utf-8"))
    if annotation_info["source_sha256"] != info["source_sha256"]:
        raise RuntimeError("Annotations belong to another source video")
    labels = read_annotations(args.annotations,
                              info["source_width"], info["source_height"])
    cv2.setNumThreads(1)
    counts = {name: {"visible": 0, "target_candidate_present": 0,
                     "largest_correct": 0, "candidate_present_but_wrong_selection": 0,
                     "target_candidate_missing": 0} for name in METHODS}
    records = []
    for index, frame in selected_frames(info["source"], [i for i, item in labels.items() if item["box"]]):
        label = labels[index]
        for name in METHODS:
            _, mask, target = detect(frame, name)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            any_match = False
            for contour in contours:
                if cv2.contourArea(contour) < MIN_AREA:
                    continue
                box = list(cv2.boundingRect(contour))
                m = cv2.moments(contour)
                candidate = {"box": box, "center": [m["m10"] / m["m00"], m["m01"] / m["m00"]]}
                if matches(candidate, label["box"]):
                    any_match = True
                    break
            correct = matches(target, label["box"])
            item = counts[name]
            item["visible"] += 1
            item["target_candidate_present"] += any_match
            item["largest_correct"] += correct
            item["candidate_present_but_wrong_selection"] += any_match and not correct
            item["target_candidate_missing"] += not any_match
            records.append({"frame": index, "method": name, "target_candidate_present": int(any_match),
                            "largest_correct": int(correct)})
    (output / "candidate_diagnosis.json").write_text(json.dumps(counts, indent=2), encoding="utf-8")
    with (output / "candidate_diagnosis.csv").open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=records[0].keys())
        writer.writeheader()
        writer.writerows(records)
    print(json.dumps(counts, indent=2), flush=True)


if __name__ == "__main__":
    main()
