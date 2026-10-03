from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import cv2
import numpy as np


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def score_contour(contour, image_area: int) -> tuple[float, tuple[int, int, int, int], float, float]:
    x, y, w, h = cv2.boundingRect(contour)
    area = cv2.contourArea(contour)
    rect_area = max(w * h, 1)
    fill = area / rect_area
    aspect = h / max(w, 1)
    area_ratio = area / max(image_area, 1)

    aspect_score = max(0.0, 1.0 - abs(aspect - 1.45) / 1.45)
    fill_score = min(fill / 0.72, 1.0)
    size_score = min(area_ratio / 0.03, 1.0)
    score = 0.45 * fill_score + 0.35 * aspect_score + 0.20 * size_score
    return score, (x, y, w, h), area, aspect


def detect_green_candidates(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    green_mask = cv2.inRange(hsv, np.array([35, 45, 35]), np.array([95, 255, 255]))
    highlight_mask = cv2.inRange(hsv, np.array([0, 0, 215]), np.array([179, 70, 255]))

    kernel = np.ones((5, 5), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = image.shape[0] * image.shape[1]
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < image_area * 0.001:
            continue
        score, bbox, area, aspect = score_contour(contour, image_area)
        x, y, w, h = bbox
        roi_green = green_mask[y : y + h, x : x + w]
        roi_highlight = highlight_mask[y : y + h, x : x + w]
        denom = max(w * h - cv2.countNonZero(roi_highlight), 1)
        green_score = cv2.countNonZero(roi_green) / denom
        highlight_ratio = cv2.countNonZero(roi_highlight) / max(w * h, 1)
        candidates.append(
            {
                "score": score,
                "bbox": bbox,
                "area": area,
                "aspect": aspect,
                "green_score": green_score,
                "highlight_ratio": highlight_ratio,
            }
        )
    candidates.sort(key=lambda item: item["score"], reverse=True)
    return candidates, green_mask, highlight_mask


def annotate(image, candidates):
    output = image.copy()
    for index, cand in enumerate(candidates[:5], 1):
        x, y, w, h = cand["bbox"]
        color = (0, 255, 255) if index == 1 else (255, 180, 0)
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 2)
        cv2.circle(output, (x + w // 2, y + h // 2), 4, color, -1)
        cv2.putText(
            output,
            f"#{index} score={cand['score']:.2f} green={cand['green_score']:.2f}",
            (x, max(22, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
    return output


def main() -> None:
    parser = argparse.ArgumentParser(description="Batch-test green cylinder candidates on still images.")
    parser.add_argument("--input", type=Path, default=Path("img"))
    parser.add_argument("--output", type=Path, default=Path("output/green_batch"))
    args = parser.parse_args()

    args.output.mkdir(parents=True, exist_ok=True)
    csv_path = args.output / "summary.csv"
    rows = []

    for image_path in sorted(args.input.glob("*")):
        if image_path.suffix.lower() not in IMAGE_EXTS:
            continue
        image = cv2.imread(str(image_path))
        if image is None:
            continue

        t0 = time.perf_counter()
        candidates, green_mask, highlight_mask = detect_green_candidates(image)
        latency_ms = (time.perf_counter() - t0) * 1000.0

        annotated = annotate(image, candidates)
        stem = image_path.stem
        cv2.imwrite(str(args.output / f"{stem}_annotated.jpg"), annotated)
        cv2.imwrite(str(args.output / f"{stem}_green_mask.png"), green_mask)

        best = candidates[0] if candidates else None
        row = {
            "file": image_path.name,
            "width": image.shape[1],
            "height": image.shape[0],
            "candidate_count": len(candidates),
            "latency_ms": f"{latency_ms:.2f}",
            "best_score": f"{best['score']:.3f}" if best else "",
            "best_green_score": f"{best['green_score']:.3f}" if best else "",
            "best_highlight_ratio": f"{best['highlight_ratio']:.3f}" if best else "",
            "best_bbox": ",".join(map(str, best["bbox"])) if best else "",
        }
        rows.append(row)
        print(row)

    with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()) if rows else ["file"])
        writer.writeheader()
        writer.writerows(rows)

    print(f"summary: {csv_path}")


if __name__ == "__main__":
    main()
