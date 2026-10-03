from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

from green_cylinder_image_batch import annotate
from green_cylinder_video_live import candidate_mask, default_params, detect_green_candidates_live


IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def put_data_panel(image, image_path: Path, candidates, best) -> None:
    panel_h = 150
    overlay = image.copy()
    cv2.rectangle(overlay, (0, 0), (image.shape[1], panel_h), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.58, image, 0.42, 0, dst=image)

    lines = [
        f"file: {image_path.name}",
        f"candidates: {len(candidates)}",
        f"best bbox: {best['bbox'] if best else None}",
        f"best score: {best['score']:.3f}" if best else "best score: none",
        f"green: {best['green_score']:.3f}  highlight: {best['highlight_ratio']:.3f}" if best else "",
        "left: detection / right: best-candidate mask",
    ]

    y = 26
    for line in lines:
        if not line:
            continue
        cv2.putText(
            image,
            line,
            (14, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        y += 23


def make_review_image(image_path: Path, image, debug, best_mask, candidates, best):
    left = debug.copy()
    put_data_panel(left, image_path, candidates, best)

    mask_bgr = cv2.cvtColor(best_mask, cv2.COLOR_GRAY2BGR)
    cv2.putText(
        mask_bgr,
        "MASK: best candidate",
        (14, 34),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.82,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        mask_bgr,
        f"white pixels: {cv2.countNonZero(best_mask)}",
        (14, 68),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    separator = np.full((image.shape[0], 10, 3), 35, dtype=np.uint8)
    return cv2.hconcat([left, separator, mask_bgr])


def main() -> None:
    parser = argparse.ArgumentParser(description="Preprocess still images with the current green-cylinder defaults.")
    parser.add_argument("--input", type=Path, default=Path("img"))
    parser.add_argument("--output", type=Path, default=Path("dataset/green_cylinder_preprocessed_defaults"))
    args = parser.parse_args()

    image_dir = args.output / "images"
    full_mask_dir = args.output / "full_masks"
    best_mask_dir = args.output / "best_candidate_masks"
    debug_dir = args.output / "debug"
    review_dir = args.output / "review"
    meta_dir = args.output / "meta"
    for path in [image_dir, full_mask_dir, best_mask_dir, debug_dir, review_dir, meta_dir]:
        path.mkdir(parents=True, exist_ok=True)

    params = default_params()
    rows = []

    for image_path in sorted(args.input.glob("*")):
        if image_path.suffix.lower() not in IMAGE_EXTS:
            continue

        image = cv2.imread(str(image_path))
        if image is None:
            continue

        candidates, green_mask, _ = detect_green_candidates_live(image, params)
        best = candidates[0] if candidates else None
        best_mask = candidate_mask(green_mask, best)
        debug = annotate(image, candidates)
        review = make_review_image(image_path, image, debug, best_mask, candidates, best)

        stem = image_path.stem
        out_image = image_dir / f"{stem}.jpg"
        out_full_mask = full_mask_dir / f"{stem}_full_mask.png"
        out_best_mask = best_mask_dir / f"{stem}_best_mask.png"
        out_debug = debug_dir / f"{stem}_debug.jpg"
        out_review = review_dir / f"{stem}_review.jpg"
        out_meta = meta_dir / f"{stem}.json"

        cv2.imwrite(str(out_image), image)
        cv2.imwrite(str(out_full_mask), green_mask)
        cv2.imwrite(str(out_best_mask), best_mask)
        cv2.imwrite(str(out_debug), debug)
        cv2.imwrite(str(out_review), review)

        serializable_candidates = []
        for candidate in candidates:
            item = dict(candidate)
            item["bbox"] = list(item["bbox"])
            serializable_candidates.append(item)

        meta = {
            "source": str(image_path),
            "params": params,
            "candidate_count": len(candidates),
            "best_bbox": list(best["bbox"]) if best else None,
            "candidates": serializable_candidates,
            "outputs": {
                "image": str(out_image),
                "full_mask": str(out_full_mask),
                "best_mask": str(out_best_mask),
                "debug": str(out_debug),
                "review": str(out_review),
            },
        }
        out_meta.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

        row = {
            "file": image_path.name,
            "candidate_count": len(candidates),
            "best_bbox": ",".join(map(str, best["bbox"])) if best else "",
            "best_score": f"{best['score']:.3f}" if best else "",
            "best_green_score": f"{best['green_score']:.3f}" if best else "",
            "best_highlight_ratio": f"{best['highlight_ratio']:.3f}" if best else "",
            "image": str(out_image),
            "full_mask": str(out_full_mask),
            "best_mask": str(out_best_mask),
            "debug": str(out_debug),
            "review": str(out_review),
        }
        rows.append(row)
        print(row)

    summary_path = args.output / "summary.csv"
    if rows:
        with summary_path.open("w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)

    params_path = args.output / "params.json"
    params_path.write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"done: {len(rows)} images")
    print(f"output: {args.output}")
    print(f"summary: {summary_path}")
    print(f"params: {params_path}")


if __name__ == "__main__":
    main()
