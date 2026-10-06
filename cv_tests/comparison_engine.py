"""New, stateless color detectors and a common comparison video renderer."""

from pathlib import Path
import argparse
import time

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
VIDEO = ROOT / "img" / "KakaoTalk_20261006_201439113.mp4"
HSV_LOW, HSV_HIGH = (35, 60, 35), (90, 255, 255)
LAB_LOW, LAB_HIGH = (15, 0, 135), (255, 125, 255)
MIN_AREA = 8.0
KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
METHODS = {
    "01_four_vote": (("BGR", "ExG", "HSV", "Lab"), 3),
    "02_no_exg": (("BGR", "HSV", "Lab"), 2),
    "03_no_lab": (("BGR", "ExG", "HSV"), 2),
    "04_no_bgr": (("ExG", "HSV", "Lab"), 2),
    "05_hsv": (("HSV",), 1),
    "06_normalized": (("Normalized",), 1),
    "07_hsv_normalized": (("HSV", "Normalized"), 2),
    "08_hsv_cascade": (("Proposal", "HSV ROI"), 2),
}


def color_masks(frame, names):
    masks = {}
    if any(name in names for name in ("BGR", "ExG", "Normalized")):
        b, g, r = cv2.split(frame)
        if "BGR" in names or "ExG" in names:
            gr, gb = cv2.subtract(g, r), cv2.subtract(g, b)
            dominance = cv2.bitwise_and(cv2.inRange(gr, 20, 255), cv2.inRange(gb, 20, 255))
        if "BGR" in names:
            masks["BGR"] = cv2.bitwise_and(cv2.inRange(g, 40, 255), dominance)
        if "ExG" in names:
            exg = cv2.addWeighted(g, 2, r, -1, 0, dtype=cv2.CV_16S)
            exg = cv2.subtract(exg, b, dtype=cv2.CV_16S)
            masks["ExG"] = cv2.bitwise_and(cv2.inRange(exg, 45, 32767), dominance)
            masks["ExG"] &= cv2.inRange(g, 35, 255)
        if "Normalized" in names:
            # Linear inequalities equivalent to g/sum>=.42 and (g-r,b)/sum>=.08.
            # One optimized transform avoids many NumPy full-frame temporaries.
            matrix = np.array([[-.42, .58, -.42], [-.08, .92, -1.08],
                               [-1.08, .92, -.08]], dtype=np.float32)
            scores = cv2.transform(frame.astype(np.float32), matrix)
            masks["Normalized"] = cv2.inRange(scores, (-.0001,) * 3, (255,) * 3)
            masks["Normalized"] &= cv2.inRange(g, 35, 255)
    if "HSV" in names:
        masks["HSV"] = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), HSV_LOW, HSV_HIGH)
    if "Lab" in names:
        masks["Lab"] = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2LAB), LAB_LOW, LAB_HIGH)
    return masks


def largest_target(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = max(contours, key=cv2.contourArea, default=None)
    if best is None:
        return None
    area = cv2.contourArea(best)
    if area < MIN_AREA:
        return None
    x, y, w, h = cv2.boundingRect(best)
    m = cv2.moments(best)
    return {"box": [int(x), int(y), int(w), int(h)], "area": float(area),
            "center": [float(m["m10"] / m["m00"]), float(m["m01"] / m["m00"])]}


def detect(frame, method):
    """Only compute filters actually used by this method. No previous-box fallback."""
    if method not in METHODS:
        raise ValueError(f"Unknown method: {method}")
    names, required = METHODS[method]
    if method == "08_hsv_cascade":
        height, width = frame.shape[:2]
        small = cv2.resize(frame, (max(1, width // 2), max(1, height // 2)),
                           interpolation=cv2.INTER_AREA)
        proposal = color_masks(small, ("HSV",))["HSV"]
        contours, _ = cv2.findContours(proposal, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_mask = np.zeros((height, width), np.uint8)
        # Inspect every proposal, not only the largest: no candidate rank starvation.
        for contour in contours:
            if cv2.contourArea(contour) < 1:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            sx, sy = width / small.shape[1], height / small.shape[0]
            x0, y0 = max(0, int(x * sx) - 12), max(0, int(y * sy) - 12)
            x1 = min(width, int(np.ceil((x + w) * sx)) + 12)
            y1 = min(height, int(np.ceil((y + h) * sy)) + 12)
            roi_mask[y0:y1, x0:x1] |= color_masks(frame[y0:y1, x0:x1], ("HSV",))["HSV"]
        masks = {"Proposal": cv2.resize(proposal, (width, height), interpolation=cv2.INTER_NEAREST),
                 "HSV ROI": roi_mask}
        fused = roi_mask
    else:
        masks = color_masks(frame, names)
        if len(names) == 1:
            fused = masks[names[0]].copy()
        else:
            votes = np.zeros(frame.shape[:2], np.uint8)
            for mask in masks.values():
                votes += mask // 255
            fused = cv2.inRange(votes, required, len(names))
    fused = cv2.morphologyEx(fused, cv2.MORPH_CLOSE, KERNEL)
    target = largest_target(fused)
    return masks, fused, target


def render(frame, masks, fused, target, method, index, milliseconds, fps):
    """Fixed 3x2 canvas: annotated source, up to 4 raw masks, final mask."""
    tile_w, tile_h = 270, 480
    canvas = np.zeros((tile_h * 2, tile_w * 3, 3), np.uint8)
    tiles = [(frame, "SOURCE " + ("FOUND" if target else "NONE"))]
    tiles += [(mask, name) for name, mask in masks.items()]
    while len(tiles) < 5:
        tiles.append((None, "UNUSED - NOT COMPUTED"))
    tiles.append((fused, "FINAL + BOX"))
    for i, (picture, label) in enumerate(tiles):
        # Fit rather than stretch: supports landscape and portrait source videos.
        tx, ty = (i % 3) * tile_w, (i // 3) * tile_h
        if picture is not None:
            scale = min(tile_w / picture.shape[1], (tile_h - 60) / picture.shape[0])
            interpolation = cv2.INTER_AREA if picture.ndim == 2 else cv2.INTER_LINEAR
            resized = cv2.resize(picture, None, fx=scale, fy=scale, interpolation=interpolation)
            if resized.ndim == 2:
                resized = cv2.cvtColor(resized, cv2.COLOR_GRAY2BGR)
            if target and i in (0, 5):
                x, y, w, h = [round(v * scale) for v in target["box"]]
                cv2.rectangle(resized, (x, y), (x + w, y + h), (0, 255, 0), 1)
                center = tuple(round(v * scale) for v in target["center"])
                cv2.drawMarker(resized, center, (0, 0, 255), cv2.MARKER_CROSS, 10, 1)
            canvas[ty + 60:ty + 60 + resized.shape[0], tx:tx + resized.shape[1]] = resized
        cv2.putText(canvas, label, (tx + 5, ty + 22), cv2.FONT_HERSHEY_SIMPLEX,
                    .43, (255, 255, 255), 1, cv2.LINE_AA)
        if i == 0:
            cv2.putText(canvas, f"#{index} {index / fps:.2f}s {milliseconds:.2f}ms",
                        (tx + 5, ty + 44), cv2.FONT_HERSHEY_SIMPLEX,
                        .42, (0, 255, 255), 1, cv2.LINE_AA)
            cv2.putText(canvas, f"CV-only {1000 / max(milliseconds, .0001):.1f} FPS",
                        (tx + 5, ty + 57), cv2.FONT_HERSHEY_SIMPLEX,
                        .35, (0, 255, 255), 1, cv2.LINE_AA)
        if i == 5 and target:
            cx, cy = target["center"]
            cv2.putText(canvas, f"center ({cx:.1f}, {cy:.1f}) px", (tx + 5, ty + 44),
                        cv2.FONT_HERSHEY_SIMPLEX, .4, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, method, (tile_w + 5, 44), cv2.FONT_HERSHEY_SIMPLEX,
                .45, (0, 255, 255), 1, cv2.LINE_AA)
    return canvas


def single_main(method):
    parser = argparse.ArgumentParser(description=f"Standalone comparison method: {method}")
    parser.add_argument("video", nargs="?", default=str(VIDEO))
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / f"{method}.mp4")
    parser.add_argument("--show", action="store_true")
    args = parser.parse_args()
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open: {args.video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30
    args.output.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (810, 960))
    if not writer.isOpened():
        cap.release()
        raise SystemExit(f"Cannot create: {args.output}")
    index = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            started = time.perf_counter()
            masks, fused, target = detect(frame, method)
            elapsed = (time.perf_counter() - started) * 1000
            preview = render(frame, masks, fused, target, method, index, elapsed, fps)
            writer.write(preview)
            if args.show:
                cv2.imshow(method, preview)
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
            index += 1
    finally:
        cap.release()
        writer.release()
        if args.show:
            cv2.destroyAllWindows()
    print(f"Saved {index} frames: {args.output}")
