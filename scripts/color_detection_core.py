from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


FUSION_NAMES = [
    "BGR",
    "ExG",
    "HSV",
    "Lab",
    "OR",
    "AND",
    "Vote 2",
    "Vote 3",
    "Core+Support",
]
SMOOTH_NAMES = ["None", "Gaussian", "Median", "Bilateral"]
KERNEL_SHAPES = [cv2.MORPH_RECT, cv2.MORPH_ELLIPSE, cv2.MORPH_CROSS]


DEFAULT_PARAMS: dict[str, int | float | bool] = {
    "g_min": 40,
    "gr_min": 20,
    "gb_min": 20,
    "exg_min": 45,
    "exg_g_min": 35,
    "exg_require_dominance": True,
    "h_min": 35,
    "h_max": 85,
    "s_min": 40,
    "s_max": 255,
    "v_min": 30,
    "v_max": 255,
    "lab_l_min": 15,
    "lab_l_max": 255,
    "lab_a_min": 0,
    "lab_a_max": 125,
    "lab_b_min": 135,
    "lab_b_max": 255,
    "fusion_mode": 7,
    "support_votes": 2,
    "smooth_mode": 0,
    "smooth_kernel": 3,
    "bilateral_sigma": 50,
    "morph_shape": 1,
    "morph_kernel": 3,
    "open_iterations": 0,
    "close_iterations": 1,
    "dilate_iterations": 0,
    "erode_iterations": 0,
    "fill_holes": False,
    "min_area": 300,
    "max_area_ratio": 0.80,
    "reject_border": False,
    "max_targets": 2,
    "second_area_ratio": 0.15,
    "second_min_fill": 0.15,
    "second_min_confidence": 0.70,
}


@dataclass
class DetectionResult:
    masks: dict[str, np.ndarray]
    best: dict[str, Any] | None
    candidates: list[dict[str, Any]]
    timings_ms: dict[str, float]


def _elapsed_ms(start: float) -> float:
    return (time.perf_counter() - start) * 1000.0


def _odd_kernel(value: int) -> int:
    value = max(1, int(value))
    return value if value % 2 else value + 1


def _as_mask(condition: np.ndarray) -> np.ndarray:
    return condition.astype(np.uint8) * 255


def smooth_frame(frame: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    mode = int(params["smooth_mode"])
    kernel = _odd_kernel(int(params["smooth_kernel"]))
    if mode == 1 and kernel > 1:
        return cv2.GaussianBlur(frame, (kernel, kernel), 0)
    if mode == 2 and kernel > 1:
        return cv2.medianBlur(frame, kernel)
    if mode == 3:
        diameter = max(1, kernel)
        sigma = max(1, int(params["bilateral_sigma"]))
        return cv2.bilateralFilter(frame, diameter, sigma, sigma)
    return frame


def make_color_masks(frame: np.ndarray, params: dict[str, Any]) -> tuple[dict[str, np.ndarray], dict[str, float]]:
    timings: dict[str, float] = {}

    started = time.perf_counter()
    b, g, r = cv2.split(frame)
    g_valid = cv2.inRange(g, int(params["g_min"]), 255)
    gr = cv2.subtract(g, r)
    gb = cv2.subtract(g, b)
    gr_valid = cv2.inRange(gr, int(params["gr_min"]), 255)
    gb_valid = cv2.inRange(gb, int(params["gb_min"]), 255)
    bgr_mask = cv2.bitwise_and(g_valid, cv2.bitwise_and(gr_valid, gb_valid))
    timings["bgr"] = _elapsed_ms(started)

    started = time.perf_counter()
    exg = cv2.addWeighted(g, 2.0, r, -1.0, 0.0, dtype=cv2.CV_16S)
    exg = cv2.subtract(exg, b, dtype=cv2.CV_16S)
    exg_valid = cv2.inRange(exg, int(params["exg_min"]), 32767)
    exg_g_valid = cv2.inRange(g, int(params["exg_g_min"]), 255)
    exg_mask = cv2.bitwise_and(exg_valid, exg_g_valid)
    if bool(params.get("exg_require_dominance", True)):
        gr_guard = cv2.inRange(gr, int(params["gr_min"]), 255)
        gb_guard = cv2.inRange(gb, int(params["gb_min"]), 255)
        exg_mask = cv2.bitwise_and(exg_mask, cv2.bitwise_and(gr_guard, gb_guard))
    timings["exg"] = _elapsed_ms(started)

    started = time.perf_counter()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_min = int(params["h_min"])
    h_max = int(params["h_max"])
    lower = np.array([h_min, int(params["s_min"]), int(params["v_min"])], dtype=np.uint8)
    upper = np.array([h_max, int(params["s_max"]), int(params["v_max"])], dtype=np.uint8)
    if h_min <= h_max:
        hsv_mask = cv2.inRange(hsv, lower, upper)
    else:
        low_wrap = cv2.inRange(
            hsv,
            np.array([0, int(params["s_min"]), int(params["v_min"])], dtype=np.uint8),
            upper,
        )
        high_wrap = cv2.inRange(
            hsv,
            lower,
            np.array([179, int(params["s_max"]), int(params["v_max"])], dtype=np.uint8),
        )
        hsv_mask = cv2.bitwise_or(low_wrap, high_wrap)
    timings["hsv"] = _elapsed_ms(started)

    started = time.perf_counter()
    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
    lab_mask = cv2.inRange(
        lab,
        np.array(
            [int(params["lab_l_min"]), int(params["lab_a_min"]), int(params["lab_b_min"])],
            dtype=np.uint8,
        ),
        np.array(
            [int(params["lab_l_max"]), int(params["lab_a_max"]), int(params["lab_b_max"])],
            dtype=np.uint8,
        ),
    )
    timings["lab"] = _elapsed_ms(started)

    return {"bgr": bgr_mask, "exg": exg_mask, "hsv": hsv_mask, "lab": lab_mask}, timings


def make_single_color_mask(
    frame: np.ndarray,
    params: dict[str, Any],
    method: str,
) -> np.ndarray:
    if method in {"bgr", "exg"}:
        b, g, r = cv2.split(frame)
        gr = cv2.subtract(g, r)
        gb = cv2.subtract(g, b)
        if method == "bgr":
            mask = cv2.inRange(g, int(params["g_min"]), 255)
            mask = cv2.bitwise_and(mask, cv2.inRange(gr, int(params["gr_min"]), 255))
            return cv2.bitwise_and(mask, cv2.inRange(gb, int(params["gb_min"]), 255))

        exg = cv2.addWeighted(g, 2.0, r, -1.0, 0.0, dtype=cv2.CV_16S)
        exg = cv2.subtract(exg, b, dtype=cv2.CV_16S)
        mask = cv2.inRange(exg, int(params["exg_min"]), 32767)
        mask = cv2.bitwise_and(mask, cv2.inRange(g, int(params["exg_g_min"]), 255))
        if bool(params.get("exg_require_dominance", True)):
            mask = cv2.bitwise_and(mask, cv2.inRange(gr, int(params["gr_min"]), 255))
            mask = cv2.bitwise_and(mask, cv2.inRange(gb, int(params["gb_min"]), 255))
        return mask

    if method == "hsv":
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        h_min = int(params["h_min"])
        h_max = int(params["h_max"])
        lower = np.array([h_min, int(params["s_min"]), int(params["v_min"])], dtype=np.uint8)
        upper = np.array([h_max, int(params["s_max"]), int(params["v_max"])], dtype=np.uint8)
        if h_min <= h_max:
            return cv2.inRange(hsv, lower, upper)
        low_wrap = cv2.inRange(
            hsv,
            np.array([0, int(params["s_min"]), int(params["v_min"])], dtype=np.uint8),
            upper,
        )
        high_wrap = cv2.inRange(
            hsv,
            lower,
            np.array([179, int(params["s_max"]), int(params["v_max"])], dtype=np.uint8),
        )
        return cv2.bitwise_or(low_wrap, high_wrap)

    if method == "lab":
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        return cv2.inRange(
            lab,
            np.array(
                [int(params["lab_l_min"]), int(params["lab_a_min"]), int(params["lab_b_min"])],
                dtype=np.uint8,
            ),
            np.array(
                [int(params["lab_l_max"]), int(params["lab_a_max"]), int(params["lab_b_max"])],
                dtype=np.uint8,
            ),
        )
    raise ValueError(f"Unsupported color method: {method}")


def fuse_masks(color_masks: dict[str, np.ndarray], params: dict[str, Any]) -> dict[str, np.ndarray]:
    bgr = color_masks["bgr"]
    exg = color_masks["exg"]
    hsv = color_masks["hsv"]
    lab = color_masks["lab"]

    direct = cv2.bitwise_and(bgr, exg)
    color = cv2.bitwise_or(hsv, lab)
    core = cv2.bitwise_and(direct, color)

    votes = cv2.add(bgr, exg, dtype=cv2.CV_16U)
    votes = cv2.add(votes, hsv, dtype=cv2.CV_16U)
    votes = cv2.add(votes, lab, dtype=cv2.CV_16U)
    support_votes = min(4, max(1, int(params["support_votes"])))
    support = cv2.inRange(votes, support_votes * 255, 4 * 255)

    mode = int(params["fusion_mode"])
    if mode == 0:
        fused = bgr.copy()
    elif mode == 1:
        fused = exg.copy()
    elif mode == 2:
        fused = hsv.copy()
    elif mode == 3:
        fused = lab.copy()
    elif mode == 4:
        fused = cv2.bitwise_or(cv2.bitwise_or(bgr, exg), cv2.bitwise_or(hsv, lab))
    elif mode == 5:
        fused = cv2.bitwise_and(cv2.bitwise_and(bgr, exg), cv2.bitwise_and(hsv, lab))
    elif mode == 6:
        fused = cv2.inRange(votes, 2 * 255, 4 * 255)
    elif mode == 7:
        fused = cv2.inRange(votes, 3 * 255, 4 * 255)
    else:
        count, labels = cv2.connectedComponents(support, connectivity=8)
        if count <= 1 or not np.any(core):
            fused = core.copy()
        else:
            touched = np.unique(labels[core > 0])
            touched = touched[touched != 0]
            keep = np.zeros(count, dtype=np.uint8)
            keep[touched] = 255
            fused = keep[labels]

    return {
        "direct": direct,
        "color": color,
        "core": core,
        "support": support,
        "agreement": (votes // 255).astype(np.uint8),
        "fused": fused,
    }


def fill_mask_holes(mask: np.ndarray) -> np.ndarray:
    padded = cv2.copyMakeBorder(mask, 1, 1, 1, 1, cv2.BORDER_CONSTANT, value=0)
    flood = padded.copy()
    flood_mask = np.zeros((padded.shape[0] + 2, padded.shape[1] + 2), dtype=np.uint8)
    cv2.floodFill(flood, flood_mask, (0, 0), 255)
    holes = cv2.bitwise_not(flood)[1:-1, 1:-1]
    return cv2.bitwise_or(mask, holes)


def clean_mask(mask: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    kernel_size = _odd_kernel(int(params["morph_kernel"]))
    shape_index = min(len(KERNEL_SHAPES) - 1, max(0, int(params["morph_shape"])))
    kernel = cv2.getStructuringElement(KERNEL_SHAPES[shape_index], (kernel_size, kernel_size))
    output = mask.copy()

    if int(params["open_iterations"]) > 0:
        output = cv2.morphologyEx(
            output, cv2.MORPH_OPEN, kernel, iterations=int(params["open_iterations"])
        )
    if int(params["close_iterations"]) > 0:
        output = cv2.morphologyEx(
            output, cv2.MORPH_CLOSE, kernel, iterations=int(params["close_iterations"])
        )
    if int(params["dilate_iterations"]) > 0:
        output = cv2.dilate(output, kernel, iterations=int(params["dilate_iterations"]))
    if int(params["erode_iterations"]) > 0:
        output = cv2.erode(output, kernel, iterations=int(params["erode_iterations"]))
    if bool(params["fill_holes"]):
        output = fill_mask_holes(output)
    return output


def select_components(
    mask: np.ndarray,
    params: dict[str, Any],
    agreement: np.ndarray | None = None,
) -> tuple[list[dict[str, Any]], np.ndarray]:
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return [], np.zeros_like(mask)

    image_area = mask.shape[0] * mask.shape[1]
    min_area = max(1, int(params["min_area"]))
    max_area = int(image_area * float(params["max_area_ratio"]))
    component_labels: list[tuple[int, int]] = []

    for label in range(1, count):
        x, y, w, h, area = stats[label]
        if area < min_area or area > max_area:
            continue
        touches_border = x == 0 or y == 0 or x + w >= mask.shape[1] or y + h >= mask.shape[0]
        if bool(params["reject_border"]) and touches_border:
            continue
        component_labels.append((int(area), label))

    if not component_labels:
        return [], np.zeros_like(mask)

    component_labels.sort(reverse=True)
    first_area = component_labels[0][0]
    selected = np.zeros_like(mask)
    chosen: list[dict[str, Any]] = []
    max_targets = min(2, max(1, int(params.get("max_targets", 2))))

    for area, label in component_labels:
        x, y, w, h, _ = stats[label]
        fill_ratio = area / max(int(w) * int(h), 1)

        if chosen:
            if area < first_area * float(params.get("second_area_ratio", 0.15)):
                continue
            if fill_ratio < float(params.get("second_min_fill", 0.15)):
                continue

        component_roi = _as_mask(labels[y : y + h, x : x + w] == label)
        confidence = 1.0
        if agreement is not None:
            confidence = float(
                cv2.mean(agreement[y : y + h, x : x + w], mask=component_roi)[0] / 4.0
            )
        if chosen and confidence < float(params.get("second_min_confidence", 0.70)):
            continue
        cx, cy = centroids[label]
        contours, _ = cv2.findContours(
            component_roi,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
            offset=(int(x), int(y)),
        )
        contour = max(contours, key=cv2.contourArea) if contours else None
        chosen.append(
            {
                "label": int(label),
                "area": int(area),
                "bbox": (int(x), int(y), int(w), int(h)),
                "centroid": (float(cx), float(cy)),
                "bbox_center": (float(x + w / 2), float(y + h / 2)),
                "fill_ratio": float(fill_ratio),
                "color_confidence": confidence,
                "component_count": len(component_labels),
                "contour": contour,
            }
        )
        selected[y : y + h, x : x + w] = cv2.bitwise_or(
            selected[y : y + h, x : x + w],
            component_roi,
        )
        if len(chosen) >= max_targets:
            break

    return chosen, selected


def select_largest_component(mask: np.ndarray, params: dict[str, Any]) -> tuple[dict[str, Any] | None, np.ndarray]:
    candidates, selected = select_components(mask, params)
    return (candidates[0] if candidates else None), selected


def detect_frame(frame: np.ndarray, params: dict[str, Any]) -> DetectionResult:
    total_started = time.perf_counter()

    started = time.perf_counter()
    filtered = smooth_frame(frame, params)
    smooth_ms = _elapsed_ms(started)

    color_masks, timings = make_color_masks(filtered, params)

    started = time.perf_counter()
    fusion_masks = fuse_masks(color_masks, params)
    timings["fusion"] = _elapsed_ms(started)

    started = time.perf_counter()
    cleaned = clean_mask(fusion_masks["fused"], params)
    timings["morphology"] = _elapsed_ms(started)

    started = time.perf_counter()
    candidates, selected = select_components(cleaned, params, fusion_masks["agreement"])
    best = candidates[0] if candidates else None
    timings["components"] = _elapsed_ms(started)

    timings["smooth"] = smooth_ms
    timings["total"] = _elapsed_ms(total_started)
    masks = {**color_masks, **fusion_masks, "cleaned": cleaned, "selected": selected}
    return DetectionResult(masks=masks, best=best, candidates=candidates, timings_ms=timings)


def detect_single_filter(frame: np.ndarray, params: dict[str, Any], method: str) -> DetectionResult:
    total_started = time.perf_counter()
    started = time.perf_counter()
    filtered = smooth_frame(frame, params)
    smooth_ms = _elapsed_ms(started)

    started = time.perf_counter()
    raw = make_single_color_mask(filtered, params, method)
    color_ms = _elapsed_ms(started)

    started = time.perf_counter()
    cleaned = clean_mask(raw, params)
    morphology_ms = _elapsed_ms(started)

    started = time.perf_counter()
    agreement = (raw > 0).astype(np.uint16) * 4
    candidates, selected = select_components(cleaned, params, agreement)
    components_ms = _elapsed_ms(started)
    best = candidates[0] if candidates else None
    timings = {
        "smooth": smooth_ms,
        method: color_ms,
        "fusion": 0.0,
        "morphology": morphology_ms,
        "components": components_ms,
        "total": _elapsed_ms(total_started),
    }
    masks = {method: raw, "raw": raw, "cleaned": cleaned, "selected": selected}
    return DetectionResult(masks=masks, best=best, candidates=candidates, timings_ms=timings)
