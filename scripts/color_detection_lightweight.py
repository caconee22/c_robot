from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

try:
    from color_detection_core import (
        DetectionResult,
        clean_mask,
        fuse_masks,
        make_color_masks,
        select_components,
        smooth_frame,
    )
except ModuleNotFoundError:
    from scripts.color_detection_core import (
        DetectionResult,
        clean_mask,
        fuse_masks,
        make_color_masks,
        select_components,
        smooth_frame,
    )


@dataclass(frozen=True)
class LightweightOptions:
    search_width: int = 960
    search_height: int = 540
    max_rois: int = 8
    roi_expand: float = 2.0
    roi_padding: int = 20
    proposal_min_area_ratio: float = 0.15


def _elapsed_ms(started: float) -> float:
    return (time.perf_counter() - started) * 1000.0


def _make_direct_proposal(frame: np.ndarray, params: dict[str, Any]) -> np.ndarray:
    """Create the cheap BGR/ExG union while splitting channels only once."""
    b, g, r = cv2.split(frame)
    gr = cv2.subtract(g, r)
    gb = cv2.subtract(g, b)

    g_valid = cv2.inRange(g, int(params["g_min"]), 255)
    gr_valid = cv2.inRange(gr, int(params["gr_min"]), 255)
    gb_valid = cv2.inRange(gb, int(params["gb_min"]), 255)
    dominance = cv2.bitwise_and(gr_valid, gb_valid)
    bgr_mask = cv2.bitwise_and(g_valid, dominance)

    exg = cv2.addWeighted(g, 2.0, r, -1.0, 0.0, dtype=cv2.CV_16S)
    exg = cv2.subtract(exg, b, dtype=cv2.CV_16S)
    exg_mask = cv2.inRange(exg, int(params["exg_min"]), 32767)
    exg_mask = cv2.bitwise_and(
        exg_mask,
        cv2.inRange(g, int(params["exg_g_min"]), 255),
    )
    if bool(params.get("exg_require_dominance", True)):
        exg_mask = cv2.bitwise_and(exg_mask, dominance)
    return cv2.bitwise_or(bgr_mask, exg_mask)


def _merge_overlapping_rois(rois: list[tuple[int, int, int, int]]) -> list[tuple[int, int, int, int]]:
    merged: list[tuple[int, int, int, int]] = []
    for x, y, w, h in rois:
        x2, y2 = x + w, y + h
        changed = True
        while changed:
            changed = False
            remaining: list[tuple[int, int, int, int]] = []
            for ox, oy, ow, oh in merged:
                ox2, oy2 = ox + ow, oy + oh
                if x < ox2 and ox < x2 and y < oy2 and oy < y2:
                    x, y = min(x, ox), min(y, oy)
                    x2, y2 = max(x2, ox2), max(y2, oy2)
                    changed = True
                else:
                    remaining.append((ox, oy, ow, oh))
            merged = remaining
        merged.append((x, y, x2 - x, y2 - y))
    return merged


def _proposal_rois(
    proposal: np.ndarray,
    full_shape: tuple[int, int],
    params: dict[str, Any],
    options: LightweightOptions,
) -> list[tuple[int, int, int, int]]:
    full_h, full_w = full_shape
    low_h, low_w = proposal.shape
    count, _labels, stats, _centroids = cv2.connectedComponentsWithStats(proposal, connectivity=8)
    if count <= 1:
        return []

    area_scale = (low_w / full_w) * (low_h / full_h)
    min_area = max(
        3,
        int(round(int(params["min_area"]) * area_scale * options.proposal_min_area_ratio)),
    )
    components: list[tuple[int, int, int, int, int]] = []
    for label in range(1, count):
        x, y, w, h, area = (int(value) for value in stats[label])
        if area >= min_area:
            components.append((area, x, y, w, h))
    components.sort(reverse=True)

    scale_x = full_w / low_w
    scale_y = full_h / low_h
    rois: list[tuple[int, int, int, int]] = []
    for _area, x, y, w, h in components[: max(1, options.max_rois)]:
        cx = (x + w / 2.0) * scale_x
        cy = (y + h / 2.0) * scale_y
        expanded_w = max(1, int(round(w * scale_x * options.roi_expand))) + 2 * options.roi_padding
        expanded_h = max(1, int(round(h * scale_y * options.roi_expand))) + 2 * options.roi_padding
        left = max(0, int(round(cx - expanded_w / 2)))
        top = max(0, int(round(cy - expanded_h / 2)))
        right = min(full_w, left + expanded_w)
        bottom = min(full_h, top + expanded_h)
        left = max(0, right - expanded_w)
        top = max(0, bottom - expanded_h)
        rois.append((left, top, right - left, bottom - top))
    return _merge_overlapping_rois(rois)


def _offset_candidate(candidate: dict[str, Any], x_offset: int, y_offset: int) -> dict[str, Any]:
    mapped = dict(candidate)
    x, y, w, h = candidate["bbox"]
    cx, cy = candidate["centroid"]
    bx, by = candidate["bbox_center"]
    mapped["bbox"] = (x + x_offset, y + y_offset, w, h)
    mapped["centroid"] = (cx + x_offset, cy + y_offset)
    mapped["bbox_center"] = (bx + x_offset, by + y_offset)
    contour = candidate.get("contour")
    if contour is not None:
        mapped["contour"] = contour + np.array([[[x_offset, y_offset]]], dtype=contour.dtype)
    return mapped


def _choose_global_candidates(candidates: list[dict[str, Any]], params: dict[str, Any]) -> list[dict[str, Any]]:
    candidates.sort(key=lambda item: int(item["area"]), reverse=True)
    if not candidates:
        return []
    chosen = [candidates[0]]
    first_area = float(candidates[0]["area"])
    if int(params.get("max_targets", 2)) <= 1:
        return chosen
    for candidate in candidates[1:]:
        if float(candidate["area"]) < first_area * float(params.get("second_area_ratio", 0.15)):
            continue
        if float(candidate["fill_ratio"]) < float(params.get("second_min_fill", 0.15)):
            continue
        if float(candidate["color_confidence"]) < float(params.get("second_min_confidence", 0.70)):
            continue
        chosen.append(candidate)
        break
    return chosen


def detect_frame_lightweight(
    frame: np.ndarray,
    params: dict[str, Any],
    options: LightweightOptions | None = None,
    debug_masks: bool = False,
    search_frame: np.ndarray | None = None,
    build_output_masks: bool = True,
) -> DetectionResult:
    """Low-resolution proposal followed by full-resolution four-filter ROI verification."""
    options = options or LightweightOptions()
    total_started = time.perf_counter()
    full_h, full_w = frame.shape[:2]
    search_w = min(full_w, max(32, int(options.search_width)))
    search_h = min(full_h, max(32, int(options.search_height)))

    started = time.perf_counter()
    if search_frame is None:
        search_image = cv2.resize(frame, (search_w, search_h), interpolation=cv2.INTER_AREA)
    elif search_frame.shape[1] != search_w or search_frame.shape[0] != search_h:
        search_image = cv2.resize(search_frame, (search_w, search_h), interpolation=cv2.INTER_AREA)
    else:
        search_image = search_frame
    proposal = _make_direct_proposal(search_image, params)
    proposal = cv2.morphologyEx(
        proposal,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
        iterations=1,
    )
    proposal_ms = _elapsed_ms(started)

    started = time.perf_counter()
    rois = _proposal_rois(proposal, (full_h, full_w), params, options)
    roi_select_ms = _elapsed_ms(started)

    cleaned_full = np.zeros((full_h, full_w), dtype=np.uint8) if build_output_masks else None
    selected_full = np.zeros((full_h, full_w), dtype=np.uint8) if build_output_masks else None
    debug_full: dict[str, np.ndarray] = {}
    if debug_masks:
        for name in ("bgr", "exg", "hsv", "lab", "core", "support", "fused", "cleaned", "selected"):
            debug_full[name] = np.zeros((search_h, search_w), dtype=np.uint8)

    roi_params = dict(params)
    roi_params["max_area_ratio"] = 1.0
    roi_params["max_targets"] = 2
    all_candidates: list[dict[str, Any]] = []
    verify_started = time.perf_counter()
    for x, y, w, h in rois:
        roi = frame[y : y + h, x : x + w]
        filtered = smooth_frame(roi, roi_params)
        color_masks, _timings = make_color_masks(filtered, roi_params)
        fusion_masks = fuse_masks(color_masks, roi_params)
        cleaned = clean_mask(fusion_masks["fused"], roi_params)
        local_candidates, _local_selected = select_components(
            cleaned,
            roi_params,
            fusion_masks["agreement"],
        )
        if cleaned_full is not None:
            cleaned_full[y : y + h, x : x + w] = cv2.bitwise_or(
                cleaned_full[y : y + h, x : x + w],
                cleaned,
            )
        all_candidates.extend(_offset_candidate(item, x, y) for item in local_candidates)
        if debug_masks:
            combined = {**color_masks, **fusion_masks, "cleaned": cleaned}
            left = max(0, min(search_w - 1, int(round(x * search_w / full_w))))
            top = max(0, min(search_h - 1, int(round(y * search_h / full_h))))
            right = max(left + 1, min(search_w, int(round((x + w) * search_w / full_w))))
            bottom = max(top + 1, min(search_h, int(round((y + h) * search_h / full_h))))
            for name, mask in combined.items():
                if name not in debug_full:
                    continue
                small_mask = cv2.resize(
                    mask,
                    (right - left, bottom - top),
                    interpolation=cv2.INTER_NEAREST,
                )
                debug_full[name][top:bottom, left:right] = np.maximum(
                    debug_full[name][top:bottom, left:right],
                    small_mask,
                )
    verify_ms = _elapsed_ms(verify_started)

    started = time.perf_counter()
    candidates = _choose_global_candidates(all_candidates, params)
    if cleaned_full is not None and selected_full is not None:
        for candidate in candidates:
            x, y, w, h = candidate["bbox"]
            selected_full[y : y + h, x : x + w] = cv2.bitwise_or(
                selected_full[y : y + h, x : x + w],
                cleaned_full[y : y + h, x : x + w],
            )
    if debug_masks:
        for candidate in candidates:
            x, y, w, h = candidate["bbox"]
            left = max(0, min(search_w - 1, int(round(x * search_w / full_w))))
            top = max(0, min(search_h - 1, int(round(y * search_h / full_h))))
            right = max(left + 1, min(search_w, int(round((x + w) * search_w / full_w))))
            bottom = max(top + 1, min(search_h, int(round((y + h) * search_h / full_h))))
            debug_full["selected"][top:bottom, left:right] = debug_full["cleaned"][top:bottom, left:right]
    finalize_ms = _elapsed_ms(started)

    masks: dict[str, np.ndarray] = {**debug_full, "proposal": proposal}
    if cleaned_full is not None and selected_full is not None:
        masks.update({"cleaned": cleaned_full, "selected": selected_full})
    timings = {
        "proposal": proposal_ms,
        "roi_select": roi_select_ms,
        "roi_verify": verify_ms,
        "finalize": finalize_ms,
        "roi_count": float(len(rois)),
        "total": _elapsed_ms(total_started),
    }
    return DetectionResult(
        masks=masks,
        best=candidates[0] if candidates else None,
        candidates=candidates,
        timings_ms=timings,
    )
