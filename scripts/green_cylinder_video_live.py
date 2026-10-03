from __future__ import annotations

import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import numpy as np

from green_cylinder_image_batch import annotate, score_contour


PARAMETER_HELP = """
파라미터 설명
- H min / H max: HSV 색상 범위입니다. 초록 원통은 보통 35~85 근처입니다. 파란색을 잡으면 H max를 낮추세요.
- S min / S max: 채도 범위입니다. S min을 올리면 회색/흰색/흐린 색 노이즈가 줄지만 어두운 초록을 놓칠 수 있습니다.
- V min / V max: 밝기 범위입니다. V min을 올리면 어두운 노이즈가 줄고, 낮추면 그림자 속 초록도 잡습니다.
- G-R min: BGR 기준으로 G가 R보다 최소 몇 이상 커야 하는지입니다. 빨강/노랑 계열 오탐을 줄입니다.
- G-B min: BGR 기준으로 G가 B보다 최소 몇 이상 커야 하는지입니다. 파란색/청록색 오탐을 줄이는 핵심값입니다.
- HL S max / HL V min: 채도가 낮고 밝기가 높은 유광 반사 영역 기준입니다. green score 계산에서 반사부를 보정합니다.
- min area x0.001%: 너무 작은 후보 제거입니다. 값을 올리면 작은 잡음은 줄지만 멀리 있는 원통을 놓칠 수 있습니다.
- kernel: 마스크 구멍 메우기/잡음 제거 강도입니다. 너무 크면 물체가 붙거나 얇은 부분이 사라집니다.
- max candidates: 화면에 표시할 후보 개수입니다.
- show mask: 1이면 오른쪽에 전체 마스크를, 0이면 1번 후보만 잘라낸 저장용 마스크를 보여줍니다.
- seek x0.1%: 영상 위치입니다. 0은 시작, 1000은 끝입니다.

추천 시작점
- 파란색이 잡히면: H max를 85 -> 80 순서로 낮추고, G-B min을 20 -> 35 -> 50 순서로 올려보세요.
- 원통 일부가 사라지면: S min이나 G-B min을 조금 낮추고 kernel을 3~7 사이에서 조정하세요.
"""


def find_default_video() -> Path:
    videos = sorted(Path("img").glob("*.mp4"))
    if not videos:
        raise FileNotFoundError("No .mp4 file found in img/. Use --source path_to_video.mp4")
    return videos[0]


def put_status(frame, frame_index: int, fps: float, latency_ms: float, candidates: int) -> None:
    lines = [
        f"frame={frame_index}",
        f"fps={fps:.1f}",
        f"latency={latency_ms:.1f} ms",
        f"candidates={candidates}",
        "q quit | space pause | s save | n/p step",
    ]
    y = 28
    for line in lines:
        cv2.putText(
            frame,
            line,
            (14, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.68,
            (40, 240, 40),
            2,
            cv2.LINE_AA,
        )
        y += 28


def nothing(_value: int) -> None:
    pass


def create_control_window() -> str:
    window = "Controls"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 460, 520)

    defaults = {
        "H min": 35,
        "H max": 85,
        "S min": 45,
        "S max": 255,
        "V min": 35,
        "V max": 255,
        "G-R min": 15,
        "G-B min": 25,
        "HL S max": 70,
        "HL V min": 215,
        "min area x0.001%": 50,
        "kernel": 5,
        "max candidates": 3,
        "show mask": 1,
        "seek x0.1%": 920,
    }
    ranges = {
        "H min": 179,
        "H max": 179,
        "S min": 255,
        "S max": 255,
        "V min": 255,
        "V max": 255,
        "G-R min": 255,
        "G-B min": 255,
        "HL S max": 255,
        "HL V min": 255,
        "min area x0.001%": 5000,
        "kernel": 31,
        "max candidates": 20,
        "show mask": 1,
        "seek x0.1%": 1000,
    }
    for name, value in defaults.items():
        cv2.createTrackbar(name, window, value, ranges[name], nothing)
    return window


def set_seek_bar(window: str, frame_index: int, total_frames: int, current_seek: int) -> int:
    if total_frames <= 0:
        return current_seek
    new_seek = min(1000, int(frame_index * 1000 / total_frames))
    if new_seek != current_seek:
        cv2.setTrackbarPos("seek x0.1%", window, new_seek)
    return new_seek


def read_params(window: str) -> dict[str, int | bool]:
    params = {
        "h_min": cv2.getTrackbarPos("H min", window),
        "h_max": cv2.getTrackbarPos("H max", window),
        "s_min": cv2.getTrackbarPos("S min", window),
        "s_max": cv2.getTrackbarPos("S max", window),
        "v_min": cv2.getTrackbarPos("V min", window),
        "v_max": cv2.getTrackbarPos("V max", window),
        "g_r_min": cv2.getTrackbarPos("G-R min", window),
        "g_b_min": cv2.getTrackbarPos("G-B min", window),
        "hl_s_max": cv2.getTrackbarPos("HL S max", window),
        "hl_v_min": cv2.getTrackbarPos("HL V min", window),
        "min_area_ratio": cv2.getTrackbarPos("min area x0.001%", window) / 100000.0,
        "kernel": cv2.getTrackbarPos("kernel", window),
        "max_candidates": max(1, cv2.getTrackbarPos("max candidates", window)),
        "show_mask": cv2.getTrackbarPos("show mask", window) == 1,
        "seek": cv2.getTrackbarPos("seek x0.1%", window),
    }
    return params


def default_params() -> dict[str, int | float | bool]:
    return {
        "h_min": 35,
        "h_max": 85,
        "s_min": 45,
        "s_max": 255,
        "v_min": 35,
        "v_max": 255,
        "g_r_min": 15,
        "g_b_min": 25,
        "hl_s_max": 70,
        "hl_v_min": 215,
        "min_area_ratio": 0.0005,
        "kernel": 5,
        "max_candidates": 3,
        "show_mask": True,
        "seek": 920,
    }


def detect_green_candidates_live(image, params):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_green = np.array([params["h_min"], params["s_min"], params["v_min"]])
    upper_green = np.array([params["h_max"], params["s_max"], params["v_max"]])
    if params["h_min"] <= params["h_max"]:
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
    else:
        low_wrap = cv2.inRange(hsv, np.array([0, params["s_min"], params["v_min"]]), upper_green)
        high_wrap = cv2.inRange(hsv, lower_green, np.array([179, params["s_max"], params["v_max"]]))
        green_mask = cv2.bitwise_or(low_wrap, high_wrap)

    b, g, r = cv2.split(image)
    g_i = g.astype(np.int16)
    r_i = r.astype(np.int16)
    b_i = b.astype(np.int16)
    green_dominance = ((g_i - r_i) >= params["g_r_min"]) & ((g_i - b_i) >= params["g_b_min"])
    green_mask = cv2.bitwise_and(green_mask, (green_dominance.astype(np.uint8) * 255))

    highlight_mask = cv2.inRange(
        hsv,
        np.array([0, 0, params["hl_v_min"]]),
        np.array([179, params["hl_s_max"], 255]),
    )

    kernel_size = int(params["kernel"])
    if kernel_size >= 3:
        if kernel_size % 2 == 0:
            kernel_size += 1
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    image_area = image.shape[0] * image.shape[1]
    candidates = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < image_area * params["min_area_ratio"]:
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
    return candidates[: params["max_candidates"]], green_mask, highlight_mask


def candidate_mask(mask, candidate):
    output = np.zeros_like(mask)
    if not candidate:
        return output
    x, y, w, h = candidate["bbox"]
    output[y : y + h, x : x + w] = mask[y : y + h, x : x + w]
    return output


def build_side_by_side_view(shown, green_mask, candidates, show_full_mask: bool):
    if show_full_mask:
        mask_to_show = green_mask
        label = "MASK: full threshold"
    else:
        best = candidates[0] if candidates else None
        mask_to_show = candidate_mask(green_mask, best)
        label = "MASK: best candidate only"

    mask_bgr = cv2.cvtColor(mask_to_show, cv2.COLOR_GRAY2BGR)
    cv2.putText(
        shown,
        "DETECTION",
        (14, shown.shape[0] - 18),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        mask_bgr,
        label,
        (14, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.75,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.putText(
        mask_bgr,
        f"white pixels={cv2.countNonZero(mask_to_show)}",
        (14, 64),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    separator = np.full((shown.shape[0], 8, 3), 35, dtype=np.uint8)
    return cv2.hconcat([shown, separator, mask_bgr])


def save_training_sample(
    save_dir: Path,
    source: Path,
    frame_index: int,
    frame,
    shown,
    green_mask,
    candidates,
    params,
) -> None:
    image_dir = save_dir / "images"
    mask_dir = save_dir / "masks"
    debug_dir = save_dir / "debug"
    meta_dir = save_dir / "meta"
    for path in [image_dir, mask_dir, debug_dir, meta_dir]:
        path.mkdir(parents=True, exist_ok=True)

    stem = f"{source.stem}_frame_{frame_index:06d}"
    best = candidates[0] if candidates else None
    mask_to_save = candidate_mask(green_mask, best)

    image_path = image_dir / f"{stem}.jpg"
    mask_path = mask_dir / f"{stem}.png"
    debug_path = debug_dir / f"{stem}.jpg"
    meta_path = meta_dir / f"{stem}.json"
    index_path = save_dir / "index.csv"

    cv2.imwrite(str(image_path), frame)
    cv2.imwrite(str(mask_path), mask_to_save)
    cv2.imwrite(str(debug_path), shown)

    serializable_candidates = []
    for candidate in candidates:
        item = dict(candidate)
        item["bbox"] = list(item["bbox"])
        serializable_candidates.append(item)

    meta = {
        "source": str(source),
        "frame_index": frame_index,
        "image": str(image_path),
        "mask": str(mask_path),
        "debug": str(debug_path),
        "params": params,
        "best_bbox": list(best["bbox"]) if best else None,
        "candidate_count": len(candidates),
        "candidates": serializable_candidates,
    }
    meta_path.write_text(json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")

    new_file = not index_path.exists()
    with index_path.open("a", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "source",
                "frame_index",
                "image",
                "mask",
                "debug",
                "best_bbox",
                "candidate_count",
            ],
        )
        if new_file:
            writer.writeheader()
        writer.writerow(
            {
                "source": str(source),
                "frame_index": frame_index,
                "image": str(image_path),
                "mask": str(mask_path),
                "debug": str(debug_path),
                "best_bbox": ",".join(map(str, best["bbox"])) if best else "",
                "candidate_count": len(candidates),
            }
        )

    print(f"saved training sample: {stem}")


def process_and_show_frame(
    frame,
    frame_index: int,
    live_fps: float,
    controls: str | None,
    benchmark: bool,
    window_name: str,
):
    t0 = time.perf_counter()
    params = default_params() if controls is None else read_params(controls)
    candidates, green_mask, _ = detect_green_candidates_live(frame, params)
    latency_ms = (time.perf_counter() - t0) * 1000.0

    if benchmark:
        return {
            "params": params,
            "candidates": candidates,
            "mask": green_mask,
            "shown": None,
            "latency_ms": latency_ms,
        }

    shown = annotate(frame, candidates)
    put_status(shown, frame_index, live_fps, latency_ms, len(candidates))
    combined = build_side_by_side_view(shown.copy(), green_mask, candidates, params["show_mask"])
    cv2.imshow(window_name, combined)

    return {
        "params": params,
        "candidates": candidates,
        "mask": green_mask,
        "shown": shown,
        "latency_ms": latency_ms,
    }


def run_video(
    source: Path,
    display_width: int | None,
    max_frames: int | None,
    real_time: bool,
    benchmark: bool,
    save_dir: Path,
) -> None:
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {source}")

    source_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frame_delay = max(1, int(1000 / source_fps)) if real_time else 1
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    window_name = "Green cylinder detection | left: video, right: mask"
    paused = False
    frame_index = 0
    last_tick = time.perf_counter()
    run_started = time.perf_counter()
    latency_sum_ms = 0.0
    controls = None if benchmark else create_control_window()
    seek_pos = 0
    last_seek_request_time = 0.0
    processed_count = 0
    current_frame = None
    current_shown = None
    current_mask = None
    current_candidates = []
    current_params = default_params()
    last_processed_params = None
    step_request = 0

    print(f"source: {source}")
    print(f"source_fps: {source_fps:.2f}")
    print(f"total_frames: {total_frames}")
    if benchmark:
        print("benchmark mode: window disabled")
    else:
        print(PARAMETER_HELP)
        print("press q in the video window to quit")

    while True:
        seek_requested = False
        if controls is not None and total_frames > 0:
            requested_seek = cv2.getTrackbarPos("seek x0.1%", controls)
            seek_requested = abs(requested_seek - seek_pos) > 2
            if seek_requested:
                target_frame = min(total_frames - 1, max(0, int(total_frames * requested_seek / 1000)))
                cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
                frame_index = target_frame
                seek_pos = requested_seek
                last_seek_request_time = time.perf_counter()

        should_read = not paused or step_request != 0 or seek_requested
        if should_read:
            if step_request < 0:
                target_frame = max(0, frame_index - 2)
                cap.set(cv2.CAP_PROP_POS_FRAMES, target_frame)
                frame_index = target_frame

            ok, frame = cap.read()
            if not ok:
                break
            frame_index += 1
            processed_count += 1
            step_request = 0

            if controls is not None and total_frames > 0 and time.perf_counter() - last_seek_request_time > 0.15:
                seek_pos = set_seek_bar(controls, frame_index, total_frames, seek_pos)

            if display_width and frame.shape[1] > display_width:
                scale = display_width / frame.shape[1]
                frame = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

            now = time.perf_counter()
            live_fps = 1.0 / max(now - last_tick, 1e-6)
            last_tick = now

            result = process_and_show_frame(frame, frame_index, live_fps, controls, benchmark, window_name)
            latency_sum_ms += result["latency_ms"]
            current_frame = frame.copy()
            current_shown = None if result["shown"] is None else result["shown"].copy()
            current_mask = result["mask"].copy()
            current_candidates = list(result["candidates"])
            current_params = dict(result["params"])
            last_processed_params = dict(result["params"])

            if max_frames and processed_count >= max_frames:
                break
        elif paused and controls is not None and current_frame is not None:
            params = read_params(controls)
            if params != last_processed_params:
                result = process_and_show_frame(current_frame, frame_index, 0.0, controls, benchmark, window_name)
                current_shown = None if result["shown"] is None else result["shown"].copy()
                current_mask = result["mask"].copy()
                current_candidates = list(result["candidates"])
                current_params = dict(result["params"])
                last_processed_params = dict(result["params"])

        if benchmark:
            continue

        key = cv2.waitKey(frame_delay if not paused else 30) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused
        if key == ord("s") and current_frame is not None:
            save_training_sample(
                save_dir,
                source,
                frame_index,
                current_frame,
                current_shown,
                current_mask,
                current_candidates,
                current_params,
            )
        if key == ord("n"):
            paused = True
            step_request = 1
        if key == ord("p"):
            paused = True
            step_request = -1

    cap.release()
    cv2.destroyAllWindows()
    elapsed = time.perf_counter() - run_started
    avg_fps = processed_count / max(elapsed, 1e-6)
    avg_latency_ms = latency_sum_ms / max(processed_count, 1)
    print(f"done. processed_frames: {processed_count}")
    print(f"last_frame_index: {frame_index}")
    print(f"elapsed_sec: {elapsed:.2f}")
    print(f"avg_loop_fps: {avg_fps:.2f}")
    print(f"avg_detection_latency_ms: {avg_latency_ms:.2f}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Live OpenCV window for green cylinder video detection.")
    parser.add_argument("--source", type=Path, default=None, help="Video path. Defaults to the first img/*.mp4.")
    parser.add_argument("--display-width", type=int, default=720, help="Resize each side before side-by-side display.")
    parser.add_argument("--max-frames", type=int, default=None, help="Stop automatically after N frames.")
    parser.add_argument("--fast", action="store_true", help="Run as fast as possible instead of source FPS.")
    parser.add_argument("--benchmark", action="store_true", help="Disable windows and measure processing speed.")
    parser.add_argument("--save-dir", type=Path, default=Path("dataset/green_cylinder"))
    args = parser.parse_args()

    source = args.source if args.source else find_default_video()
    run_video(
        source,
        args.display_width,
        args.max_frames,
        real_time=not args.fast,
        benchmark=args.benchmark,
        save_dir=args.save_dir,
    )


if __name__ == "__main__":
    main()
