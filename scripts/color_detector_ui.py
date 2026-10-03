from __future__ import annotations

import argparse
import csv
import json
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from color_detection_core import (
        DEFAULT_PARAMS,
        FUSION_NAMES,
        SMOOTH_NAMES,
        DetectionResult,
        detect_frame,
    )
except ModuleNotFoundError:
    from scripts.color_detection_core import (
        DEFAULT_PARAMS,
        FUSION_NAMES,
        SMOOTH_NAMES,
        DetectionResult,
        detect_frame,
    )


CONTROL_WINDOW = "Color detector controls"
DISPLAY_WINDOW = "Green target detector | original/final + BGR/ExG/HSV/Lab"
DEFAULT_CONFIG = Path("config/color_detector_defaults.json")
LAST_CONFIG = Path("config/color_detector_last.json")
SNAPSHOT_DIR = Path("output/color_detector/snapshots")
BENCHMARK_DIR = Path("output/color_detector")


TRACKBARS: list[tuple[str, str, int]] = [
    ("BGR G min", "g_min", 255),
    ("BGR G-R min", "gr_min", 255),
    ("BGR G-B min", "gb_min", 255),
    ("ExG min", "exg_min", 510),
    ("ExG G min", "exg_g_min", 255),
    ("HSV H min", "h_min", 179),
    ("HSV H max", "h_max", 179),
    ("HSV S min", "s_min", 255),
    ("HSV S max", "s_max", 255),
    ("HSV V min", "v_min", 255),
    ("HSV V max", "v_max", 255),
    ("Lab L min", "lab_l_min", 255),
    ("Lab L max", "lab_l_max", 255),
    ("Lab a min", "lab_a_min", 255),
    ("Lab a max", "lab_a_max", 255),
    ("Lab b min", "lab_b_min", 255),
    ("Lab b max", "lab_b_max", 255),
    ("Fusion 0-8", "fusion_mode", 8),
    ("Support votes", "support_votes", 4),
    ("Smooth 0-3", "smooth_mode", 3),
    ("Smooth kernel", "smooth_kernel", 15),
    ("Bilateral sigma", "bilateral_sigma", 200),
    ("Morph shape 0-2", "morph_shape", 2),
    ("Morph kernel", "morph_kernel", 15),
    ("Open iter", "open_iterations", 5),
    ("Close iter", "close_iterations", 5),
    ("Dilate iter", "dilate_iterations", 5),
    ("Erode iter", "erode_iterations", 5),
    ("Fill holes", "fill_holes", 1),
    ("Min area", "min_area", 200000),
    ("Max area x0.1pct", "max_area_permille", 1000),
    ("Reject border", "reject_border", 1),
]


def load_config(path: Path) -> dict[str, Any]:
    params = dict(DEFAULT_PARAMS)
    if path.exists():
        loaded = json.loads(path.read_text(encoding="utf-8"))
        params.update({key: value for key, value in loaded.items() if key in params})
    return params


def save_config(path: Path, params: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"saved config: {path}")


def create_controls(params: dict[str, Any]) -> None:
    cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(CONTROL_WINDOW, 560, 900)
    for label, key, maximum in TRACKBARS:
        if key == "max_area_permille":
            value = int(float(params["max_area_ratio"]) * 1000)
        else:
            value = int(params[key])
        cv2.createTrackbar(label, CONTROL_WINDOW, min(maximum, max(0, value)), maximum, lambda _value: None)
    cv2.createTrackbar("Seek x0.1pct", CONTROL_WINDOW, 0, 1000, lambda _value: None)


def read_controls() -> dict[str, Any]:
    params = dict(DEFAULT_PARAMS)
    for label, key, _maximum in TRACKBARS:
        value = cv2.getTrackbarPos(label, CONTROL_WINDOW)
        if key == "max_area_permille":
            params["max_area_ratio"] = max(0.001, value / 1000.0)
        elif key in {"fill_holes", "reject_border"}:
            params[key] = bool(value)
        else:
            params[key] = value
    params["support_votes"] = max(1, int(params["support_votes"]))
    params["smooth_kernel"] = max(1, int(params["smooth_kernel"]))
    params["morph_kernel"] = max(1, int(params["morph_kernel"]))
    return params


def write_controls(params: dict[str, Any]) -> None:
    for label, key, maximum in TRACKBARS:
        if key == "max_area_permille":
            value = int(float(params["max_area_ratio"]) * 1000)
        else:
            value = int(params[key])
        cv2.setTrackbarPos(label, CONTROL_WINDOW, min(maximum, max(0, value)))


def default_source() -> Path:
    converted = Path("dataset/videos/irc_highlight_1080p50.mp4")
    if converted.exists():
        return converted
    videos = sorted(Path("img").glob("*.mp4"))
    if not videos:
        raise FileNotFoundError("No test video found. Run prepare_test_video.py first.")
    return videos[0]


def annotate_frame(frame: np.ndarray, result: DetectionResult, frame_index: int, source_fps: float) -> np.ndarray:
    output = frame.copy()
    best = result.best
    colors = [(0, 255, 255), (255, 0, 255)]
    for index, candidate in enumerate(result.candidates[:2]):
        x, y, w, h = candidate["bbox"]
        cx, cy = candidate["centroid"]
        color = colors[index]
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 3)
        cv2.circle(output, (int(round(cx)), int(round(cy))), 8, color, -1)
        cv2.putText(output, f"#{index + 1}", (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        if candidate["contour"] is not None:
            cv2.drawContours(output, [candidate["contour"]], -1, color, 2)

    total_ms = result.timings_ms["total"]
    status_color = (40, 220, 40) if total_ms <= 20.0 else (0, 210, 255) if total_ms <= 33.33 else (0, 80, 255)
    lines = [
        f"frame={frame_index} source={source_fps:.2f}fps total={total_ms:.2f}ms",
        (
            f"BGR={result.timings_ms['bgr']:.2f} ExG={result.timings_ms['exg']:.2f} "
            f"HSV={result.timings_ms['hsv']:.2f} Lab={result.timings_ms['lab']:.2f} ms"
        ),
        (
            f"fusion={result.timings_ms['fusion']:.2f} morph={result.timings_ms['morphology']:.2f} "
            f"components={result.timings_ms['components']:.2f} ms"
        ),
    ]
    if best:
        lines.append(
            f"target center=({best['centroid'][0]:.1f},{best['centroid'][1]:.1f}) "
            f"area={best['area']} fill={best['fill_ratio']:.2f} candidates={best['component_count']}"
        )
    else:
        lines.append("target: not detected")

    overlay = output.copy()
    cv2.rectangle(overlay, (0, 0), (output.shape[1], 130), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.58, output, 0.42, 0, dst=output)
    for index, line in enumerate(lines):
        cv2.putText(
            output,
            line,
            (16, 30 + index * 29),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.72,
            status_color if index == 0 else (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return output


def mask_panel(mask: np.ndarray, label: str, width: int, height: int) -> np.ndarray:
    panel = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    panel = cv2.resize(panel, (width, height), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(panel, (0, 0), (width, 32), (0, 0, 0), -1)
    cv2.putText(panel, label, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)
    return panel


def compose_dashboard(
    frame: np.ndarray,
    result: DetectionResult,
    params: dict[str, Any],
    frame_index: int,
    source_fps: float,
    panel_width: int,
) -> np.ndarray:
    panel_height = max(180, int(panel_width * 9 / 16))
    annotated = annotate_frame(frame, result, frame_index, source_fps)
    annotated = cv2.resize(annotated, (panel_width, panel_height), interpolation=cv2.INTER_AREA)
    final_panel = mask_panel(result.masks["cleaned"], "FINAL MASK", panel_width, panel_height)
    if result.best:
        x, y, w, h = result.best["bbox"]
        sx = panel_width / frame.shape[1]
        sy = panel_height / frame.shape[0]
        cv2.rectangle(
            final_panel,
            (int(x * sx), int(y * sy)),
            (int((x + w) * sx), int((y + h) * sy)),
            (0, 255, 255),
            2,
        )
    top = cv2.hconcat([annotated, final_panel])

    small_width = panel_width // 2
    small_height = max(90, panel_height // 2)
    bottom = cv2.hconcat(
        [
            mask_panel(result.masks["bgr"], "BGR", small_width, small_height),
            mask_panel(result.masks["exg"], "ExG", small_width, small_height),
            mask_panel(result.masks["hsv"], "HSV", small_width, small_height),
            mask_panel(result.masks["lab"], "Lab", small_width, small_height),
        ]
    )
    dashboard = cv2.vconcat([top, bottom])
    fusion = FUSION_NAMES[min(8, max(0, int(params["fusion_mode"])))]
    smoothing = SMOOTH_NAMES[min(3, max(0, int(params["smooth_mode"])))]
    kernel_shape = ["Rect", "Ellipse", "Cross"][min(2, max(0, int(params["morph_shape"])))]
    footer = np.zeros((38, dashboard.shape[1], 3), dtype=np.uint8)
    footer_text = (
        f"fusion={fusion}  smoothing={smoothing}  morph={kernel_shape}/{params['morph_kernel']}  "
        "SPACE pause | N/P step | S snapshot | W save | L load | R reset | B benchmark | Q quit"
    )
    cv2.putText(footer, footer_text, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
    return cv2.vconcat([dashboard, footer])


def snapshot(frame: np.ndarray, result: DetectionResult, params: dict[str, Any], frame_index: int) -> None:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    folder = SNAPSHOT_DIR / f"{stamp}_frame_{frame_index:06d}"
    folder.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(folder / "original.jpg"), frame)
    cv2.imwrite(str(folder / "annotated.jpg"), annotate_frame(frame, result, frame_index, 0.0))
    for name, mask in result.masks.items():
        cv2.imwrite(str(folder / f"{name}_mask.png"), mask)
    (folder / "params.json").write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"saved snapshot: {folder}")


def benchmark_row(frame_index: int, result: DetectionResult) -> dict[str, Any]:
    best = result.best
    return {
        "frame": frame_index,
        "detected": int(best is not None),
        "center_x": f"{best['centroid'][0]:.3f}" if best else "",
        "center_y": f"{best['centroid'][1]:.3f}" if best else "",
        "area": best["area"] if best else 0,
        **{f"{name}_ms": f"{value:.4f}" for name, value in result.timings_ms.items()},
    }


def save_benchmark(rows: list[dict[str, Any]]) -> Path | None:
    if not rows:
        return None
    BENCHMARK_DIR.mkdir(parents=True, exist_ok=True)
    path = BENCHMARK_DIR / f"benchmark_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    with path.open("w", newline="", encoding="utf-8-sig") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print(f"saved benchmark: {path}")
    return path


def print_benchmark_summary(rows: list[dict[str, Any]]) -> None:
    if not rows:
        return
    values = np.array([float(row["total_ms"]) for row in rows], dtype=np.float64)
    detected = sum(int(row["detected"]) for row in rows)
    print(f"processed_frames: {len(rows)}")
    print(f"detected_frames: {detected}")
    print(f"avg_total_ms: {values.mean():.3f}")
    print(f"p95_total_ms: {np.percentile(values, 95):.3f}")
    print(f"p99_total_ms: {np.percentile(values, 99):.3f}")
    print(f"throughput_fps: {1000.0 / max(values.mean(), 1e-9):.2f}")
    print(f"over_20ms_50fps: {int(np.count_nonzero(values > 20.0))}")
    print(f"over_33.33ms_30fps: {int(np.count_nonzero(values > 33.3333))}")
    print(f"over_41.67ms_24fps: {int(np.count_nonzero(values > 41.6667))}")


def run_headless(source: Path, params: dict[str, Any], max_frames: int | None, start_frame: int) -> None:
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {source}")
    if start_frame > 0:
        cap.set(cv2.CAP_PROP_POS_FRAMES, start_frame)
    rows: list[dict[str, Any]] = []
    frame_index = start_frame
    processed_count = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_index += 1
        processed_count += 1
        result = detect_frame(frame, params)
        rows.append(benchmark_row(frame_index, result))
        if max_frames and processed_count >= max_frames:
            break
    cap.release()
    save_benchmark(rows)
    print_benchmark_summary(rows)


def run_ui(source: Path, params: dict[str, Any], panel_width: int) -> None:
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {source}")

    source_fps = cap.get(cv2.CAP_PROP_FPS) or 50.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    delay_ms = max(1, int(round(1000.0 / source_fps)))
    create_controls(params)
    cv2.namedWindow(DISPLAY_WINDOW, cv2.WINDOW_NORMAL)

    paused = False
    step = 0
    frame_index = 0
    current_frame = None
    current_result = None
    last_signature: tuple[Any, ...] | None = None
    last_seek_slider = 0
    collecting = False
    benchmark_rows: list[dict[str, Any]] = []

    print(f"source: {source}")
    print(f"video: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))} @ {source_fps:.3f} fps")
    print("Fusion: 0 BGR | 1 ExG | 2 HSV | 3 Lab | 4 OR | 5 AND | 6 Vote2 | 7 Vote3 | 8 Core+Support")
    print("Smooth: 0 None | 1 Gaussian | 2 Median | 3 Bilateral")
    print("Morph shape: 0 Rect | 1 Ellipse | 2 Cross")

    while True:
        loop_started = time.perf_counter()
        requested_seek = cv2.getTrackbarPos("Seek x0.1pct", CONTROL_WINDOW)
        if abs(requested_seek - last_seek_slider) > 2 and total_frames > 0:
            target = min(total_frames - 1, max(0, int(total_frames * requested_seek / 1000)))
            cap.set(cv2.CAP_PROP_POS_FRAMES, target)
            frame_index = target
            step = 1
            paused = True
            last_seek_slider = requested_seek

        should_read = not paused or step != 0 or current_frame is None
        if should_read:
            if step < 0:
                target = max(0, frame_index - 2)
                cap.set(cv2.CAP_PROP_POS_FRAMES, target)
                frame_index = target
            ok, frame = cap.read()
            if not ok:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                frame_index = 0
                paused = True
                step = 0
                continue
            current_frame = frame
            frame_index += 1
            step = 0
            if total_frames > 0:
                last_seek_slider = min(1000, int(frame_index * 1000 / total_frames))
                cv2.setTrackbarPos("Seek x0.1pct", CONTROL_WINDOW, last_seek_slider)

        params = read_controls()
        signature = tuple(sorted(params.items()))
        if should_read or signature != last_signature or current_result is None:
            current_result = detect_frame(current_frame, params)
            last_signature = signature
            if collecting:
                benchmark_rows.append(benchmark_row(frame_index, current_result))

        dashboard = compose_dashboard(current_frame, current_result, params, frame_index, source_fps, panel_width)
        cv2.imshow(DISPLAY_WINDOW, dashboard)
        loop_elapsed_ms = (time.perf_counter() - loop_started) * 1000.0
        wait_ms = 30 if paused else max(1, int(round(delay_ms - loop_elapsed_ms)))
        key = cv2.waitKey(wait_ms) & 0xFF

        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused
        elif key == ord("n"):
            paused = True
            step = 1
        elif key == ord("p"):
            paused = True
            step = -1
        elif key == ord("s"):
            snapshot(current_frame, current_result, params, frame_index)
        elif key == ord("w"):
            save_config(LAST_CONFIG, params)
        elif key == ord("l"):
            loaded = load_config(LAST_CONFIG if LAST_CONFIG.exists() else DEFAULT_CONFIG)
            write_controls(loaded)
        elif key == ord("r"):
            write_controls(load_config(DEFAULT_CONFIG))
        elif key == ord("b"):
            if collecting:
                collecting = False
                save_benchmark(benchmark_rows)
                print_benchmark_summary(benchmark_rows)
                benchmark_rows = []
            else:
                collecting = True
                benchmark_rows = []
                print("benchmark collection started")

    if benchmark_rows:
        save_benchmark(benchmark_rows)
        print_benchmark_summary(benchmark_rows)
    cap.release()
    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="Interactive four-method green target detector.")
    parser.add_argument("--source", type=Path, default=None)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--panel-width", type=int, default=640)
    parser.add_argument("--benchmark", action="store_true", help="Run without UI and save timing CSV.")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--start-frame", type=int, default=0, help="First frame for headless benchmark mode.")
    parser.add_argument("--advanced-ui", action="store_true", help="Open the original all-controls UI.")
    args = parser.parse_args()

    source = args.source or default_source()
    params = load_config(args.config)
    if args.benchmark:
        run_headless(source, params, args.max_frames, max(0, args.start_frame))
    elif not args.advanced_ui:
        try:
            from color_detector_wizard import run_wizard
        except ModuleNotFoundError:
            from scripts.color_detector_wizard import run_wizard
        run_wizard(source, args.config)
    else:
        run_ui(source, params, args.panel_width)


if __name__ == "__main__":
    main()
