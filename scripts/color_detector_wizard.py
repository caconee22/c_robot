from __future__ import annotations

import json
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from threading import Event, Thread
from typing import Any

import cv2
import numpy as np

try:
    from color_detection_core import DEFAULT_PARAMS, DetectionResult, detect_frame, detect_single_filter, make_single_color_mask
except ModuleNotFoundError:
    from scripts.color_detection_core import (
        DEFAULT_PARAMS,
        DetectionResult,
        detect_frame,
        detect_single_filter,
        make_single_color_mask,
    )

try:
    from color_detector_export import export_video
except ModuleNotFoundError:
    from scripts.color_detector_export import export_video


MAIN_WINDOW = "Green detector tuning wizard"
PLAYBACK_WINDOW = "1 Playback"
CONTROL_WINDOW = "2 Current stage controls"
DEFAULT_CONFIG = Path("config/color_detector_defaults.json")
LAST_CONFIG = Path("config/color_detector_last.json")
SNAPSHOT_DIR = Path("output/color_detector/wizard_snapshots")

VIEW_WIDTH = 700
VIEW_HEIGHT = 394
MASK_WIDTH = 700
GUIDE_HEIGHT = 205

BUTTONS = {
    "export": (945, VIEW_HEIGHT + 43, 1150, VIEW_HEIGHT + 88),
    "stop": (1170, VIEW_HEIGHT + 43, 1375, VIEW_HEIGHT + 88),
    "snapshot": (945, VIEW_HEIGHT + 105, 1150, VIEW_HEIGHT + 150),
    "save": (1170, VIEW_HEIGHT + 105, 1375, VIEW_HEIGHT + 150),
}


@dataclass(frozen=True)
class Slider:
    label: str
    key: str
    maximum: int
    scale: float = 1.0
    boolean: bool = False


class ExportJob:
    def __init__(self) -> None:
        self.running = False
        self.current = 0
        self.total = 0
        self.elapsed = 0.0
        self.output: Path | None = None
        self.error = ""
        self.cancel_event = Event()
        self.thread: Thread | None = None

    def start(self, source: Path, params: dict[str, Any]) -> bool:
        if self.running:
            return False
        self.running = True
        self.current = 0
        self.total = 0
        self.elapsed = 0.0
        self.error = ""
        self.cancel_event.clear()
        self.output = Path("output/color_detector/exports") / (
            f"green_detector_all_masks_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
        )

        def progress(current: int, total: int, elapsed: float) -> None:
            self.current = current
            self.total = total
            self.elapsed = elapsed

        def worker() -> None:
            try:
                metadata = export_video(
                    source,
                    self.output,
                    dict(params),
                    progress_callback=progress,
                    cancel_event=self.cancel_event,
                )
                self.output = Path(metadata["output"])
            except Exception as exc:  # UI must remain open and report the export failure.
                self.error = str(exc)
            finally:
                self.running = False

        self.thread = Thread(target=worker, name="color-detector-export", daemon=True)
        self.thread.start()
        return True

    def cancel(self) -> None:
        if self.running:
            self.cancel_event.set()

    def status(self) -> str:
        if self.running:
            percent = self.current * 100.0 / max(self.total, 1)
            speed = self.current / max(self.elapsed, 1e-6)
            return f"EXPORT {percent:.1f}% ({self.current}/{self.total}) {speed:.1f}fps"
        if self.error:
            return f"EXPORT ERROR: {self.error}"
        if self.output:
            return f"EXPORT READY: {self.output}"
        return "EXPORT: idle"


STAGES = [
    {
        "name": "1/8 BGR dominance",
        "method": "bgr",
        "sliders": [
            Slider("G minimum", "g_min", 255),
            Slider("G-R minimum", "gr_min", 255),
            Slider("G-B minimum", "gb_min", 255),
        ],
        "guide": [
            "Keep the green cylinder white. Remove orange/cyan background.",
            "Raise G-R to reject orange. Raise G-B to reject cyan/blue.",
            "Raise G minimum only when dark noise remains.",
        ],
    },
    {
        "name": "2/8 ExG",
        "method": "exg",
        "sliders": [
            Slider("ExG minimum", "exg_min", 510),
            Slider("G minimum", "exg_g_min", 255),
            Slider("Use BGR guard", "exg_require_dominance", 1, boolean=True),
        ],
        "guide": [
            "ExG emphasizes green: 2G-R-B.",
            "Keep BGR guard ON to reject orange and cyan false positives.",
            "Raise ExG minimum until colored background disappears.",
        ],
    },
    {
        "name": "3/8 HSV",
        "method": "hsv",
        "sliders": [
            Slider("H minimum", "h_min", 179),
            Slider("H maximum", "h_max", 179),
            Slider("S minimum", "s_min", 255),
            Slider("V minimum", "v_min", 255),
        ],
        "guide": [
            "Adjust H first. Green is usually around 35-85 in OpenCV.",
            "Lower S minimum if reflections disappear from the cylinder.",
            "Raise V minimum only to remove very dark noise.",
        ],
    },
    {
        "name": "4/8 Lab",
        "method": "lab",
        "sliders": [
            Slider("L minimum", "lab_l_min", 255),
            Slider("a minimum", "lab_a_min", 255),
            Slider("a maximum", "lab_a_max", 255),
            Slider("b minimum", "lab_b_min", 255),
            Slider("b maximum", "lab_b_max", 255),
        ],
        "guide": [
            "Lower a values are greener. Raise b minimum to reject cyan.",
            "Lower a maximum to reject orange/red areas.",
            "Use L minimum last; it only removes dark pixels.",
        ],
    },
    {
        "name": "5/8 Filter fusion",
        "method": None,
        "sliders": [
            Slider("Fusion 0-8", "fusion_mode", 8),
            Slider("Support votes", "support_votes", 4),
        ],
        "guide": [
            "Recommended: Fusion 7 = at least 3 of 4 filters agree.",
            "6=Vote2 is wider. 5=AND is strict. 8=Core+Support is slower.",
            "Check that orange/cyan regions are black in the final mask.",
        ],
    },
    {
        "name": "6/8 Smoothing and morphology",
        "method": None,
        "sliders": [
            Slider("Smooth 0-3", "smooth_mode", 3),
            Slider("Smooth kernel", "smooth_kernel", 15),
            Slider("Morph kernel", "morph_kernel", 15),
            Slider("Open iterations", "open_iterations", 3),
            Slider("Close iterations", "close_iterations", 3),
            Slider("Fill holes", "fill_holes", 1, boolean=True),
        ],
        "guide": [
            "Start with smoothing 0=None. Add Gaussian only if necessary.",
            "Closing joins small gaps. Opening removes small dots.",
            "Use the smallest kernel that fixes the mask; normally 3.",
        ],
    },
    {
        "name": "7/8 Target candidates",
        "method": None,
        "sliders": [
            Slider("Minimum area", "min_area", 200000),
            Slider("Maximum targets", "max_targets", 2),
            Slider("Second area pct", "second_area_ratio", 100, scale=100.0),
            Slider("Second fill pct", "second_min_fill", 100, scale=100.0),
            Slider("Second confidence pct", "second_min_confidence", 100, scale=100.0),
            Slider("Reject border", "reject_border", 1, boolean=True),
        ],
        "guide": [
            "Target #1 is the largest valid green component.",
            "Target #2 appears only when area, fill and confidence pass.",
            "Raise second thresholds if small noise is marked as #2.",
        ],
    },
    {
        "name": "8/8 Final check",
        "method": None,
        "sliders": [],
        "guide": [
            "Play the video and confirm #1/#2 remain on real cylinders.",
            "Press W to save settings. Press S to save a review snapshot.",
            "Later, test the saved settings with the Raspberry Pi camera.",
        ],
    },
]


def load_params(path: Path) -> dict[str, Any]:
    params = dict(DEFAULT_PARAMS)
    if path.exists():
        params.update(json.loads(path.read_text(encoding="utf-8")))
    return params


def save_params(path: Path, params: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"saved settings: {path}")


def recreate_stage_controls(stage_index: int, params: dict[str, Any]) -> None:
    try:
        cv2.destroyWindow(CONTROL_WINDOW)
    except cv2.error:
        pass
    cv2.namedWindow(CONTROL_WINDOW, cv2.WINDOW_NORMAL)
    slider_count = max(1, len(STAGES[stage_index]["sliders"]))
    cv2.resizeWindow(CONTROL_WINDOW, 520, 95 + slider_count * 42)
    for slider in STAGES[stage_index]["sliders"]:
        value = int(round(float(params[slider.key]) * slider.scale))
        cv2.createTrackbar(
            slider.label,
            CONTROL_WINDOW,
            min(slider.maximum, max(0, value)),
            slider.maximum,
            lambda _value: None,
        )


def read_stage_controls(stage_index: int, params: dict[str, Any]) -> dict[str, Any]:
    output = dict(params)
    for slider in STAGES[stage_index]["sliders"]:
        value = cv2.getTrackbarPos(slider.label, CONTROL_WINDOW)
        if slider.boolean:
            output[slider.key] = bool(value)
        else:
            output[slider.key] = value / slider.scale
            if slider.scale == 1.0:
                output[slider.key] = int(value)
    output["support_votes"] = max(1, int(output["support_votes"]))
    output["max_targets"] = max(1, int(output["max_targets"]))
    output["smooth_kernel"] = max(1, int(output["smooth_kernel"]))
    output["morph_kernel"] = max(1, int(output["morph_kernel"]))
    return output


def annotate(frame: np.ndarray, result: DetectionResult, roi: tuple[int, int, int, int] | None) -> np.ndarray:
    output = frame.copy()
    colors = [(0, 255, 255), (255, 0, 255)]
    for index, candidate in enumerate(result.candidates[:2]):
        color = colors[index]
        x, y, w, h = candidate["bbox"]
        cx, cy = candidate["centroid"]
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 3)
        cv2.circle(output, (int(cx), int(cy)), 7, color, -1)
        cv2.putText(
            output,
            f"#{index + 1} area={candidate['area']} conf={candidate['color_confidence']:.2f}",
            (x, max(25, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
    if roi:
        x, y, w, h = roi
        cv2.rectangle(output, (x, y), (x + w, y + h), (255, 255, 255), 3)
        cv2.putText(output, "AUTO-TUNE ROI", (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2)
    return output


def mask_to_panel(mask: np.ndarray, label: str) -> np.ndarray:
    panel = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    panel = cv2.resize(panel, (MASK_WIDTH, VIEW_HEIGHT), interpolation=cv2.INTER_NEAREST)
    cv2.rectangle(panel, (0, 0), (MASK_WIDTH, 42), (0, 0, 0), -1)
    cv2.putText(panel, label, (12, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
    return panel


def build_dashboard(
    frame: np.ndarray,
    result: DetectionResult,
    stage_index: int,
    tuning_mode: bool,
    paused: bool,
    roi: tuple[int, int, int, int] | None,
    status: str,
    export_status: str,
) -> np.ndarray:
    annotated = annotate(frame, result, roi)
    annotated = cv2.resize(annotated, (VIEW_WIDTH, VIEW_HEIGHT), interpolation=cv2.INTER_AREA)
    stage = STAGES[stage_index]
    if tuning_mode and stage["method"]:
        shown_mask = result.masks[stage["method"]]
        mask_label = f"{stage['method'].upper()} ONLY"
    else:
        shown_mask = result.masks["cleaned"]
        mask_label = "FINAL MASK"
    top = cv2.hconcat([annotated, mask_to_panel(shown_mask, mask_label)])

    guide = np.full((GUIDE_HEIGHT, top.shape[1], 3), 24, dtype=np.uint8)
    mode = "TUNE" if tuning_mode else "PLAY"
    state = "PAUSED" if paused else "RUNNING"
    title = f"{stage['name']}   mode={mode}   {state}   total={result.timings_ms['total']:.2f}ms"
    cv2.putText(guide, title, (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.78, (0, 255, 255), 2, cv2.LINE_AA)
    for index, line in enumerate(stage["guide"]):
        cv2.putText(guide, line, (24, 65 + index * 25), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (225, 225, 225), 1, cv2.LINE_AA)
    cv2.putText(
        guide,
        "[ / ] stage   M play/tune   SPACE pause   N/P frame   drag ROI + A auto-tune   C clear ROI",
        (16, 151),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (100, 230, 100),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        guide,
        "W save   L load   R reset   S snapshot   Q quit",
        (16, 177),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (100, 230, 100),
        1,
        cv2.LINE_AA,
    )
    if status:
        cv2.putText(guide, status[:105], (16, 201), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (0, 180, 255), 1, cv2.LINE_AA)
    button_specs = [
        ("export", "EXPORT FULL VIDEO", (45, 145, 45)),
        ("stop", "STOP EXPORT", (45, 45, 145)),
        ("snapshot", "SAVE SNAPSHOT", (100, 90, 25)),
        ("save", "SAVE SETTINGS", (100, 90, 25)),
    ]
    for name, label, color in button_specs:
        x1, y1, x2, y2 = BUTTONS[name]
        local_y1, local_y2 = y1 - VIEW_HEIGHT, y2 - VIEW_HEIGHT
        cv2.rectangle(guide, (x1, local_y1), (x2, local_y2), color, -1)
        cv2.rectangle(guide, (x1, local_y1), (x2, local_y2), (180, 180, 180), 1)
        cv2.putText(
            guide,
            label,
            (x1 + 12, local_y1 + 29),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.54,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
    cv2.putText(
        guide,
        export_status[:70],
        (945, 191),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.47,
        (0, 220, 255),
        1,
        cv2.LINE_AA,
    )
    return cv2.vconcat([top, guide])


def auto_tune(frame: np.ndarray, roi: tuple[int, int, int, int], method: str, params: dict[str, Any]) -> tuple[dict[str, Any], str]:
    x, y, w, h = roi
    crop = frame[y : y + h, x : x + w]
    if crop.size == 0:
        return params, "ROI is empty"
    pixels = crop.reshape(-1, 3)
    b = pixels[:, 0].astype(np.int16)
    g = pixels[:, 1].astype(np.int16)
    r = pixels[:, 2].astype(np.int16)
    exg = 2 * g - r - b
    green_like = (g > r) & (g > b) & (exg > 5)
    positive_reference = green_like.reshape(h, w)
    if np.count_nonzero(green_like) >= max(20, len(pixels) // 10):
        pixels = pixels[green_like]
        b, g, r, exg = b[green_like], g[green_like], r[green_like], exg[green_like]
    else:
        positive_reference = np.ones((h, w), dtype=bool)

    tuned = dict(params)
    q = lambda values, pct: int(round(float(np.percentile(values, pct))))
    if method == "bgr":
        tuned["g_min"] = max(0, q(g, 3) - 3)
        tuned["gr_min"] = max(0, q(g - r, 3) - 3)
        tuned["gb_min"] = max(0, q(g - b, 3) - 3)
    elif method == "exg":
        tuned["exg_min"] = max(0, q(exg, 3) - 5)
        tuned["exg_g_min"] = max(0, q(g, 3) - 3)
        tuned["exg_require_dominance"] = True
    elif method == "hsv":
        hsv_pixels = cv2.cvtColor(pixels.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_BGR2HSV).reshape(-1, 3)
        tuned["h_min"] = max(0, q(hsv_pixels[:, 0], 2) - 2)
        tuned["h_max"] = min(179, q(hsv_pixels[:, 0], 98) + 2)
        tuned["s_min"] = max(0, q(hsv_pixels[:, 1], 3) - 8)
        tuned["v_min"] = max(0, q(hsv_pixels[:, 2], 3) - 8)
    elif method == "lab":
        lab_pixels = cv2.cvtColor(pixels.reshape(-1, 1, 3).astype(np.uint8), cv2.COLOR_BGR2LAB).reshape(-1, 3)
        tuned["lab_l_min"] = max(0, q(lab_pixels[:, 0], 2) - 5)
        background_lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)[::8, ::8].reshape(-1, 3)
        best_score = -1e9
        best_range = None
        for a_pct in (1, 3, 5, 8, 12):
            for b_pct in (1, 3, 5, 8, 12):
                a_min = max(0, q(lab_pixels[:, 1], a_pct) - 3)
                a_max = min(255, q(lab_pixels[:, 1], 100 - a_pct) + 3)
                b_min = max(0, q(lab_pixels[:, 2], b_pct) - 3)
                b_max = min(255, q(lab_pixels[:, 2], 100 - b_pct) + 3)
                target_pass = (
                    (lab_pixels[:, 1] >= a_min)
                    & (lab_pixels[:, 1] <= a_max)
                    & (lab_pixels[:, 2] >= b_min)
                    & (lab_pixels[:, 2] <= b_max)
                )
                background_pass = (
                    (background_lab[:, 0] >= tuned["lab_l_min"])
                    & (background_lab[:, 1] >= a_min)
                    & (background_lab[:, 1] <= a_max)
                    & (background_lab[:, 2] >= b_min)
                    & (background_lab[:, 2] <= b_max)
                )
                recall = float(np.mean(target_pass))
                false_rate = float(np.mean(background_pass))
                score = recall - 8.0 * false_rate - (0.25 if recall < 0.90 else 0.0)
                if score > best_score:
                    best_score = score
                    best_range = (a_min, a_max, b_min, b_max)
        assert best_range is not None
        tuned["lab_a_min"], tuned["lab_a_max"], tuned["lab_b_min"], tuned["lab_b_max"] = best_range

    mask = make_single_color_mask(frame, tuned, method)
    target_mask = mask[y : y + h, x : x + w]
    target_recall = np.count_nonzero((target_mask > 0) & positive_reference) / max(
        np.count_nonzero(positive_reference), 1
    )
    outside_pixels = mask.size - w * h
    outside_white = cv2.countNonZero(mask) - cv2.countNonZero(target_mask)
    false_rate = outside_white / max(outside_pixels, 1)
    status = f"Auto {method.upper()}: ROI coverage={target_recall * 100:.1f}%  outside white={false_rate * 100:.2f}%"
    return tuned, status


def save_snapshot(frame: np.ndarray, result: DetectionResult, params: dict[str, Any], frame_index: int) -> None:
    stamp = time.strftime("%Y%m%d_%H%M%S")
    folder = SNAPSHOT_DIR / f"{stamp}_frame_{frame_index:06d}"
    folder.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(folder / "original.jpg"), frame)
    cv2.imwrite(str(folder / "annotated.jpg"), annotate(frame, result, None))
    for name, mask in result.masks.items():
        if isinstance(mask, np.ndarray) and mask.ndim == 2 and mask.dtype == np.uint8:
            cv2.imwrite(str(folder / f"{name}.png"), mask)
    (folder / "params.json").write_text(json.dumps(params, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"saved snapshot: {folder}")


def run_wizard(source: Path, config_path: Path) -> None:
    params = load_params(config_path)
    cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open video: {source}")
    source_fps = cap.get(cv2.CAP_PROP_FPS) or 50.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frame_period = 1.0 / source_fps

    cv2.namedWindow(MAIN_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(MAIN_WINDOW, VIEW_WIDTH + MASK_WIDTH, VIEW_HEIGHT + GUIDE_HEIGHT)
    cv2.namedWindow(PLAYBACK_WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(PLAYBACK_WINDOW, 520, 105)
    cv2.createTrackbar("Seek x0.1pct", PLAYBACK_WINDOW, 920, 1000, lambda _value: None)

    stage_index = 0
    recreate_stage_controls(stage_index, params)
    paused = True
    tuning_mode = True
    frame_index = min(total_frames - 1, int(total_frames * 0.92))
    cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
    current_frame = None
    result = None
    status = "Drag a tight box around the green cylinder, then press A for auto-tune."
    roi: tuple[int, int, int, int] | None = None
    drag_start: tuple[int, int] | None = None
    dragging = False
    last_seek = 920
    last_signature = None
    need_frame = True
    play_wall = time.perf_counter()
    play_start_frame = frame_index
    pending_action: str | None = None
    export_job = ExportJob()

    def mouse_callback(event: int, mx: int, my: int, _flags: int, _userdata: Any) -> None:
        nonlocal drag_start, dragging, roi, pending_action
        if event == cv2.EVENT_LBUTTONUP:
            for name, (x1, y1, x2, y2) in BUTTONS.items():
                if x1 <= mx <= x2 and y1 <= my <= y2:
                    pending_action = name
                    return
        if mx >= VIEW_WIDTH or my >= VIEW_HEIGHT:
            return
        sx = frame_width / VIEW_WIDTH
        sy = frame_height / VIEW_HEIGHT
        px = min(frame_width - 1, max(0, int(mx * sx)))
        py = min(frame_height - 1, max(0, int(my * sy)))
        if event == cv2.EVENT_LBUTTONDOWN:
            drag_start = (px, py)
            dragging = True
        elif event == cv2.EVENT_LBUTTONUP and dragging and drag_start:
            x0, y0 = drag_start
            x1, y1 = px, py
            x, y = min(x0, x1), min(y0, y1)
            w, h = abs(x1 - x0), abs(y1 - y0)
            if w >= 4 and h >= 4:
                roi = (x, y, w, h)
            dragging = False

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    cv2.setMouseCallback(MAIN_WINDOW, mouse_callback)

    print("Tuning wizard started")
    print("[ and ] change stage. M switches play/tune. Drag target ROI and press A to auto-tune.")

    while True:
        if pending_action:
            action = pending_action
            pending_action = None
            if action == "export":
                if export_job.start(source, params):
                    status = "Full 3x3 mask video export started in background."
                else:
                    status = "An export is already running."
            elif action == "stop":
                export_job.cancel()
                status = "Export cancellation requested. Partial video will be preserved."
            elif action == "snapshot" and current_frame is not None and result is not None:
                save_snapshot(current_frame, result, params, frame_index)
                status = "Snapshot saved."
            elif action == "save":
                save_params(LAST_CONFIG, params)
                status = f"Saved: {LAST_CONFIG}"

        requested_seek = cv2.getTrackbarPos("Seek x0.1pct", PLAYBACK_WINDOW)
        if abs(requested_seek - last_seek) > 1:
            frame_index = min(total_frames - 1, max(0, int(total_frames * requested_seek / 1000)))
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            last_seek = requested_seek
            paused = True
            need_frame = True

        if not paused:
            target_frame = min(total_frames - 1, play_start_frame + int((time.perf_counter() - play_wall) * source_fps))
            while frame_index < target_frame - 1:
                if not cap.grab():
                    break
                frame_index += 1
            need_frame = True

        if need_frame or current_frame is None:
            ok, frame = cap.read()
            if not ok:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                frame_index = 0
                paused = True
                need_frame = True
                continue
            current_frame = frame
            frame_index += 1
            last_seek = min(1000, int(frame_index * 1000 / max(total_frames, 1)))
            cv2.setTrackbarPos("Seek x0.1pct", PLAYBACK_WINDOW, last_seek)
            need_frame = False

        params = read_stage_controls(stage_index, params)
        signature = (stage_index, tuning_mode, tuple(sorted(params.items())), frame_index)
        if signature != last_signature or result is None:
            method = STAGES[stage_index]["method"]
            if tuning_mode and method:
                result = detect_single_filter(current_frame, params, method)
            else:
                result = detect_frame(current_frame, params)
            last_signature = signature

        dashboard = build_dashboard(
            current_frame,
            result,
            stage_index,
            tuning_mode,
            paused,
            roi,
            status,
            export_job.status(),
        )
        cv2.imshow(MAIN_WINDOW, dashboard)
        key = cv2.waitKey(1 if not paused else 25) & 0xFF

        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused
            if not paused:
                play_wall = time.perf_counter()
                play_start_frame = frame_index
        elif key == ord("]"):
            stage_index = min(len(STAGES) - 1, stage_index + 1)
            recreate_stage_controls(stage_index, params)
            last_signature = None
        elif key == ord("["):
            stage_index = max(0, stage_index - 1)
            recreate_stage_controls(stage_index, params)
            last_signature = None
        elif key == ord("m"):
            tuning_mode = not tuning_mode
            paused = tuning_mode
            if not paused:
                play_wall = time.perf_counter()
                play_start_frame = frame_index
            last_signature = None
        elif key == ord("n"):
            paused = True
            need_frame = True
        elif key == ord("p"):
            paused = True
            frame_index = max(0, frame_index - 2)
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
            need_frame = True
        elif key == ord("a"):
            method = STAGES[stage_index]["method"]
            if roi and method:
                params, status = auto_tune(current_frame, roi, method, params)
                recreate_stage_controls(stage_index, params)
                last_signature = None
            else:
                status = "Auto-tune requires BGR/ExG/HSV/Lab stage and a selected ROI."
        elif key == ord("c"):
            roi = None
            status = "ROI cleared."
        elif key == ord("w"):
            save_params(LAST_CONFIG, params)
            status = f"Saved: {LAST_CONFIG}"
        elif key == ord("l"):
            params = load_params(LAST_CONFIG if LAST_CONFIG.exists() else DEFAULT_CONFIG)
            recreate_stage_controls(stage_index, params)
            last_signature = None
            status = "Saved settings loaded."
        elif key == ord("r"):
            params = load_params(DEFAULT_CONFIG)
            recreate_stage_controls(stage_index, params)
            last_signature = None
            status = "Default settings restored."
        elif key == ord("s"):
            save_snapshot(current_frame, result, params, frame_index)
            status = "Snapshot saved."
        elif key == ord("e"):
            if export_job.start(source, params):
                status = "Full 3x3 mask video export started in background."
            else:
                status = "An export is already running."
        elif key == ord("x"):
            export_job.cancel()
            status = "Export cancellation requested. Partial video will be preserved."

    if export_job.running:
        export_job.cancel()
        if export_job.thread:
            export_job.thread.join(timeout=10.0)
    cap.release()
    cv2.destroyAllWindows()
