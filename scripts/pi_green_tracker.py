from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from color_detection_core import DEFAULT_PARAMS
    from color_detection_lightweight import LightweightOptions, detect_frame_lightweight
except ModuleNotFoundError:
    from scripts.color_detection_core import DEFAULT_PARAMS
    from scripts.color_detection_lightweight import LightweightOptions, detect_frame_lightweight


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = ROOT / "config" / "color_detector_last.json"
DEFAULT_VIDEO = ROOT / "dataset" / "videos" / "irc_highlight_1080p50.mp4"


def load_params(path: Path) -> dict[str, Any]:
    params = dict(DEFAULT_PARAMS)
    params.update(json.loads(path.read_text(encoding="utf-8")))
    return params


class VideoSource:
    def __init__(self, source: str, start_frame: int = 0) -> None:
        value: int | str = int(source) if source.isdigit() else source
        self.capture = cv2.VideoCapture(value)
        if not self.capture.isOpened():
            raise RuntimeError(f"영상을 열 수 없습니다: {source}")
        if start_frame > 0:
            self.capture.set(cv2.CAP_PROP_POS_FRAMES, start_frame)

    @property
    def fps(self) -> float:
        return float(self.capture.get(cv2.CAP_PROP_FPS) or 50.0)

    @property
    def total_frames(self) -> int:
        return int(self.capture.get(cv2.CAP_PROP_FRAME_COUNT))

    @property
    def position(self) -> int:
        return int(self.capture.get(cv2.CAP_PROP_POS_FRAMES))

    def seek(self, frame_index: int) -> None:
        target = min(max(0, int(frame_index)), max(0, self.total_frames - 1))
        self.capture.set(cv2.CAP_PROP_POS_FRAMES, target)

    def read(self):
        ok, frame = self.capture.read()
        return ok, frame, None

    def close(self) -> None:
        self.capture.release()


class PiCameraSource:
    def __init__(self, width: int, height: int, search_width: int, search_height: int, fps: float) -> None:
        try:
            from picamera2 import Picamera2
        except ModuleNotFoundError as exc:
            raise RuntimeError("Picamera2가 없습니다. 먼저 ./setup_raspberry_pi.sh을 실행하세요.") from exc

        self.camera = Picamera2()
        config = self.camera.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"},
            lores={"size": (search_width, search_height), "format": "RGB888"},
            controls={"FrameRate": fps},
            buffer_count=4,
        )
        self.camera.configure(config)
        self.camera.start()

    def read(self):
        request = self.camera.capture_request()
        try:
            frame = request.make_array("main")
            search = request.make_array("lores")
        finally:
            request.release()
        return True, frame, search

    def close(self) -> None:
        self.camera.stop()
        self.camera.close()


def print_targets(
    frame_number: int,
    result,
    process_ms: float,
    effective_fps: float,
    as_json: bool,
) -> None:
    targets = [
        {
            "id": index + 1,
            "x": round(float(candidate["centroid"][0]), 1),
            "y": round(float(candidate["centroid"][1]), 1),
            "area": int(candidate["area"]),
            "confidence": round(float(candidate["color_confidence"]), 3),
        }
        for index, candidate in enumerate(result.candidates)
    ]
    if as_json:
        print(
            json.dumps(
                {
                    "frame": frame_number,
                    "process_ms": round(process_ms, 2),
                    "fps": round(effective_fps, 2),
                    "targets": targets,
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        return
    compact = " ".join(
        f"#{item['id']} x={item['x']:.1f} y={item['y']:.1f} area={item['area']} conf={item['confidence']:.2f}"
        for item in targets
    )
    print(
        (
            f"frame={frame_number} fps={effective_fps:.1f} "
            f"process={process_ms:.2f}ms targets={len(targets)} {compact}"
        ).rstrip(),
        flush=True,
    )


def draw_preview(
    frame,
    result,
    process_ms: float,
    loop_values: list[float],
    benchmark_progress: str,
    show_masks: bool,
    exporting: bool,
    video_position: int,
    total_frames: int,
):
    source_h, source_w = frame.shape[:2]
    canvas_w, canvas_h = 1280, 720
    if show_masks:
        image_x, image_y, image_w, image_h = 0, 90, 960, 540
    else:
        image_x, image_y, image_w, image_h = 0, 0, canvas_w, canvas_h
    scale_x = image_w / source_w
    scale_y = image_h / source_h
    output = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)
    output[image_y : image_y + image_h, image_x : image_x + image_w] = cv2.resize(
        frame,
        (image_w, image_h),
        interpolation=cv2.INTER_AREA,
    )
    colors = [(0, 255, 255), (255, 0, 255)]
    for index, candidate in enumerate(result.candidates):
        x, y, w, h = candidate["bbox"]
        cx, cy = candidate["centroid"]
        x, y = round(x * scale_x) + image_x, round(y * scale_y) + image_y
        w, h = round(w * scale_x), round(h * scale_y)
        cx, cy = cx * scale_x + image_x, cy * scale_y + image_y
        color = colors[index]
        cv2.rectangle(output, (x, y), (x + w, y + h), color, 3)
        cv2.circle(output, (round(cx), round(cy)), 6, color, -1)
        cv2.putText(
            output,
            f"#{index + 1} ({cx:.0f}, {cy:.0f})",
            (x, max(25, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            color,
            2,
            cv2.LINE_AA,
        )
    recent = np.asarray(loop_values[-60:], dtype=np.float64)
    avg_loop = float(recent.mean()) if recent.size else 0.0
    effective_fps = 1000.0 / avg_loop if avg_loop > 0 else 0.0
    cv2.rectangle(output, (0, 0), (600, 105), (0, 0, 0), -1)
    lines = [
        f"DETECT {process_ms:.2f} ms",
        f"DISPLAY LOOP {avg_loop:.2f} ms   {effective_fps:.1f} FPS",
        f"{benchmark_progress}   M masks   E export full   Q quit",
    ]
    for index, text in enumerate(lines):
        cv2.putText(
            output,
            text,
            (15, 32 + index * 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    mask_button = (610, 17, 160, 45)
    export_button = (785, 17, 160, 45)
    for (x, y, w, h), label, active in (
        (mask_button, f"MASKS {'ON' if show_masks else 'OFF'}", show_masks),
        (export_button, "EXPORTING" if exporting else "EXPORT FULL", exporting),
    ):
        color = (30, 150, 30) if active else (70, 70, 70)
        if label.startswith("EXPORT") and active:
            color = (30, 30, 210)
        cv2.rectangle(output, (x, y), (x + w, y + h), color, -1)
        cv2.rectangle(output, (x, y), (x + w, y + h), (230, 230, 230), 2)
        cv2.putText(output, label, (x + 12, y + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 2, cv2.LINE_AA)

    if exporting:
        cv2.circle(output, (1205, 41), 10, (0, 0, 255), -1)
        cv2.putText(output, "EXPORT", (1223, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)

    if show_masks:
        names = ["proposal", "bgr", "exg", "hsv", "lab", "core", "support", "fused", "cleaned", "selected"]
        tile_w, tile_h = 160, 144
        for index, name in enumerate(names):
            mask = result.masks.get(name)
            if mask is None:
                mask = np.zeros((tile_h, tile_w), dtype=np.uint8)
            elif name == "agreement":
                mask = np.clip(mask.astype(np.uint16) * 63, 0, 255).astype(np.uint8)
            mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask_bgr = cv2.resize(mask_bgr, (tile_w, tile_h), interpolation=cv2.INTER_NEAREST)
            tx = 960 + (index % 2) * tile_w
            ty = (index // 2) * tile_h
            output[ty : ty + tile_h, tx : tx + tile_w] = mask_bgr
            cv2.rectangle(output, (tx, ty), (tx + tile_w - 1, ty + tile_h - 1), (80, 80, 80), 1)
            cv2.rectangle(output, (tx, ty), (tx + tile_w, ty + 27), (0, 0, 0), -1)
            cv2.putText(output, name.upper(), (tx + 7, ty + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1, cv2.LINE_AA)

    if total_frames > 0:
        bar_x, bar_y, bar_w, bar_h = 40, 688, 1200, 14
        ratio = min(1.0, max(0.0, video_position / max(1, total_frames - 1)))
        cv2.rectangle(output, (bar_x - 8, bar_y - 24), (bar_x + bar_w + 8, bar_y + 26), (0, 0, 0), -1)
        cv2.rectangle(output, (bar_x, bar_y), (bar_x + bar_w, bar_y + bar_h), (85, 85, 85), -1)
        cv2.rectangle(output, (bar_x, bar_y), (bar_x + round(bar_w * ratio), bar_y + bar_h), (0, 220, 255), -1)
        knob_x = bar_x + round(bar_w * ratio)
        cv2.circle(output, (knob_x, bar_y + bar_h // 2), 10, (255, 255, 255), -1)
        cv2.putText(output, f"{video_position}/{total_frames}", (bar_x, bar_y - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def save_benchmark(
    preview: bool,
    source: str,
    process_values: list[float],
    loop_values: list[float],
) -> Path:
    process = np.asarray(process_values, dtype=np.float64)
    loop = np.asarray(loop_values, dtype=np.float64)
    report = {
        "mode": "preview" if preview else "headless",
        "source": source,
        "frames": int(loop.size),
        "detect_avg_ms": float(process.mean()),
        "detect_p95_ms": float(np.percentile(process, 95)),
        "loop_avg_ms": float(loop.mean()),
        "loop_p95_ms": float(np.percentile(loop, 95)),
        "effective_fps": float(1000.0 / max(loop.mean(), 1e-9)),
        "over_20ms": int(np.count_nonzero(loop > 20.0)),
        "over_33_33ms": int(np.count_nonzero(loop > 33.3333)),
        "over_41_67ms": int(np.count_nonzero(loop > 41.6667)),
    }
    output_dir = ROOT / "output" / "color_detector" / "benchmarks"
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / f"pi_tracker_{report['mode']}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print("\nBENCHMARK RESULT")
    for key, value in report.items():
        if isinstance(value, float):
            print(f"{key}: {value:.3f}")
        else:
            print(f"{key}: {value}")
    print(f"saved: {path}")
    return path


def main() -> None:
    parser = argparse.ArgumentParser(description="Raspberry Pi green target tracker")
    parser.add_argument(
        "--source",
        default=str(DEFAULT_VIDEO),
        help="영상 경로, camera 또는 USB 카메라 번호",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--search-width", type=int, default=960)
    parser.add_argument("--search-height", type=int, default=540)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--preview", action="store_true", help="검출 화면 표시")
    parser.add_argument("--json", action="store_true", help="좌표를 JSON Lines로 출력")
    parser.add_argument("--print-every", type=int, default=1, help="N프레임마다 좌표 출력")
    parser.add_argument("--max-frames", type=int, default=0, help="시험용 프레임 제한, 0은 무제한")
    parser.add_argument("--start-frame", type=int, default=0, help="영상 파일 시험 시작 프레임")
    parser.add_argument("--benchmark", action="store_true", help="검출 및 전체 반복 성능 측정")
    parser.add_argument("--warmup-frames", type=int, default=10, help="측정에서 제외할 초기 프레임")
    parser.add_argument("--quiet", action="store_true", help="프레임별 좌표 출력 끄기")
    parser.add_argument("--realtime", action="store_true", help="영상 원본 FPS에 맞춰 재생")
    parser.add_argument("--loop", action="store_true", help="영상이 끝나면 처음부터 반복")
    parser.add_argument("--show-masks", action="store_true", help="마스크 타일을 켠 상태로 시작")
    parser.add_argument("--export-on-start", action="store_true", help="전체 영상 내보내기를 바로 시작")
    args = parser.parse_args()

    cv2.setUseOptimized(True)
    cv2.setNumThreads(4)
    if hasattr(cv2, "ocl"):
        cv2.ocl.setUseOpenCL(False)

    params = load_params(args.config)
    options = LightweightOptions(args.search_width, args.search_height)
    source = (
        PiCameraSource(args.width, args.height, args.search_width, args.search_height, args.fps)
        if args.source.lower() == "camera"
        else VideoSource(args.source, args.start_frame)
    )

    frame_number = 0
    process_values: list[float] = []
    loop_values: list[float] = []
    seek_request: list[int | None] = [None]
    ui_state = {"show_masks": bool(args.show_masks), "start_export": bool(args.export_on_start)}
    export_process: subprocess.Popen | None = None
    export_path: Path | None = None
    if args.preview:
        cv2.namedWindow("Pi green tracker", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Pi green tracker", 1280, 720)
        cv2.setWindowProperty(
            "Pi green tracker",
            cv2.WND_PROP_FULLSCREEN,
            cv2.WINDOW_FULLSCREEN,
        )

        def on_mouse(event: int, x: int, y: int, _flags: int, _data: Any) -> None:
            if isinstance(source, VideoSource) and 670 <= y <= 719 and (
                event == cv2.EVENT_LBUTTONDOWN
                or (event == cv2.EVENT_MOUSEMOVE and (_flags & cv2.EVENT_FLAG_LBUTTON))
            ):
                ratio = min(1.0, max(0.0, (x - 40) / 1200.0))
                seek_request[0] = round(ratio * max(0, source.total_frames - 1))
                return
            if event != cv2.EVENT_LBUTTONDOWN:
                return
            if 610 <= x <= 770 and 17 <= y <= 62:
                ui_state["show_masks"] = not ui_state["show_masks"]
            elif 785 <= x <= 945 and 17 <= y <= 62:
                ui_state["start_export"] = True

        cv2.setMouseCallback("Pi green tracker", on_mouse)
    try:
        while True:
            loop_started = time.perf_counter()
            if export_process is not None and export_process.poll() is not None:
                print(f"full export finished: {export_path} (exit={export_process.returncode})")
                export_process = None
            if isinstance(source, VideoSource) and seek_request[0] is not None:
                source.seek(seek_request[0])
                seek_request[0] = None
                loop_values.clear()
            ok, frame, search_frame = source.read()
            if not ok:
                if args.loop and isinstance(source, VideoSource):
                    source.seek(0)
                    loop_values.clear()
                    continue
                break
            result = detect_frame_lightweight(
                frame,
                params,
                options,
                debug_masks=bool(ui_state["show_masks"]),
                search_frame=search_frame,
                build_output_masks=False,
            )
            process_ms = float(result.timings_ms["total"])
            frame_number += 1
            if not args.quiet and frame_number % max(1, args.print_every) == 0:
                recent_loops = np.asarray(loop_values[-60:], dtype=np.float64)
                live_fps = (
                    float(1000.0 / recent_loops.mean())
                    if recent_loops.size
                    else 0.0
                )
                print_targets(frame_number, result, process_ms, live_fps, args.json)
            if args.preview:
                progress = (
                    f"BENCH {max(0, frame_number - args.warmup_frames)}/{max(0, args.max_frames - args.warmup_frames)}"
                    if args.benchmark and args.max_frames > 0
                    else (
                        f"VIDEO {source.position}/{source.total_frames}"
                        if isinstance(source, VideoSource)
                        else "LIVE CAMERA"
                    )
                )
                preview_frame = draw_preview(
                    frame,
                    result,
                    process_ms,
                    loop_values,
                    progress,
                    bool(ui_state["show_masks"]),
                    export_process is not None and export_process.poll() is None,
                    source.position if isinstance(source, VideoSource) else 0,
                    source.total_frames if isinstance(source, VideoSource) else 0,
                )
                cv2.imshow("Pi green tracker", preview_frame)
                key = cv2.pollKey() if hasattr(cv2, "pollKey") else cv2.waitKey(1)
                if key & 0xFF == ord("q"):
                    break
                if key & 0xFF == ord("m"):
                    ui_state["show_masks"] = not ui_state["show_masks"]
                if key & 0xFF == ord("e"):
                    ui_state["start_export"] = True
                if ui_state["start_export"]:
                    ui_state["start_export"] = False
                    if not isinstance(source, VideoSource):
                        print("전체 영상 내보내기는 영상 파일 입력에서만 사용할 수 있습니다.")
                    elif export_process is None or export_process.poll() is not None:
                        export_dir = ROOT / "output" / "color_detector" / "exports"
                        export_dir.mkdir(parents=True, exist_ok=True)
                        export_path = export_dir / f"lightweight_full_masks_{datetime.now().strftime('%Y%m%d_%H%M%S')}.mp4"
                        export_process = subprocess.Popen(
                            [
                                sys.executable,
                                str(ROOT / "scripts" / "export_lightweight_full_video.py"),
                                "--source",
                                args.source,
                                "--config",
                                str(args.config),
                                "--output",
                                str(export_path),
                            ],
                            cwd=str(ROOT),
                        )
                        print(f"full export started: {export_path}")
            if args.realtime and isinstance(source, VideoSource):
                deadline = loop_started + (1.0 / max(source.fps, 1.0))
                # Yield while there is time left, then use a very short final
                # precision wait without starving the GUI event thread.
                while True:
                    remaining = deadline - time.perf_counter()
                    if remaining <= 0:
                        break
                    if remaining > 0.002:
                        time.sleep(0)
            loop_ms = (time.perf_counter() - loop_started) * 1000.0
            if frame_number > max(0, args.warmup_frames):
                if args.benchmark:
                    process_values.append(process_ms)
                loop_values.append(loop_ms)
                if not args.benchmark:
                    del loop_values[:-60]
            if args.max_frames > 0 and frame_number >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        source.close()
        cv2.destroyAllWindows()
    if args.benchmark and loop_values:
        save_benchmark(args.preview, args.source, process_values, loop_values)
    print(f"done: {frame_number} frames", file=sys.stderr)


if __name__ == "__main__":
    main()
