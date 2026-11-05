"""Raspberry Pi: detect the color tower and send packet V1 to the ESP32 over UART.

Detection runs as fast as the camera and CPU allow. 11/04: 1280x720 (search 640x360)
gives ~45 fps and ~33 ms frame age vs ~33 fps / ~83 ms at 1920x1080; the packet
still uses the 1920x1080 scale.
A small sender thread writes the latest packet every 20 ms (50 Hz, the protocol
requirement). The sequence changes only when a new frame was processed, so a
repeat of the same result never looks fresh to the ESP32 (README section 10).

If the camera or detection fails the program exits and sending stops; the ESP32
then drops the link after 120 ms and drives without vision. No auto restart.

  python3 scripts/pi_vision_sender.py                 # camera -> /dev/ttyAMA0
  python3 scripts/pi_vision_sender.py --dry-run       # no UART, print packets
  python3 scripts/pi_vision_sender.py --detector legacy   # pre-11/04 4-filter detector
  python3 scripts/pi_vision_sender.py --preview       # live window (VNC desktop), bench only

--preview: the tower settings app (scripts/tower_settings_app.py) with the
live view, color/size/camera settings and presets. Detection and sending keep
running in their own thread. Stop the c-robot-vision service first, the camera
can only be opened once (the desktop icon does this).

Camera mode and controls come from config/camera.json (scripts/camera_settings.py);
--width/--height/--search-*/--fps override the mode.

Detector (11/04): default "hsv" = scripts/tower_detector_hsv.py (HSV + shape +
tracking, color range from config/tower_hsv.json saved by the calibrator; same
accuracy on the 10/06 videos, worst case 50 ms -> 2.3 ms on the Pi).
"legacy" = color_detection_lightweight with config/color_detector_last.json.
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from color_detection_lightweight import LightweightOptions, detect_frame_lightweight  # noqa: E402
from pi_green_tracker import DEFAULT_CONFIG, PiCameraSource, VideoSource, load_params  # noqa: E402
from tower_detector_hsv import TowerDetector  # noqa: E402
import camera_settings  # noqa: E402
import vision_packet  # noqa: E402

SEND_PERIOD_S = 0.020  # 50 Hz


class PacketSender(threading.Thread):
    """Writes the latest packet at a fixed period. Never waits on a full UART."""

    def __init__(self, port: str | None, baud: int) -> None:
        super().__init__(daemon=True)
        self.serial = None
        if port:
            import serial

            # write_timeout=0: a full buffer drops this packet instead of blocking.
            self.serial = serial.Serial(port, baud, write_timeout=0)
        self.latest: bytes | None = None
        self.sent = 0
        self.dropped = 0
        self.stopping = threading.Event()

    def run(self) -> None:
        next_time = time.perf_counter()
        while not self.stopping.is_set():
            packet = self.latest
            if packet is not None:
                if self.serial is None:
                    self.sent += 1
                else:
                    try:
                        self.serial.write(packet)
                        self.sent += 1
                    except Exception:  # SerialTimeoutException on a full buffer
                        self.dropped += 1
            next_time += SEND_PERIOD_S
            delay = next_time - time.perf_counter()
            if delay < -SEND_PERIOD_S:
                next_time = time.perf_counter()  # fell far behind: restart the grid
            elif delay > 0:
                time.sleep(delay)

    def stop(self) -> None:
        self.stopping.set()
        self.join(timeout=1.0)
        if self.serial is not None:
            self.serial.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="Pi vision -> ESP32 UART packet sender")
    parser.add_argument("--source", default="camera", help="camera, or a video path for testing")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="UART device (GPIO14 TX / GPIO15 RX)")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--dry-run", action="store_true", help="do not open the UART")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--width", type=int)
    parser.add_argument("--height", type=int)
    parser.add_argument("--search-width", type=int)
    parser.add_argument("--search-height", type=int)
    parser.add_argument("--fps", type=float)
    parser.add_argument("--detector", choices=("hsv", "legacy"), default="hsv")
    parser.add_argument("--preview", action="store_true", help="open the tower settings app")
    parser.add_argument("--preview-simple", action="store_true",
                        help="the earlier OpenCV window with sliders (scripts/vision_preview.py)")
    parser.add_argument("--max-frames", type=int, default=0, help="stop after N frames, 0 = run until Ctrl+C")
    parser.add_argument("--quiet", action="store_true", help="no once-per-second status line")
    args = parser.parse_args()
    if args.preview and args.detector != "hsv":
        parser.error("--preview calibrates the hsv detector; drop --detector legacy")

    cv2.setUseOptimized(True)
    cv2.setNumThreads(4)
    params = load_params(args.config)
    cam = camera_settings.load()
    (main_w, main_h), (search_w, search_h), mode_fps = camera_settings.MODES[cam["mode"]]
    args.width, args.height = args.width or main_w, args.height or main_h
    args.search_width, args.search_height = args.search_width or search_w, args.search_height or search_h
    args.fps = args.fps or mode_fps
    options = LightweightOptions(args.search_width, args.search_height)
    tower = TowerDetector()
    if args.source.lower() == "camera":
        source = PiCameraSource(args.width, args.height, args.search_width, args.search_height, args.fps)
        camera_settings.apply(source.camera, cam)
    else:
        source = VideoSource(args.source)
    sender = PacketSender(None if args.dry_run else args.port, args.baud)
    sender.start()
    stop = threading.Event()
    stats = {"frames": 0, "fps": 0.0, "detect_ms": 0.0, "age_ms": None, "targets": 0}

    def detect_loop(on_frame=None) -> None:
        sequence = stable = report_frames = 0
        last_report = time.perf_counter()
        while not stop.is_set():
            ok, frame, search_frame = source.read()
            if not ok:
                break
            started = time.perf_counter()
            if args.detector == "hsv":
                result = tower.detect(frame, search_frame)
            else:
                result = detect_frame_lightweight(
                    frame, params, options, search_frame=search_frame, build_output_masks=False
                )
            stats["detect_ms"] = (time.perf_counter() - started) * 1000
            stable = stable + 1 if result.candidates else 0
            sequence = (sequence + 1) & 0xFF
            packet = vision_packet.from_candidates(
                sequence, result.candidates, (frame.shape[1], frame.shape[0]), stable
            )
            sender.latest = packet
            stats["frames"] += 1
            stats["targets"] = len(result.candidates)
            stamp = getattr(source, "metadata", {}).get("SensorTimestamp")
            if stamp:
                stats["age_ms"] = (time.clock_gettime_ns(time.CLOCK_BOOTTIME) - stamp) / 1e6
            if on_frame:
                on_frame(frame, search_frame, result)
            report_frames += 1
            now = time.perf_counter()
            if now - last_report >= 1.0:
                stats["fps"] = report_frames / (now - last_report)
                if not args.quiet:
                    print(f"detect {stats['fps']:5.1f} fps | sent {sender.sent} dropped {sender.dropped} | "
                          f"targets {len(result.candidates)} | {packet.hex(' ').upper()}", flush=True)
                last_report, report_frames = now, 0
            if args.max_frames and stats["frames"] >= args.max_frames:
                break
        stop.set()

    try:
        if args.preview_simple:
            from vision_preview import EVERY, Preview  # needs a desktop; never loaded by the service
            preview = Preview(tower)
            detect_loop(lambda frame, search, result: preview.show(frame, search, result.candidates,
                                                                   stats["frames"])
                        if stats["frames"] % EVERY == 0 else None)
        elif args.preview:
            # Tk must own the main thread; detection and sending run beside it.
            from tower_settings_app import run_app  # needs a desktop; never loaded by the service
            run_app(detect_loop, stop, tower, source, sender, stats)
        else:
            detect_loop()
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        sender.stop()
        source.close()
    print(f"done: {stats['frames']} frames, {sender.sent} packets sent, {sender.dropped} dropped",
          file=sys.stderr)


if __name__ == "__main__":
    main()
