"""Raspberry Pi: detect the color tower and send packet V1 to the ESP32 over UART.

Detection runs as fast as the camera and CPU allow (about 25-40 fps on a Pi 5).
A small sender thread writes the latest packet every 20 ms (50 Hz, the protocol
requirement). The sequence changes only when a new frame was processed, so a
repeat of the same result never looks fresh to the ESP32 (README section 10).

If the camera or detection fails the program exits and sending stops; the ESP32
then drops the link after 120 ms and drives without vision. No auto restart.

  python3 scripts/pi_vision_sender.py                 # camera -> /dev/ttyAMA0
  python3 scripts/pi_vision_sender.py --dry-run       # no UART, print packets
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
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--search-width", type=int, default=960)
    parser.add_argument("--search-height", type=int, default=540)
    parser.add_argument("--fps", type=float, default=50.0)
    parser.add_argument("--max-frames", type=int, default=0, help="stop after N frames, 0 = run until Ctrl+C")
    parser.add_argument("--quiet", action="store_true", help="no once-per-second status line")
    args = parser.parse_args()

    cv2.setUseOptimized(True)
    cv2.setNumThreads(4)
    params = load_params(args.config)
    options = LightweightOptions(args.search_width, args.search_height)
    source = (
        PiCameraSource(args.width, args.height, args.search_width, args.search_height, args.fps)
        if args.source.lower() == "camera"
        else VideoSource(args.source)
    )
    sender = PacketSender(None if args.dry_run else args.port, args.baud)
    sender.start()

    sequence = 0
    stable = 0
    frames = 0
    last_report = time.perf_counter()
    report_frames = 0
    try:
        while True:
            ok, frame, search_frame = source.read()
            if not ok:
                break
            result = detect_frame_lightweight(
                frame, params, options, search_frame=search_frame, build_output_masks=False
            )
            stable = stable + 1 if result.candidates else 0
            sequence = (sequence + 1) & 0xFF
            packet = vision_packet.from_candidates(
                sequence, result.candidates, (frame.shape[1], frame.shape[0]), stable
            )
            sender.latest = packet
            frames += 1
            report_frames += 1
            now = time.perf_counter()
            if not args.quiet and now - last_report >= 1.0:
                fps = report_frames / (now - last_report)
                print(
                    f"detect {fps:5.1f} fps | sent {sender.sent} dropped {sender.dropped} | "
                    f"targets {len(result.candidates)} | {packet.hex(' ').upper()}",
                    flush=True,
                )
                last_report, report_frames = now, 0
            if args.max_frames and frames >= args.max_frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        sender.stop()
        source.close()
    print(f"done: {frames} frames, {sender.sent} packets sent, {sender.dropped} dropped", file=sys.stderr)


if __name__ == "__main__":
    main()
