"""Tracking viewer for the Pi desktop: shows what the running sender sees.

Unlike "C_ROBOT 타워 설정" this does not stop the c-robot-vision service. The
sender keeps detecting and sending at full rate and hands this window a small
frame now and then through /dev/shm (scripts/view_tap.py). All drawing and
display happen here, at low priority (the desktop icon starts it with nice).

  python3 scripts/vision_viewer.py        # q or Esc to close
"""
from __future__ import annotations

import os
import signal
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import view_tap as T  # noqa: E402

WINDOW = "C_ROBOT 추적 화면"
SCALE = 1.5          # window size vs the small search frame
SHOW_HZ = 15
STALE_S = 1.0


def main() -> None:
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))   # closing the terminal/timeout: clean up too
    signal.signal(signal.SIGHUP, lambda *_: sys.exit(0))
    m = T.open_map(create=True)
    if m is None:
        sys.exit(f"cannot create {T.PATH}")
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    last_seq = -1
    image = None
    try:
        while True:
            now = time.monotonic()
            struct.pack_into("<d", m, 4, now)                  # tell the sender someone watches
            seq = struct.unpack_from("<I", m, 0)[0]
            if seq != last_seq and seq % 2 == 0 and seq:
                written, w, h, fw, fh, n, fps, det, age, *boxes = T.HEADER.unpack_from(m, 12)
                frame = np.frombuffer(m, np.uint8, h * w * 3, T.HEADER_SIZE).reshape(h, w, 3).copy()
                if struct.unpack_from("<I", m, 0)[0] == seq:     # not overwritten while copying
                    last_seq = seq
                    image = draw(frame, fw, fh, n, boxes, fps, det, age)
            stale = image is None or now - struct.unpack_from("<d", m, 12)[0] > STALE_S
            shown = waiting() if image is None else image.copy()
            if stale and image is not None:
                cv2.putText(shown, "NO NEW FRAMES (service stopped?)", (10, shown.shape[0] - 14),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow(WINDOW, shown)
            key = cv2.waitKey(int(1000 / SHOW_HZ)) & 0xFF
            if key in (ord("q"), 27) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        m.close()
        try:
            os.remove(T.PATH)
        except OSError:
            pass


def waiting() -> np.ndarray:
    img = np.zeros((int(360 * SCALE), int(640 * SCALE), 3), np.uint8)
    cv2.putText(img, "waiting for c-robot-vision ...", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (200, 200, 200), 2)
    return img


def draw(frame, fw, fh, n, boxes, fps, det, age) -> np.ndarray:
    h, w = frame.shape[:2]
    img = cv2.resize(frame, (int(w * SCALE), int(h * SCALE)), interpolation=cv2.INTER_LINEAR)
    sx, sy = img.shape[1] / max(fw, 1), img.shape[0] / max(fh, 1)
    cx = img.shape[1] // 2
    cv2.line(img, (cx, 0), (cx, img.shape[0]), (255, 255, 0), 1)
    for i in range(n):
        x, y, bw, bh = boxes[i * 4:i * 4 + 4]
        p0, p1 = (int(x * sx), int(y * sy)), (int((x + bw) * sx), int((y + bh) * sy))
        color = (0, 255, 0) if i == 0 else (0, 165, 255)       # chosen tower green, others orange
        cv2.rectangle(img, p0, p1, color, 3 if i == 0 else 1)
        if i == 0:
            mx = (p0[0] + p1[0]) // 2
            cv2.line(img, (mx, p0[1]), (mx, p1[1]), color, 1)
            # packet scale is 1920x1080: same numbers the ESP32 compares (ATTACK_BOX_HEIGHT 280)
            cv2.putText(img, f"x {int((x + bw / 2) * 1920 / max(fw, 1))}  h {int(bh * 1080 / max(fh, 1))}",
                        (p0[0], max(20, p0[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    status = f"detect {fps:4.1f} fps  {det:4.1f} ms  age {age:3.0f} ms  targets {n}"
    cv2.rectangle(img, (0, 0), (img.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(img, status, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
    return img


if __name__ == "__main__":
    main()
