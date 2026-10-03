from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2


def find_cascade() -> str:
    cascade_path = Path(cv2.data.haarcascades) / "haarcascade_frontalface_default.xml"
    if not cascade_path.exists():
        raise FileNotFoundError(f"Missing OpenCV Haar cascade: {cascade_path}")
    return str(cascade_path)


def detect_faces(frame, detector):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    return detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40))


def draw_faces(frame, faces, fps=None):
    for i, (x, y, w, h) in enumerate(faces, 1):
        center = (x + w // 2, y + h // 2)
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.circle(frame, center, 4, (0, 255, 255), -1)
        cv2.putText(
            frame,
            f"Face #{i} ({center[0]}, {center[1]})",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    status = f"faces={len(faces)}"
    if fps is not None:
        status += f" fps={fps:.1f}"
    cv2.putText(frame, status, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (40, 220, 40), 2)


def run_image(source: Path, output: Path) -> None:
    detector = cv2.CascadeClassifier(find_cascade())
    frame = cv2.imread(str(source))
    if frame is None:
        raise RuntimeError(f"Could not read image: {source}")

    t0 = time.perf_counter()
    faces = detect_faces(frame, detector)
    latency_ms = (time.perf_counter() - t0) * 1000.0
    draw_faces(frame, faces)

    output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output), frame)
    print("face detection demo complete")
    print(f"source: {source}")
    print(f"faces: {len(faces)}")
    print(f"latency_ms: {latency_ms:.2f}")
    print(f"output: {output}")


def run_camera(camera_index: int) -> None:
    detector = cv2.CascadeClassifier(find_cascade())
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        raise RuntimeError(f"Could not open camera index {camera_index}")

    prev = time.perf_counter()
    while True:
        ok, frame = cap.read()
        if not ok:
            break

        now = time.perf_counter()
        fps = 1.0 / max(now - prev, 1e-6)
        prev = now

        faces = detect_faces(frame, detector)
        draw_faces(frame, faces, fps=fps)
        cv2.imshow("Face tracking demo - press q to quit", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


def main() -> None:
    parser = argparse.ArgumentParser(description="Basic OpenCV face detection/tracking smoke test.")
    parser.add_argument("--source", type=Path, help="Image file to test.")
    parser.add_argument("--camera", type=int, help="Camera index to run live tracking.")
    parser.add_argument("--output", type=Path, default=Path("output/face_tracking_demo.jpg"))
    args = parser.parse_args()

    if args.camera is not None:
        run_camera(args.camera)
    elif args.source:
        run_image(args.source, args.output)
    else:
        raise SystemExit("Use --source image.jpg or --camera 0")


if __name__ == "__main__":
    main()
