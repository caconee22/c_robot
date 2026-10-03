from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    out_dir = Path("output")
    out_dir.mkdir(exist_ok=True)

    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    frame[:] = (25, 25, 25)

    # Arena-like colored regions for a quick visual smoke test.
    cv2.circle(frame, (320, 240), 95, (20, 20, 210), -1)
    cv2.rectangle(frame, (70, 70), (185, 145), (220, 160, 20), -1)

    # Synthetic glossy green cylinder target.
    cv2.ellipse(frame, (420, 210), (55, 82), 0, 0, 360, (40, 190, 55), -1)
    cv2.ellipse(frame, (405, 175), (14, 30), -25, 0, 360, (245, 245, 245), -1)

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    lower_green = np.array([35, 45, 35])
    upper_green = np.array([90, 255, 255])
    green_mask = cv2.inRange(hsv, lower_green, upper_green)

    kernel = np.ones((5, 5), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    annotated = frame.copy()

    if contours:
        contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        center = (x + w // 2, y + h // 2)

        cv2.rectangle(annotated, (x, y), (x + w, y + h), (0, 255, 255), 2)
        cv2.circle(annotated, center, 5, (255, 255, 255), -1)
        cv2.putText(
            annotated,
            f"green target area={area:.0f}",
            (x, max(25, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.imwrite(str(out_dir / "opencv_demo_frame.png"), frame)
    cv2.imwrite(str(out_dir / "opencv_demo_mask.png"), green_mask)
    cv2.imwrite(str(out_dir / "opencv_demo_annotated.png"), annotated)

    print("OpenCV green-cylinder demo complete")
    print(f"contours: {len(contours)}")
    if contours:
        print(f"bbox: x={x}, y={y}, w={w}, h={h}")
        print(f"center: x={center[0]}, y={center[1]}")
    print(f"output: {out_dir / 'opencv_demo_annotated.png'}")


if __name__ == "__main__":
    main()
