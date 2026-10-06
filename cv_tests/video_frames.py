"""Index frames by sequential decoding; phone/VFR seeking can return other frames."""

import cv2


def selected_frames(video, indices):
    requested = sorted(set(indices))
    if not requested:
        return
    if requested[0] < 0:
        raise ValueError("Frame indices must be nonnegative")
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open {video}")
    cursor = 0
    index = 0
    try:
        while cursor < len(requested):
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f"Missing requested source frame {requested[cursor]}")
            if index == requested[cursor]:
                yield index, frame
                cursor += 1
            index += 1
    finally:
        cap.release()
