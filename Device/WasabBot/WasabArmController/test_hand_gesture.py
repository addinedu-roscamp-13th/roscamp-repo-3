"""Test MediaPipe two-open-palm recognition without connecting to the robot."""
from __future__ import annotations

import argparse
import time

import cv2

from robot_client import config
from robot_client.hand_gesture import OpenPalmTrigger


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="print recognition status only (useful over SSH)",
    )
    args = parser.parse_args()

    cap = cv2.VideoCapture(config.CAMERA_ID, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_FRAME_HEIGHT)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera: {config.CAMERA_ID!r}")

    detector = OpenPalmTrigger(
        stable_frames=config.HAND_GESTURE_STABLE_FRAMES,
        release_frames=config.HAND_GESTURE_RELEASE_FRAMES,
        cooldown_sec=config.HAND_GESTURE_COOLDOWN_SEC,
        min_detection_confidence=config.HAND_GESTURE_MIN_DETECTION_CONFIDENCE,
        min_tracking_confidence=config.HAND_GESTURE_MIN_TRACKING_CONFIDENCE,
    )
    interval_sec = 1.0 / config.HAND_GESTURE_PROCESS_FPS
    previous_status = ""
    print(
        "[GESTURE TEST] Robot control is disabled. Hold both palms open until "
        "TWO PALMS reaches 20/20; Ctrl+C quits."
    )

    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            triggered, status = detector.process(frame)
            if status != previous_status or triggered:
                print(f"[GESTURE TEST] {status}", flush=True)
                previous_status = status

            if not args.no_window:
                color = (0, 255, 0) if triggered else (0, 200, 255)
                cv2.putText(
                    frame,
                    status,
                    (18, 34),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.65,
                    color,
                    2,
                    cv2.LINE_AA,
                )
                cv2.imshow("MediaPipe open-palm test (q: quit)", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            time.sleep(interval_sec)
    finally:
        detector.close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
