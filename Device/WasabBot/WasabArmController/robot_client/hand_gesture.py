"""MediaPipe single-palm hold recognizer."""
from __future__ import annotations

import time
import threading
from typing import Any

import cv2


class OpenPalmTrigger:
    """Recognize one open palm held continuously for three seconds."""

    _FINGER_CHAINS = (
        (5, 6, 7, 8),
        (9, 10, 11, 12),
        (13, 14, 15, 16),
        (17, 18, 19, 20),
    )
    _HAND_CONNECTIONS = (
        (0, 1), (1, 2), (2, 3), (3, 4),
        (0, 5), (5, 6), (6, 7), (7, 8),
        (5, 9), (9, 10), (10, 11), (11, 12),
        (9, 13), (13, 14), (14, 15), (15, 16),
        (13, 17), (17, 18), (18, 19), (19, 20), (0, 17),
    )
    def __init__(
        self,
        *,
        stable_frames: int,
        release_frames: int,
        cooldown_sec: float,
        min_detection_confidence: float,
        min_tracking_confidence: float,
        hitbox: tuple[float, float, float, float] | None = None,
        hold_sec: float = 3.0,
        min_palm_span_norm: float = 0.07,
        max_palm_span_norm: float = 0.30,
        edge_margin_norm: float = 0.06,
        min_palm_v_norm: float = 0.0,
    ) -> None:
        try:
            import mediapipe as mp
        except ImportError as exc:
            raise RuntimeError(
                "MediaPipe is required when [hand_gesture] enabled=true. "
                "Install Device/WasabBot/WasabArmController/requirements.txt."
            ) from exc

        self._hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            model_complexity=0,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence,
        )
        self._stable_frames = stable_frames
        self._hold_sec = max(0.1, float(hold_sec))
        self._min_palm_span_norm = float(min_palm_span_norm)
        self._max_palm_span_norm = float(max_palm_span_norm)
        self._edge_margin_norm = float(edge_margin_norm)
        self._min_palm_v_norm = float(min_palm_v_norm)
        self._release_frames = release_frames
        self._cooldown_sec = cooldown_sec
        self._open_count = 0
        self._release_count = 0
        self._armed = True
        self._last_trigger_at = float("-inf")
        self._open_started_at: float | None = None
        self._landmark_lock = threading.Lock()
        self._last_landmarks: list[list[tuple[float, float]]] = []
        self._progress_stage = 0
        self._recognized = False
        self._palm_center_norm: tuple[float, float] | None = None
        self._guidance = "SHOW ONE OPEN PALM"
        self._hitbox = hitbox

    def set_hitbox(self, hitbox: tuple[float, float, float, float]) -> None:
        """Set a normalized display-only hitbox."""
        with self._landmark_lock:
            self._hitbox = hitbox

    @staticmethod
    def _distance_sq(first: Any, second: Any) -> float:
        return (first.x - second.x) ** 2 + (first.y - second.y) ** 2

    def _is_open_palm(self, landmarks: list[Any]) -> bool:
        wrist = landmarks[0]
        extended = 0
        for mcp_index, pip_index, dip_index, tip_index in self._FINGER_CHAINS:
            mcp = landmarks[mcp_index]
            pip = landmarks[pip_index]
            dip = landmarks[dip_index]
            tip = landmarks[tip_index]
            first_x, first_y = pip.x - mcp.x, pip.y - mcp.y
            second_x, second_y = tip.x - dip.x, tip.y - dip.y
            first_len_sq = first_x * first_x + first_y * first_y
            second_len_sq = second_x * second_x + second_y * second_y
            same_direction = (
                first_len_sq > 1e-6
                and second_len_sq > 1e-6
                and (first_x * second_x + first_y * second_y)
                / (first_len_sq * second_len_sq) ** 0.5
                > 0.20
            )
            tip_clear_of_pip = (
                self._distance_sq(tip, wrist)
                > self._distance_sq(pip, wrist) * 1.02
            )
            if same_direction and tip_clear_of_pip:
                extended += 1
        palm_size_sq = self._distance_sq(landmarks[9], wrist)
        # Three straight fingers are sufficient because perspective can shorten
        # one finger when a normal palm faces the camera.
        return extended >= 3 and palm_size_sq >= 0.0016

    def detect_valid_open_palm(
        self,
        frame,
    ) -> tuple[bool, tuple[float, float] | None, str]:
        """Inspect one frame without changing the three-second trigger state."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self._hands.process(rgb)
        hands = list(result.multi_hand_landmarks or [])
        one_open_palm = len(hands) == 1 and self._is_open_palm(hands[0].landmark)
        palm_center: tuple[float, float] | None = None
        guidance = "SHOW ONE OPEN PALM"
        valid_palm = False
        if one_open_palm:
            landmarks = hands[0].landmark
            palm_center = (
                sum(landmarks[index].x for index in (0, 5, 9, 13, 17)) / 5.0,
                sum(landmarks[index].y for index in (0, 5, 9, 13, 17)) / 5.0,
            )
            palm_span = self._distance_sq(landmarks[0], landmarks[9]) ** 0.5
            margin = self._edge_margin_norm
            if not (
                margin <= palm_center[0] <= 1.0 - margin
                and margin <= palm_center[1] <= 1.0 - margin
            ):
                guidance = "MOVE TO CENTER"
            elif palm_center[1] < self._min_palm_v_norm:
                guidance = "SHOW ONE OPEN PALM"
            elif palm_span < self._min_palm_span_norm:
                guidance = "SHOW ONE OPEN PALM"
            elif palm_span > self._max_palm_span_norm:
                guidance = "MOVE BACK"
            else:
                guidance = "HOLD STILL"
                valid_palm = True
        with self._landmark_lock:
            self._last_landmarks = (
                [
                    [(float(item.x), float(item.y)) for item in hand.landmark]
                    for hand in hands
                ]
                if valid_palm
                else []
            )
            self._palm_center_norm = palm_center if valid_palm else None
            self._guidance = guidance
        return valid_palm, palm_center, guidance

    def detect_hand_presence(
        self,
        frame,
    ) -> tuple[bool, tuple[float, float] | None, str]:
        """Detect MediaPipe hand landmarks, with a partial skin/finger fallback."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self._hands.process(rgb)
        hands = list(result.multi_hand_landmarks or [])
        if not hands:
            fallback_center = self._detect_skin_finger_region(frame)
            if fallback_center is not None:
                with self._landmark_lock:
                    self._last_landmarks = []
                    self._palm_center_norm = fallback_center
                    self._guidance = "SKIN/FINGERS DETECTED"
                return True, fallback_center, "SKIN/FINGERS DETECTED"
            with self._landmark_lock:
                self._last_landmarks = []
                self._palm_center_norm = None
                self._guidance = "SHOW SKIN/FINGERS"
            return False, None, "SHOW SKIN/FINGERS"

        landmarks = hands[0].landmark
        hand_center = (
            sum(landmarks[index].x for index in (0, 5, 9, 13, 17)) / 5.0,
            sum(landmarks[index].y for index in (0, 5, 9, 13, 17)) / 5.0,
        )
        with self._landmark_lock:
            self._last_landmarks = [
                [(float(item.x), float(item.y)) for item in hand.landmark]
                for hand in hands
            ]
            self._palm_center_norm = hand_center
            self._guidance = "HAND DETECTED"
        return True, hand_center, "HAND DETECTED"

    @staticmethod
    def _detect_skin_finger_region(
        frame,
    ) -> tuple[float, float] | None:
        """Return a partial-hand center from skin color plus finger-like shape."""
        height, width = frame.shape[:2]
        ycrcb = cv2.cvtColor(frame, cv2.COLOR_BGR2YCrCb)
        skin_mask = cv2.inRange(
            ycrcb,
            (35, 125, 65),
            (255, 185, 145),
        )
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        skin_mask = cv2.morphologyEx(
            skin_mask,
            cv2.MORPH_OPEN,
            kernel,
        )
        skin_mask = cv2.morphologyEx(
            skin_mask,
            cv2.MORPH_CLOSE,
            kernel,
            iterations=2,
        )
        contours, _ = cv2.findContours(
            skin_mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        for contour in sorted(contours, key=cv2.contourArea, reverse=True):
            area = cv2.contourArea(contour)
            if area < 700.0:
                continue
            x, y, box_width, box_height = cv2.boundingRect(contour)
            short_side = max(1, min(box_width, box_height))
            long_side = max(box_width, box_height)
            finger_like = long_side >= 35 and long_side / short_side >= 1.6
            defect_count = 0
            hull = cv2.convexHull(contour, returnPoints=False)
            if hull is not None and len(hull) >= 4 and len(contour) >= 5:
                defects = cv2.convexityDefects(contour, hull)
                if defects is not None:
                    defect_count = sum(
                        1
                        for defect in defects[:, 0]
                        if defect[3] / 256.0 >= 6.0
                    )
            if not finger_like and defect_count < 1:
                continue
            moments = cv2.moments(contour)
            if abs(moments["m00"]) < 1e-6:
                continue
            center_x = moments["m10"] / moments["m00"]
            center_y = moments["m01"] / moments["m00"]
            return center_x / width, center_y / height
        return None

    def process(self, frame) -> tuple[bool, str]:
        valid_palm, palm_center, guidance = self.detect_valid_open_palm(frame)
        distance_rejected = (
            not valid_palm
            and guidance == "SHOW ONE OPEN PALM"
            and palm_center is not None
        )
        now = time.monotonic()
        if valid_palm:
            if self._open_started_at is None:
                self._open_started_at = now
            held_sec = now - self._open_started_at
            self._open_count += 1
            self._release_count = 0
            progress_stage = min(3, int(held_sec))
        else:
            held_sec = 0.0
            progress_stage = 0
            self._open_started_at = None
            self._open_count = 0
            self._release_count += 1
            if distance_rejected:
                self._armed = True
                self._recognized = False
            if self._release_count >= self._release_frames:
                self._armed = True
                self._recognized = False

        triggered = (
            self._armed
            and valid_palm
            and held_sec >= self._hold_sec
            and now - self._last_trigger_at >= self._cooldown_sec
        )
        if triggered:
            progress_stage = 3
            self._recognized = True
            guidance = "RECOGNIZED"
        elif self._recognized:
            progress_stage = 3
            guidance = "RECOGNIZED"
        with self._landmark_lock:
            self._progress_stage = progress_stage
            self._palm_center_norm = palm_center if valid_palm else None
            self._guidance = guidance
            recognized = self._recognized
        if triggered:
            self._armed = False
            self._last_trigger_at = now
            return True, "ONE PALM RECOGNIZED"
        if recognized:
            return False, "ONE PALM RECOGNIZED"
        if valid_palm:
            return False, f"ONE PALM {held_sec:.1f}/3.0 SEC"
        return False, guidance

    def draw_skeleton(self, frame):
        """Draw landmarks in-camera and return a frame with a separate status footer."""
        with self._landmark_lock:
            detected_hands = [list(hand) for hand in self._last_landmarks]
            progress_stage = self._progress_stage
            recognized = self._recognized
            palm_center_norm = self._palm_center_norm
            guidance = self._guidance
            hitbox = self._hitbox
        height, width = frame.shape[:2]
        if hitbox is not None:
            x1 = max(0, min(width - 1, int(round(hitbox[0] * width))))
            y1 = max(0, min(height - 1, int(round(hitbox[1] * height))))
            x2 = max(0, min(width - 1, int(round(hitbox[2] * width))))
            y2 = max(0, min(height - 1, int(round(hitbox[3] * height))))
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
            cv2.putText(
                frame,
                "PALM HITBOX",
                (x1, max(24, y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.58,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        status_color = (0, 255, 0) if recognized else (0, 210, 255)
        palm_pixel: tuple[int, int] | None = None
        if palm_center_norm is not None:
            palm_u = max(0, min(width - 1, int(round(palm_center_norm[0] * width))))
            palm_v = max(0, min(height - 1, int(round(palm_center_norm[1] * height))))
            palm_pixel = (palm_u, palm_v)
            cv2.drawMarker(
                frame,
                palm_pixel,
                status_color,
                cv2.MARKER_CROSS,
                22,
                2,
                cv2.LINE_AA,
            )
        for landmarks in detected_hands:
            if len(landmarks) != 21:
                continue
            points = [
                (
                    max(0, min(width - 1, int(round(x * width)))),
                    max(0, min(height - 1, int(round(y * height)))),
                )
                for x, y in landmarks
            ]
            for start, end in self._HAND_CONNECTIONS:
                cv2.line(frame, points[start], points[end], (255, 180, 0), 2, cv2.LINE_AA)
            for index, point in enumerate(points):
                color = (0, 255, 0) if index in {4, 8, 12, 16, 20} else (0, 140, 255)
                cv2.circle(frame, point, 4, color, -1, cv2.LINE_AA)

        # Preserve the calibrated 640x480 stream size. Fit the camera image
        # above a footer instead of increasing the frame height (the server
        # rejects non-calibrated dimensions).
        footer_height = min(82, max(60, height // 4))
        camera_area_height = height - footer_height
        camera_area_width = max(1, int(round(camera_area_height * width / height)))
        camera_area_width = min(width, camera_area_width)
        camera_view = cv2.resize(
            frame,
            (camera_area_width, camera_area_height),
            interpolation=cv2.INTER_AREA,
        )
        output = frame.copy()
        output[:] = (10, 12, 15)
        camera_x = (width - camera_area_width) // 2
        output[:camera_area_height, camera_x:camera_x + camera_area_width] = camera_view
        output[camera_area_height:, :] = (22, 25, 29)
        footer_y = camera_area_height
        for index in range(3):
            color = (0, 255, 0) if index < progress_stage else (90, 90, 90)
            center = (28 + index * 30, footer_y + 25)
            cv2.circle(output, center, 9, color, -1, cv2.LINE_AA)
            cv2.circle(output, center, 10, (230, 230, 230), 1, cv2.LINE_AA)
        cv2.putText(
            output,
            guidance,
            (126, footer_y + 31),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            status_color,
            2,
            cv2.LINE_AA,
        )
        coordinate_text = (
            f"PALM u={palm_pixel[0]} v={palm_pixel[1]}"
            if palm_pixel is not None
            else "PALM u=--- v=---"
        )
        cv2.putText(
            output,
            coordinate_text,
            (16, footer_y + 63),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (210, 215, 220),
            1,
            cv2.LINE_AA,
        )
        return output

    def clear_skeleton(self) -> None:
        with self._landmark_lock:
            self._last_landmarks = []
            self._progress_stage = 0
            self._recognized = False
            self._open_started_at = None
            self._palm_center_norm = None
            self._guidance = "SHOW ONE OPEN PALM"

    def get_palm_center_norm(self) -> tuple[float, float] | None:
        """Return the latest recognized/observed palm center."""
        with self._landmark_lock:
            return self._palm_center_norm

    def close(self) -> None:
        self._hands.close()
