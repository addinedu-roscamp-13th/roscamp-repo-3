"""Raspberry Pi execution entry point for the laptop-local YOLO pick/place service.

Keys:
  g: capture a fresh 640x480 frame + current Flange pose -> laptop plan -> validate -> pick
  p: print current Flange pose
  q: toggle gripper close/open
  r: move to a safe random pose around home
  s / servo-release: release all servos so the arm can be moved by hand
  k / servo-focus: focus/enable all servos
  f / place: place the held object: home -> last picked pose -> open gripper
  m / move: move to configured manual Flange pose
  t: run the existing throw motion after a successful pick
  space / stop: immediately stop current motion
  w: stop current motion and return home
  x: request stop and exit

The laptop performs YOLO and 2D->3D pick planning. This Pi remains responsible
for camera capture, MyCobot/gripper control, and the final local safety gate.
"""
from __future__ import annotations

import json
import math
import os
import random
import threading
import time
from queue import Empty, Queue
from typing import Any, Callable

import cv2

from . import config
from .api_client import (
    WaSaBServiceError,
    check_wasab_service_health,
    request_wasab_marker_place_plan,
    request_wasab_operation_plan,
    send_udp_streamer_frame,
    upload_streamer_frame,
    stream_wasab_arm_commands,
)
from .robot_controller import JOINT_LIMITS_DEG, WaSaBArmController


def _is_in_range(value: float, limits: tuple[float, float]) -> bool:
    return limits[0] <= value <= limits[1]


def validate_server_plan(
    payload: dict[str, Any],
) -> tuple[bool, str, list[float] | None]:
    """Run the final Pi-side safety validation before any robot command."""
    if payload.get("status") != "ok":
        return (
            False,
            str(payload.get("message", "target not found")),
            None,
        )

    plan = payload.get("plan")
    if not isinstance(plan, dict):
        return False, "response.plan is missing", None

    command = plan.get("flange_command")
    if not isinstance(command, list) or len(command) != 6:
        return False, "flange_command must contain six values", None

    try:
        command = [float(v) for v in command]
    except (TypeError, ValueError):
        return False, "flange_command contains non-numeric values", None

    if not all(math.isfinite(v) for v in command):
        return False, "flange_command contains non-finite values", None

    if not _is_in_range(command[0], config.SAFE_X_MM):
        return False, f"unsafe X={command[0]:.1f} mm", None
    if not _is_in_range(command[1], config.SAFE_Y_MM):
        return False, f"unsafe Y={command[1]:.1f} mm", None
    if not _is_in_range(command[2], config.SAFE_Z_MM):
        return False, f"unsafe Z={command[2]:.1f} mm", None
    if any(abs(v) > config.SAFE_EULER_ABS_DEG for v in command[3:]):
        return False, "unsafe Euler value", None

    return True, "ok", command



def _clamp(value: float, limits: tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def make_safe_random_flange_coords() -> list[float]:
    """Generate a conservative random flange pose near HOME and inside safety limits."""
    home = [float(value) for value in config.HOME_FLANGE_COORDS]
    target = [
        _clamp(
            home[0] + random.uniform(-config.RANDOM_X_RADIUS_MM, config.RANDOM_X_RADIUS_MM),
            config.SAFE_X_MM,
        ),
        _clamp(
            home[1] + random.uniform(-config.RANDOM_Y_RADIUS_MM, config.RANDOM_Y_RADIUS_MM),
            config.SAFE_Y_MM,
        ),
        _clamp(
            home[2] + random.uniform(-config.RANDOM_Z_RADIUS_MM, config.RANDOM_Z_RADIUS_MM),
            config.SAFE_Z_MM,
        ),
        home[3],
        home[4],
        home[5],
    ]
    return [round(value, 2) for value in target]

CONDUCT_2_KEY = 1002
CONDUCT_3_KEY = 1003
CONDUCT_4_KEY = 1004
STOP_KEY = 1005


REMOTE_COMMAND_TO_KEY = {
    "g": ord("g"),
    "pick": ord("g"),
    "p": ord("p"),
    "pose": ord("p"),
    "q": ord("q"),
    "gripper": ord("q"),
    "r": ord("r"),
    "random": ord("r"),
    "s": ord("s"),
    "servo-release": ord("s"),
    "k": ord("k"),
    "servo-focus": ord("k"),
    "f": ord("f"),
    "place": ord("f"),
    "m": ord("m"),
    "move": ord("m"),
    "a": ord("a"),
    "find-marker": ord("a"),
    "marker": ord("a"),
    "marker-search": ord("a"),
    "t": ord("t"),
    "throw": ord("t"),
    "w": ord("w"),
    "home": ord("w"),
    "stop": STOP_KEY,
    "halt": STOP_KEY,
    "emergency-stop": STOP_KEY,
    "emergency_stop": STOP_KEY,
    "x": ord("x"),
    "exit": ord("x"),
}


def remote_command_to_key(command: str) -> int | None:
    return REMOTE_COMMAND_TO_KEY.get(command.lower().strip())


def make_conducting_flange_sequence(beat_count: int) -> list[list[float]]:
    """Build a conducting pattern from configured reference poses.

    distance_scale lets the operator start with small motions and gradually move
    closer to the recorded reference poses without editing every coordinate.
    """
    home = [float(value) for value in config.HOME_FLANGE_COORDS]
    scale = float(config.CONDUCTING_DISTANCE_SCALE)

    def scaled_pose(reference: list[float]) -> list[float]:
        return [
            round(home[index] + (float(reference[index]) - home[index]) * scale, 2)
            for index in range(6)
        ]

    start = scaled_pose(config.CONDUCTING_START_FLANGE_COORDS)
    down = scaled_pose(config.CONDUCTING_DOWN_FLANGE_COORDS)
    up = scaled_pose(config.CONDUCTING_UP_FLANGE_COORDS)
    left = scaled_pose(config.CONDUCTING_LEFT_FLANGE_COORDS)
    right = scaled_pose(config.CONDUCTING_RIGHT_FLANGE_COORDS)

    if beat_count == 2:
        pattern = [down, up]
    elif beat_count == 3:
        pattern = [down, right, left]
    elif beat_count == 4:
        pattern = [down, left, right, up]
    else:
        raise ValueError(f"unsupported conducting beat count: {beat_count}")

    sequence = [start]
    for _ in range(config.CONDUCTING_CYCLES):
        sequence.extend(pattern)
    sequence.append(start)
    return sequence


def validate_conducting_sequence(sequence: list[list[float]]) -> tuple[bool, str, list[list[float]]]:
    safe_sequence: list[list[float]] = []
    for index, command in enumerate(sequence, start=1):
        pseudo_payload = {"status": "ok", "plan": {"flange_command": command}}
        is_safe, reason, safe_command = validate_server_plan(pseudo_payload)
        if not is_safe or safe_command is None:
            return False, f"point {index}: {reason}", []
        safe_sequence.append(safe_command)
    return True, "ok", safe_sequence


def interpolate_flange_sequence(
    sequence: list[list[float]],
    steps_per_segment: int,
) -> list[list[float]]:
    if len(sequence) < 2:
        return sequence

    steps = max(1, int(steps_per_segment))
    interpolated: list[list[float]] = [sequence[0]]
    for start, end in zip(sequence, sequence[1:]):
        for step in range(1, steps + 1):
            ratio = step / steps
            interpolated.append([
                round(float(start[index]) + (float(end[index]) - float(start[index])) * ratio, 2)
                for index in range(6)
            ])
    return interpolated


def _conducting_beat_targets(beat_count: int) -> list[tuple[float, float]]:
    if beat_count == 2:
        return [(0.0, 1.0), (0.0, -0.7)]
    if beat_count == 3:
        return [(0.0, 1.0), (0.9, -0.15), (-0.9, -0.15)]
    if beat_count == 4:
        return [(0.0, 1.0), (-0.9, 0.05), (0.9, 0.05), (0.0, -0.75)]
    raise ValueError(f"unsupported conducting beat count: {beat_count}")


def _conducting_wait(period_sec: float, abort_event: threading.Event | None = None) -> bool:
    deadline = time.monotonic() + period_sec
    while time.monotonic() < deadline:
        if abort_event is not None and abort_event.is_set():
            return False
        time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))
    return True


def _execute_conducting_joint_rhythm(
    wasab_arm_controller: WaSaBArmController,
    beat_count: int,
    abort_event: threading.Event | None = None,
) -> bool:
    base = wasab_arm_controller.get_joint_angles()
    current = list(base)
    targets = _conducting_beat_targets(beat_count)
    rate_hz = float(config.CONDUCTING_RATE_HZ)
    period_sec = 1.0 / rate_hz
    beat_steps = max(1, int(round(float(config.CONDUCTING_BEAT_SEC) * rate_hz)))
    return_steps = max(1, int(round(float(config.CONDUCTING_RETURN_SEC) * rate_hz)))
    scale = float(config.CONDUCTING_DISTANCE_SCALE)
    gain = float(config.CONDUCTING_SERVO_GAIN)
    yaw_amp = float(config.CONDUCTING_YAW_AMPLITUDE_DEG) * scale
    pitch_amp = float(config.CONDUCTING_PITCH_AMPLITUDE_DEG) * scale
    wrist_amp = float(config.CONDUCTING_WRIST_AMPLITUDE_DEG) * scale

    print(
        f"[CONDUCT] joint_rhythm beat={beat_count}, cycles={config.CONDUCTING_CYCLES}, "
        f"rate={rate_hz:.1f}Hz, beat={config.CONDUCTING_BEAT_SEC:.2f}s, "
        f"scale={scale:.2f}"
    )

    def stream_toward(goal: list[float], steps: int, label: str) -> bool:
        nonlocal current
        for step in range(steps):
            if abort_event is not None and abort_event.is_set():
                wasab_arm_controller.stop_motion()
                return False
            phase = (step + 1) / steps
            pulse = math.sin(math.pi * phase)
            commanded = list(current)
            for joint_index, target_angle in enumerate(goal):
                commanded[joint_index] += (target_angle - commanded[joint_index]) * gain

            commanded[5] = base[5] + (goal[5] - base[5]) * pulse
            current = commanded
            print(f"[CONDUCT] {label} {step + 1}/{steps}:", [round(v, 2) for v in commanded])
            wasab_arm_controller.send_joint_angles(
                commanded,
                speed=config.CONDUCTING_MOVE_SPEED,
                async_command=True,
            )
            if step + 1 < steps:
                if not _conducting_wait(period_sec, abort_event):
                    wasab_arm_controller.stop_motion()
                    return False
        return True

    for cycle in range(int(config.CONDUCTING_CYCLES)):
        for beat_index, (yaw_unit, pitch_unit) in enumerate(targets, start=1):
            goal = list(base)
            goal[0] = base[0] + yaw_unit * yaw_amp
            goal[3] = base[3] + pitch_unit * pitch_amp
            goal[5] = base[5] + (-yaw_unit * 0.5 + pitch_unit * 0.25) * wrist_amp
            if not stream_toward(goal, beat_steps, f"cycle {cycle + 1} beat {beat_index}"):
                return False

    if not stream_toward(base, return_steps, "return"):
        return False
    wasab_arm_controller.send_joint_angles(
        base,
        speed=config.CONDUCTING_MOVE_SPEED,
        async_command=True,
    )
    return wasab_arm_controller.wait_until_joint_angles(
        base,
        timeout_sec=max(2.0, float(config.CONDUCTING_RETURN_SEC) + 1.0),
        tolerance_deg=3.0,
        abort_event=abort_event,
    )


def _execute_conducting_flange_sequence(
    wasab_arm_controller: WaSaBArmController,
    safe_sequence: list[list[float]],
    abort_event: threading.Event | None = None,
) -> bool:
    if not safe_sequence:
        return False

    if not config.CONDUCTING_CONTINUOUS:
        total_points = len(safe_sequence)
        for index, command in enumerate(safe_sequence, start=1):
            print(f"[CONDUCT] point {index}/{total_points}:", command)
            if not wasab_arm_controller.send_flange_coords_and_wait(
                command,
                speed=config.CONDUCTING_MOVE_SPEED,
                abort_event=abort_event,
            ):
                return False
        return True

    smooth_sequence = interpolate_flange_sequence(
        safe_sequence,
        config.CONDUCTING_INTERPOLATION_STEPS,
    )
    total_points = len(smooth_sequence)
    print(
        f"[CONDUCT] flange continuous points={total_points}, "
        f"steps={config.CONDUCTING_INTERPOLATION_STEPS}, "
        f"interval={config.CONDUCTING_COMMAND_INTERVAL_SEC:.3f}s"
    )
    for index, command in enumerate(smooth_sequence, start=1):
        if abort_event is not None and abort_event.is_set():
            wasab_arm_controller.stop_motion()
            return False
        print(f"[CONDUCT] stream {index}/{total_points}:", command)
        wasab_arm_controller.send_flange_coords(
            command,
            speed=config.CONDUCTING_MOVE_SPEED,
        )
        if index < total_points:
            if not _conducting_wait(config.CONDUCTING_COMMAND_INTERVAL_SEC, abort_event):
                wasab_arm_controller.stop_motion()
                return False

    return wasab_arm_controller.wait_until_flange_pose(smooth_sequence[-1], abort_event=abort_event)


def execute_conducting_sequence(
    wasab_arm_controller: WaSaBArmController,
    beat_count: int,
    safe_sequence: list[list[float]],
    abort_event: threading.Event | None = None,
) -> bool:
    if config.CONDUCTING_CONTROL_MODE == "joint_rhythm":
        return _execute_conducting_joint_rhythm(wasab_arm_controller, beat_count, abort_event)
    return _execute_conducting_flange_sequence(wasab_arm_controller, safe_sequence, abort_event)


def validate_frame_size(frame) -> None:
    """Reject frames that do not match the laptop's camera-calibration resolution."""
    if frame is None or getattr(frame, "ndim", 0) < 2:
        raise RuntimeError("Camera frame is invalid")

    height, width = frame.shape[:2]
    if (
        width != config.CAMERA_FRAME_WIDTH
        or height != config.CAMERA_FRAME_HEIGHT
    ):
        raise RuntimeError(
            "Camera frame size mismatch: "
            f"got {width}x{height}, expected "
            f"{config.CAMERA_FRAME_WIDTH}x{config.CAMERA_FRAME_HEIGHT}. "
            "The laptop calibration is valid only at the configured resolution."
        )


def open_calibrated_camera():
    """Open the Pi camera and make resolution mismatch fail before robot startup."""
    cap = cv2.VideoCapture(config.CAMERA_ID, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(
            f"Cannot open camera: CAMERA_ID={config.CAMERA_ID!r}. "
            "Check the real capture device with `v4l2-ctl --list-devices` or "
            "`ls -l /dev/video*`, then set [camera] camera_id to that index "
            "or path, for example /dev/video2."
        )

    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_FRAME_HEIGHT)

    # Let USB/CSI cameras apply their requested capture format before validation.
    for _ in range(max(2, config.CAMERA_FLUSH_FRAMES)):
        cap.grab()

    ret, probe_frame = cap.read()
    if not ret or probe_frame is None:
        with cap_lock:
            cap.release()
        raise RuntimeError("Cannot read initial camera frame")

    try:
        validate_frame_size(probe_frame)
    except Exception:
        with cap_lock:
            cap.release()
        raise

    print(
        "[CAMERA] calibrated capture size: "
        f"{config.CAMERA_FRAME_WIDTH}x{config.CAMERA_FRAME_HEIGHT}"
    )
    return cap


def capture_fresh_plan_frame(cap):
    """Capture a recent frame when pick is requested, reducing buffered-camera latency."""
    for _ in range(config.CAMERA_FLUSH_FRAMES):
        cap.grab()

    ret, frame = cap.read()
    if not ret or frame is None:
        raise RuntimeError("Cannot capture a current camera frame for pick planning")

    validate_frame_size(frame)
    return frame


def _create_marker_detector():
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("OpenCV aruco module is unavailable. Install opencv-contrib-python.")

    dictionary_id = getattr(cv2.aruco, config.MARKER_SEARCH_DICTIONARY, None)
    if dictionary_id is None:
        raise RuntimeError(f"Unknown marker dictionary: {config.MARKER_SEARCH_DICTIONARY}")

    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(cv2.aruco, "DetectorParameters"):
        parameters = cv2.aruco.DetectorParameters()
    else:
        parameters = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        return cv2.aruco.ArucoDetector(dictionary, parameters), None, None
    return None, dictionary, parameters


def detect_april_marker(frame) -> dict[str, Any] | None:
    detector, dictionary, parameters = _create_marker_detector()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if detector is not None:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is None or len(ids) == 0:
        return None

    ids_flat = ids.flatten().astype(int).tolist()
    selected_indices = [
        index for index, marker_id in enumerate(ids_flat)
        if not config.MARKER_SEARCH_TARGET_IDS or marker_id in config.MARKER_SEARCH_TARGET_IDS
    ]
    if not selected_indices:
        return None

    selected = selected_indices[0]
    points = corners[selected].reshape(-1, 2).astype(float)
    center = points.mean(axis=0)
    x_min, y_min = points.min(axis=0)
    x_max, y_max = points.max(axis=0)
    return {
        "id": ids_flat[selected],
        "ids": ids_flat,
        "center": [float(center[0]), float(center[1])],
        "corners": [[float(x), float(y)] for x, y in points],
        "bbox": [float(x_min), float(y_min), float(x_max), float(y_max)],
        "corner_count": int(len(points)),
    }


def execute_marker_search(
    wasab_arm_controller: WaSaBArmController,
    get_frame: Callable[..., tuple[object | None, int]],
    status_callback: Callable[[str], None],
    abort_event: threading.Event | None = None,
) -> tuple[bool, str, dict[str, Any] | None]:
    base_angles = wasab_arm_controller.get_joint_angles()
    pan_index = config.MARKER_SEARCH_PAN_JOINT - 1
    joint_limits = JOINT_LIMITS_DEG[pan_index]
    pan_low = max(joint_limits[0], base_angles[pan_index] - config.MARKER_SEARCH_PAN_RANGE_DEG)
    pan_high = min(joint_limits[1], base_angles[pan_index] + config.MARKER_SEARCH_PAN_RANGE_DEG)
    target_angle = _clamp(base_angles[pan_index], (pan_low, pan_high))
    search_dir = 1.0
    period_sec = 1.0 / config.MARKER_SEARCH_HZ
    started_at = time.monotonic()
    deadline = None
    if config.MARKER_SEARCH_MAX_DURATION_SEC > 0:
        deadline = started_at + config.MARKER_SEARCH_MAX_DURATION_SEC

    status_callback(
        f"Marker search started: J{config.MARKER_SEARCH_PAN_JOINT} "
        f"{pan_low:.1f}..{pan_high:.1f} deg"
    )
    print(
        "[MARKER] search started:",
        f"dictionary={config.MARKER_SEARCH_DICTIONARY}",
        f"target_ids={sorted(config.MARKER_SEARCH_TARGET_IDS) or 'any'}",
        f"J{config.MARKER_SEARCH_PAN_JOINT}={pan_low:.1f}..{pan_high:.1f}",
    )

    while True:
        if abort_event is not None and abort_event.is_set():
            wasab_arm_controller.stop_motion()
            return False, "Marker search stopped", None
        if deadline is not None and time.monotonic() >= deadline:
            return False, "Marker search timeout", None

        frame, _ = get_frame(timeout_sec=period_sec)
        if frame is not None:
            detection = detect_april_marker(frame)
            if detection is not None:
                message = (
                    f"Marker found: id={detection['id']} "
                    f"center=({detection['center'][0]:.0f}, {detection['center'][1]:.0f})"
                )
                print("[MARKER]", message, "all_ids=", detection["ids"])
                wasab_arm_controller.stop_motion()
                return True, message, detection

        target_angle += config.MARKER_SEARCH_STEP_DEG * search_dir
        if target_angle >= pan_high or target_angle <= pan_low:
            target_angle = _clamp(target_angle, (pan_low, pan_high))
            search_dir *= -1.0

        wasab_arm_controller.send_joint_angle(
            config.MARKER_SEARCH_PAN_JOINT,
            target_angle,
            config.MARKER_SEARCH_SPEED,
        )
        status_callback(f"Searching marker... J{config.MARKER_SEARCH_PAN_JOINT}={target_angle:.1f}")
        time.sleep(period_sec)


def draw_result(
    frame,
    payload: dict[str, Any] | None,
    error: str | None,
    throw_running: bool,
    marker_detection: dict[str, Any] | None = None,
) -> None:
    text_y = 32

    if config.DRY_RUN:
        cv2.putText(
            frame,
            "DRY RUN: robot motion disabled",
            (18, text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (0, 165, 255),
            2,
        )
        text_y += 32

    if payload and payload.get("status") == "ok":
        det = payload.get("detection", {})
        bbox = det.get("bbox")
        midpoint = det.get("midpoint_uv")

        if isinstance(bbox, list) and len(bbox) == 4:
            x1, box_y1, x2, box_y2 = (
                int(round(float(v))) for v in bbox
            )
            cv2.rectangle(
                frame, (x1, box_y1), (x2, box_y2), (0, 255, 0), 2
            )
            label = (
                f"{det.get('label', 'object')} "
                f"{float(det.get('confidence', 0.0)):.2f}"
            )
            cv2.putText(
                frame,
                label,
                (x1, max(24, box_y1 - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.60,
                (0, 255, 0),
                2,
            )

        if isinstance(midpoint, list) and len(midpoint) == 2:
            u, v = (int(round(float(value))) for value in midpoint)
            cv2.drawMarker(
                frame,
                (u, v),
                (0, 255, 0),
                cv2.MARKER_CROSS,
                20,
                2,
            )

        plan = payload.get("plan", {})
        tcp = plan.get("tcp_target_base_mm")
        if isinstance(tcp, list) and len(tcp) == 3:
            text = f"TCP Base: {tcp[0]:.1f}, {tcp[1]:.1f}, {tcp[2]:.1f} mm"
            cv2.putText(
                frame,
                text,
                (18, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 255),
                2,
            )
            text_y += 28

        command = plan.get("flange_command")
        if isinstance(command, list) and len(command) == 6:
            text = (
                f"Flange: {command[0]:.1f}, "
                f"{command[1]:.1f}, {command[2]:.1f} mm"
            )
            cv2.putText(
                frame,
                text,
                (18, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (255, 255, 0),
                2,
            )
            text_y += 28

    if marker_detection:
        marker_bbox = marker_detection.get("bbox")
        if isinstance(marker_bbox, list) and len(marker_bbox) == 4:
            x1, marker_y1, x2, marker_y2 = (
                int(round(float(value))) for value in marker_bbox
            )
            cv2.rectangle(frame, (x1, marker_y1), (x2, marker_y2), (0, 255, 0), 3)
            marker_label = f"Marker found id={marker_detection.get('id', '?')}"
            cv2.putText(
                frame,
                marker_label,
                (x1, max(24, marker_y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.62,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
        marker_center = marker_detection.get("center")
        if isinstance(marker_center, list) and len(marker_center) == 2:
            u, v = (int(round(float(value))) for value in marker_center)
            cv2.drawMarker(frame, (u, v), (0, 255, 0), cv2.MARKER_CROSS, 24, 2)


def main() -> None:
    print("=== Raspberry Pi -> laptop-local robot pick/place client ===")
    print("Laptop endpoint:", config.GRASP_SERVER_URL)
    print("Expected runtime:", config.EXPECTED_SERVER_RUNTIME)
    print("DRY_RUN:", config.DRY_RUN)
    print("Camera stream:", config.CAMERA_STREAM_ENABLED)
    print("Streamer transport:", "udp" if config.UDP_STREAM_ENABLED else "http")
    print("Remote control:", config.REMOTE_COMMAND_ENABLED)

    if config.CHECK_SERVER_ON_STARTUP:
        try:
            health = check_wasab_service_health()
            print(
                "[NETWORK] laptop server reachable: "
                f"runtime={health.get('runtime')}, "
                f"device={health.get('device')}, "
                f"model={health.get('model_path')}"
            )
        except WaSaBServiceError as exc:
            raise RuntimeError(
                "Laptop server preflight failed. Check the [network] "
                "grasp_server_url, laptop firewall, and shared LAN path before "
                f"starting robot control.\n{exc}"
            ) from exc

    cap = open_calibrated_camera()
    cap_lock = threading.Lock()
    wasab_arm_controller = WaSaBArmController()

    if config.DRY_RUN:
        print("[SAFETY] DRY_RUN=True: Startup home/open is skipped.")
    else:
        wasab_arm_controller.move_home_and_open_gripper()

    last_payload: dict[str, Any] | None = None
    last_payload_expires_at = 0.0
    last_marker_detection: dict[str, Any] | None = None
    last_marker_detection_expires_at = 0.0
    last_error: str | None = None
    gripper_closed_on_target = False
    last_pick_flange_command: list[float] | None = None
    state_lock = threading.Lock()
    frame_condition = threading.Condition()
    capture_stop = threading.Event()
    stream_stop = threading.Event()
    latest_frame = None
    latest_frame_seq = 0
    last_stream_error_at = 0.0
    last_remote_command_error_at = 0.0
    remote_command_queue: Queue[str] = Queue()
    remote_stop = threading.Event()
    stop_request = threading.Event()

    throw_running = threading.Event()
    throw_abort = threading.Event()
    throw_thread: threading.Thread | None = None
    stop_lock = threading.Lock()

    def clear_remote_command_queue() -> None:
        with remote_command_queue.mutex:
            remote_command_queue.queue.clear()

    def request_immediate_stop(source: str) -> None:
        nonlocal last_error
        with stop_lock:
            stop_request.set()
            throw_abort.set()
            clear_remote_command_queue()
            if not config.DRY_RUN:
                try:
                    wasab_arm_controller.stop_motion()
                except Exception as exc:
                    print(f"[STOP] stop error from {source}:", exc)
            with state_lock:
                last_error = f"STOP requested from {source}"
            print(f"[STOP] requested from {source}")

    def finish_stop_request() -> None:
        nonlocal gripper_closed_on_target
        if not stop_request.is_set():
            return
        if throw_thread is not None and throw_thread.is_alive():
            throw_thread.join(timeout=0.2)
        throw_running.clear()
        stop_request.clear()
        clear_remote_command_queue()
        with state_lock:
            last_error = "STOP complete"

    def run_throw_worker() -> None:
        nonlocal gripper_closed_on_target, last_error
        try:
            success, message, released = wasab_arm_controller.execute_throw_mode(
                abort_event=throw_abort
            )
            if released:
                gripper_closed_on_target = False
            if success:
                with state_lock:
                    last_error = None
                print("[THROW]", message)
            else:
                with state_lock:
                    last_error = f"THROW failed: {message}"
                print("[THROW]", last_error)
        except Exception as exc:
            with state_lock:
                last_error = f"THROW ERROR: {type(exc).__name__}: {exc}"
            print(last_error)
        finally:
            throw_running.clear()

    show_window = bool(config.SHOW_WINDOW)
    if show_window and not os.environ.get("DISPLAY"):
        show_window = False
        print("[UI] DISPLAY is not set; local OpenCV window disabled.")
    elif not show_window:
        print("[UI] Local OpenCV window disabled; use AdminGUI or remote commands.")

    if show_window:
        cv2.namedWindow(config.WINDOW_NAME, cv2.WINDOW_NORMAL)

    def streamer_capture_worker() -> None:
        nonlocal cap, latest_frame, latest_frame_seq, last_error, last_stream_error_at
        read_failures = 0
        while not capture_stop.is_set():
            with cap_lock:
                ret, frame = cap.read()
            now = time.monotonic()
            if not ret or frame is None:
                read_failures += 1
                if read_failures >= 30:
                    message = "camera read failed repeatedly; reopening camera"
                    with state_lock:
                        last_error = message
                    if now - last_stream_error_at >= 5.0:
                        print("[CAMERA]", message)
                        last_stream_error_at = now
                    try:
                        with cap_lock:
                            cap.release()
                            cap = open_calibrated_camera()
                        read_failures = 0
                    except Exception as exc:
                        with state_lock:
                            last_error = f"Camera reopen failed: {exc}"
                        time.sleep(1.0)
                else:
                    time.sleep(0.02)
                continue

            read_failures = 0
            try:
                validate_frame_size(frame)
            except RuntimeError as exc:
                with state_lock:
                    last_error = str(exc)
                if now - last_stream_error_at >= 5.0:
                    print("[CAMERA]", exc)
                    last_stream_error_at = now
                time.sleep(0.05)
                continue

            with frame_condition:
                latest_frame = frame
                latest_frame_seq += 1
                frame_condition.notify_all()

    def get_latest_frame(
        *,
        min_seq: int | None = None,
        timeout_sec: float = 0.5,
    ) -> tuple[object | None, int]:
        deadline = time.monotonic() + timeout_sec
        with frame_condition:
            while latest_frame is None or (min_seq is not None and latest_frame_seq <= min_seq):
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                frame_condition.wait(timeout=remaining)
            if latest_frame is None:
                return None, latest_frame_seq
            return latest_frame.copy(), latest_frame_seq

    capture_thread = threading.Thread(target=streamer_capture_worker, daemon=True)
    capture_thread.start()

    def snapshot_overlay_state() -> tuple[dict[str, Any] | None, str | None, bool, dict[str, Any] | None]:
        with state_lock:
            now = time.monotonic()
            payload = last_payload if now < last_payload_expires_at else None
            marker = last_marker_detection if now < last_marker_detection_expires_at else None
            return payload, last_error, throw_running.is_set(), marker

    def streamer_upload_worker() -> None:
        nonlocal last_stream_error_at
        next_frame_at = 0.0
        last_uploaded_seq = 0
        while not stream_stop.is_set():
            now = time.monotonic()
            if now < next_frame_at:
                time.sleep(min(0.01, next_frame_at - now))
                continue
            next_frame_at = now + config.CAMERA_STREAM_INTERVAL_SEC

            stream_frame, seq = get_latest_frame(
                min_seq=last_uploaded_seq,
                timeout_sec=config.CAMERA_STREAM_INTERVAL_SEC,
            )
            if stream_frame is None:
                continue
            last_uploaded_seq = seq

            payload_snapshot, error_snapshot, throw_snapshot, marker_snapshot = snapshot_overlay_state()
            draw_result(stream_frame, payload_snapshot, error_snapshot, throw_snapshot, marker_snapshot)
            try:
                if config.UDP_STREAM_ENABLED:
                    send_udp_streamer_frame(stream_frame)
                else:
                    upload_streamer_frame(stream_frame)
            except WaSaBServiceError as exc:
                if config.UDP_STREAM_ENABLED and config.UDP_STREAM_FALLBACK_HTTP:
                    try:
                        upload_streamer_frame(stream_frame)
                        continue
                    except WaSaBServiceError as fallback_exc:
                        exc = fallback_exc
                if now - last_stream_error_at >= 5.0:
                    print("[CAMERA STREAM]", exc)
                    last_stream_error_at = now

    stream_thread: threading.Thread | None = None
    if config.CAMERA_STREAM_ENABLED:
        stream_thread = threading.Thread(target=streamer_upload_worker, daemon=True)
        stream_thread.start()

    def remote_command_stream_worker() -> None:
        nonlocal last_remote_command_error_at
        while not remote_stop.is_set():
            try:
                for remote_command in stream_wasab_arm_commands(remote_stop):
                    if remote_stop.is_set():
                        break
                    mapped_key = remote_command_to_key(remote_command)
                    if mapped_key == STOP_KEY:
                        request_immediate_stop("remote")
                        continue
                    remote_command_queue.put(remote_command)
            except WaSaBServiceError as exc:
                now = time.monotonic()
                if now - last_remote_command_error_at >= 5.0:
                    print("[REMOTE COMMAND]", exc)
                    last_remote_command_error_at = now
                remote_stop.wait(1.0)

    remote_thread: threading.Thread | None = None
    if config.REMOTE_COMMAND_ENABLED:
        remote_thread = threading.Thread(target=remote_command_stream_worker, daemon=True)
        remote_thread.start()

    print(
        "[READY] Robot client is running; waiting for camera frames and commands "
        "(g/pick, a/find-marker, t/throw, space/stop, w/home, x/exit)."
    )
    last_main_frame_wait_log_at = 0.0

    try:
        while True:
            frame, frame_seq = get_latest_frame(timeout_sec=0.5)
            if frame is None:
                now = time.monotonic()
                if now - last_main_frame_wait_log_at >= 5.0:
                    print("[WAIT] No camera frame yet; check camera_id, cable, and frame size settings.")
                    last_main_frame_wait_log_at = now
                time.sleep(0.05)
                continue

            payload_snapshot, error_snapshot, throw_snapshot, marker_snapshot = snapshot_overlay_state()
            annotated = frame.copy()
            draw_result(
                annotated,
                payload_snapshot,
                error_snapshot,
                throw_snapshot,
                marker_snapshot,
            )
            if show_window:
                cv2.imshow(config.WINDOW_NAME, annotated)
                key = cv2.waitKey(1) & 0xFF
            else:
                key = 255

            if config.REMOTE_COMMAND_ENABLED:
                try:
                    remote_command = remote_command_queue.get_nowait()
                except Empty:
                    pass
                else:
                    mapped_key = remote_command_to_key(remote_command)
                    if mapped_key is None:
                        with state_lock:
                            last_error = f"Remote command ignored: {remote_command}"
                        continue
                    key = mapped_key

            if key in {STOP_KEY, ord(" ")}:
                request_immediate_stop("local" if key == ord(" ") else "remote")
                finish_stop_request()
                continue

            if stop_request.is_set():
                finish_stop_request()
                continue

            if key == ord("p"):
                try:
                    coords = wasab_arm_controller.get_flange_coords()
                    text = (
                        "POSE Flange: "
                        f"x={coords[0]:.1f}, y={coords[1]:.1f}, z={coords[2]:.1f}, "
                        f"rx={coords[3]:.2f}, ry={coords[4]:.2f}, rz={coords[5]:.2f}"
                    )
                    print("[POSE]", coords)
                    with state_lock:
                        last_error = text
                except Exception as exc:
                    with state_lock:
                        last_error = f"POSE ERROR: {type(exc).__name__}: {exc}"
                    print(last_error)
                continue

            if key == ord("q"):
                if throw_running.is_set():
                    with state_lock:
                        last_error = "Gripper ignored: throw mode is running"
                elif config.DRY_RUN:
                    with state_lock:
                        last_error = "DRY RUN: gripper command not sent"
                else:
                    try:
                        if gripper_closed_on_target:
                            wasab_arm_controller.open_gripper()
                            gripper_closed_on_target = False
                            with state_lock:
                                last_error = "Gripper opened"
                        else:
                            wasab_arm_controller.close_gripper()
                            gripper_closed_on_target = True
                            with state_lock:
                                last_error = "Gripper closed"
                    except RuntimeError as exc:
                        with state_lock:
                            last_error = f"GRIPPER ERROR: {exc}"
                        print(last_error)
                continue

            if key == ord("x"):
                request_immediate_stop("exit")
                if throw_running.is_set():
                    print("[THROW] abort requested")
                    throw_abort.set()
                    if not config.DRY_RUN:
                        try:
                            wasab_arm_controller.stop_motion()
                        except Exception as exc:
                            print("[THROW] stop error:", exc)
                break

            if key == ord("s"):
                if throw_running.is_set():
                    print("[SERVO] aborting throw before servo release")
                    throw_abort.set()
                    if not config.DRY_RUN:
                        try:
                            wasab_arm_controller.stop_motion()
                        except Exception as exc:
                            print("[SERVO] stop error:", exc)
                    if throw_thread is not None and throw_thread.is_alive():
                        throw_thread.join(timeout=1.0)
                    throw_running.clear()

                if config.DRY_RUN:
                    with state_lock:
                        last_error = "DRY RUN: servo release not sent"
                else:
                    try:
                        wasab_arm_controller.release_all_servos()
                        with state_lock:
                            last_error = "Servos released"
                    except Exception as exc:
                        with state_lock:
                            last_error = f"SERVO RELEASE ERROR: {type(exc).__name__}: {exc}"
                        print(last_error)
                continue

            if key == ord("k"):
                if throw_running.is_set():
                    with state_lock:
                        last_error = "Servo focus ignored: throw mode is running"
                elif config.DRY_RUN:
                    with state_lock:
                        last_error = "DRY RUN: servo focus not sent"
                else:
                    try:
                        wasab_arm_controller.focus_all_servos()
                        with state_lock:
                            last_error = "Servos focused"
                    except Exception as exc:
                        with state_lock:
                            last_error = f"SERVO FOCUS ERROR: {type(exc).__name__}: {exc}"
                        print(last_error)
                continue

            if key == ord("f"):
                if throw_running.is_set():
                    with state_lock:
                        last_error = "Place ignored: throw mode is running"
                elif not gripper_closed_on_target:
                    with state_lock:
                        last_error = "Place ignored: pick an object first"
                elif not config.PLACE_MOTION_ENABLED:
                    with state_lock:
                        last_error = "Place ignored: place motion is disabled"
                elif not config.MARKER_SEARCH_ENABLED:
                    with state_lock:
                        last_error = "Place ignored: marker search is disabled"
                elif config.DRY_RUN:
                    frame_snapshot, _ = get_latest_frame(timeout_sec=0.5)
                    detection = detect_april_marker(frame_snapshot) if frame_snapshot is not None else None
                    with state_lock:
                        last_error = (
                            f"DRY RUN: place marker visible id={detection['id']}"
                            if detection is not None
                            else "DRY RUN: place marker not visible"
                        )
                        if detection is not None:
                            last_marker_detection = detection
                            last_marker_detection_expires_at = time.monotonic() + 5.0
                    print("[PLACE]", last_error)
                else:
                    def set_place_marker_status(message: str) -> None:
                        nonlocal last_error
                        with state_lock:
                            last_error = message

                    try:
                        print("[PLACE] home -> find marker -> marker place -> open gripper")
                        home_reached = wasab_arm_controller.move_home_keep_gripper_closed(abort_event=stop_request)
                        if not home_reached:
                            with state_lock:
                                last_error = "Place aborted: HOME return timeout"
                            continue

                        success, message, marker_detection = execute_marker_search(
                            wasab_arm_controller,
                            get_latest_frame,
                            set_place_marker_status,
                            stop_request,
                        )
                        with state_lock:
                            last_error = message
                            if marker_detection is not None:
                                last_marker_detection = marker_detection
                                last_marker_detection_expires_at = time.monotonic() + 5.0
                        if not success or marker_detection is None:
                            print("[PLACE]", message)
                            continue

                        current_flange_coords = wasab_arm_controller.get_flange_coords()
                        payload = request_wasab_marker_place_plan(
                            marker_detection,
                            current_flange_coords,
                        )
                        print(
                            "[MARKER PLACE RESPONSE]\n",
                            json.dumps(payload, ensure_ascii=False, indent=2),
                        )
                        with state_lock:
                            last_payload = payload
                            last_payload_expires_at = time.monotonic() + 3.0

                        is_safe, reason, safe_command = validate_server_plan(payload)
                        if not is_safe or safe_command is None:
                            with state_lock:
                                last_error = f"Marker place pose rejected locally: {reason}"
                            print("[SAFETY]", last_error)
                            continue

                        place_reached = wasab_arm_controller.send_flange_coords_and_wait(
                            safe_command,
                            abort_event=stop_request,
                        )
                        if not place_reached:
                            with state_lock:
                                last_error = "Marker place pose timeout; gripper remains closed"
                            continue

                        wasab_arm_controller.open_gripper()
                        gripper_closed_on_target = False
                        last_pick_flange_command = None
                        with state_lock:
                            last_error = "Place complete at AprilTag"
                    except Exception as exc:
                        with state_lock:
                            last_error = f"PLACE ERROR: {type(exc).__name__}: {exc}"
                        print(last_error)
                continue

            if key == ord("m"):
                if throw_running.is_set():
                    with state_lock:
                        last_error = "M ignored: throw mode is running"
                elif not config.MANUAL_MOTION_ENABLED:
                    with state_lock:
                        last_error = "M ignored: manual motion is disabled"
                elif config.DRY_RUN:
                    with state_lock:
                        last_error = f"DRY RUN: manual command not sent {config.MANUAL_FLANGE_COORDS}"
                    print("[DRY RUN] Manual flange command:", config.MANUAL_FLANGE_COORDS)
                else:
                    pseudo_payload = {"status": "ok", "plan": {"flange_command": config.MANUAL_FLANGE_COORDS}}
                    is_safe, reason, safe_command = validate_server_plan(pseudo_payload)
                    if not is_safe or safe_command is None:
                        with state_lock:
                            last_error = f"Manual pose rejected locally: {reason}"
                        print("[SAFETY]", last_error)
                    else:
                        print("[MANUAL] configured flange command:", safe_command)
                        try:
                            reached = wasab_arm_controller.send_flange_coords_and_wait(safe_command, abort_event=stop_request)
                        except Exception as exc:
                            with state_lock:
                                last_error = f"MANUAL MOVE ERROR: {type(exc).__name__}: {exc}"
                            print(last_error)
                        else:
                            with state_lock:
                                last_error = None if reached else "Manual pose timeout"
                continue

            if key in {CONDUCT_2_KEY, CONDUCT_3_KEY, CONDUCT_4_KEY, ord("2"), ord("3"), ord("4")}:
                beat_count = {
                    CONDUCT_2_KEY: 2,
                    CONDUCT_3_KEY: 3,
                    CONDUCT_4_KEY: 4,
                    ord("2"): 2,
                    ord("3"): 3,
                    ord("4"): 4,
                }[key]
                if throw_running.is_set():
                    with state_lock:
                        last_error = f"Conduct {beat_count} ignored: throw mode is running"
                elif not config.CONDUCTING_ENABLED:
                    with state_lock:
                        last_error = "Conducting ignored: conducting motion is disabled"
                else:
                    if config.CONDUCTING_CONTROL_MODE == "flange":
                        sequence = make_conducting_flange_sequence(beat_count)
                        is_safe, reason, safe_sequence = validate_conducting_sequence(sequence)
                    else:
                        is_safe, reason, safe_sequence = True, "ok", []

                    if not is_safe:
                        with state_lock:
                            last_error = f"Conducting rejected locally: {reason}"
                        print("[SAFETY]", last_error)
                    elif config.DRY_RUN:
                        with state_lock:
                            last_error = f"DRY RUN: conduct-{beat_count} command not sent"
                        print(
                            f"[DRY RUN] Conduct {beat_count} "
                            f"mode={config.CONDUCTING_CONTROL_MODE} sequence:",
                            safe_sequence,
                        )
                    else:
                        print(
                            f"[CONDUCT] beat={beat_count}, "
                            f"mode={config.CONDUCTING_CONTROL_MODE}, "
                            f"scale={config.CONDUCTING_DISTANCE_SCALE:.2f}, "
                            f"speed={config.CONDUCTING_MOVE_SPEED}, "
                            f"continuous={config.CONDUCTING_CONTINUOUS}, "
                            f"cycles={config.CONDUCTING_CYCLES}"
                        )
                        try:
                            success = execute_conducting_sequence(
                                wasab_arm_controller,
                                beat_count,
                                safe_sequence,
                                stop_request,
                            )
                            with state_lock:
                                last_error = None if success else f"Conduct {beat_count} pose timeout"
                        except Exception as exc:
                            with state_lock:
                                last_error = f"CONDUCT ERROR: {type(exc).__name__}: {exc}"
                            print(last_error)
                continue

            if key == ord("a"):
                if throw_running.is_set():
                    with state_lock:
                        last_error = "Find marker ignored: throw mode is running"
                elif not config.MARKER_SEARCH_ENABLED:
                    with state_lock:
                        last_error = "Find marker ignored: marker search is disabled"
                elif config.DRY_RUN:
                    frame_snapshot, _ = get_latest_frame(timeout_sec=0.5)
                    detection = detect_april_marker(frame_snapshot) if frame_snapshot is not None else None
                    with state_lock:
                        last_error = (
                            f"DRY RUN: marker visible id={detection['id']}"
                            if detection is not None
                            else "DRY RUN: marker not visible; scan command not sent"
                        )
                        if detection is not None:
                            last_marker_detection = detection
                            last_marker_detection_expires_at = time.monotonic() + 5.0
                    print("[MARKER]", last_error)
                else:
                    def set_marker_status(message: str) -> None:
                        nonlocal last_error
                        with state_lock:
                            last_error = message

                    try:
                        success, message, marker_detection = execute_marker_search(
                            wasab_arm_controller,
                            get_latest_frame,
                            set_marker_status,
                            stop_request,
                        )
                        with state_lock:
                            last_error = message
                            if marker_detection is not None:
                                last_marker_detection = marker_detection
                                last_marker_detection_expires_at = time.monotonic() + 5.0
                        if not success:
                            print("[MARKER]", message)
                    except Exception as exc:
                        with state_lock:
                            last_error = f"MARKER SEARCH ERROR: {type(exc).__name__}: {exc}"
                        print(last_error)
                continue

            if key == ord("t"):
                if config.DRY_RUN:
                    with state_lock:
                        last_error = "DRY RUN: Throw command not sent"
                elif throw_running.is_set():
                    with state_lock:
                        last_error = "Throw ignored: throw mode is already running"
                elif not gripper_closed_on_target:
                    with state_lock:
                        last_error = "Throw ignored: pick an object first"
                else:
                    throw_abort.clear()
                    throw_running.set()
                    throw_thread = threading.Thread(
                        target=run_throw_worker,
                        daemon=False,
                    )
                    throw_thread.start()
                    with state_lock:
                        last_error = None
                    print("[THROW] mode started")
                continue


            if key == ord("r"):
                if throw_running.is_set():
                    with state_lock:
                        last_error = "R ignored: throw mode is running"
                elif not config.RANDOM_MOTION_ENABLED:
                    with state_lock:
                        last_error = "R ignored: random motion is disabled"
                elif config.DRY_RUN:
                    command = make_safe_random_flange_coords()
                    with state_lock:
                        last_error = f"DRY RUN: random command not sent {command}"
                    print("[DRY RUN] Random safe flange command:", command)
                else:
                    command = make_safe_random_flange_coords()
                    pseudo_payload = {"status": "ok", "plan": {"flange_command": command}}
                    is_safe, reason, safe_command = validate_server_plan(pseudo_payload)
                    if not is_safe or safe_command is None:
                        with state_lock:
                            last_error = f"Random pose rejected locally: {reason}"
                        print("[SAFETY]", last_error)
                    else:
                        print("[RANDOM] safe flange command:", safe_command)
                        reached = wasab_arm_controller.send_flange_coords_and_wait(safe_command, abort_event=stop_request)
                        with state_lock:
                            last_error = None if reached else "Random pose timeout"
                continue

            if key == ord("w"):
                if throw_running.is_set():
                    print("[HOME] aborting throw before home return")
                    throw_abort.set()
                    if not config.DRY_RUN:
                        try:
                            wasab_arm_controller.stop_motion()
                        except Exception as exc:
                            print("[HOME] stop error:", exc)
                    if throw_thread is not None and throw_thread.is_alive():
                        throw_thread.join(timeout=1.0)
                    throw_running.clear()

                if config.DRY_RUN:
                    with state_lock:
                        last_error = "DRY RUN: Home command not sent"
                else:
                    reached = wasab_arm_controller.move_home_keep_gripper_closed(abort_event=stop_request)
                    gripper_closed_on_target = False
                    with state_lock:
                        last_error = None if reached else "HOME return timeout"
                continue

            if key != ord("g"):
                continue

            if throw_running.is_set():
                with state_lock:
                    last_error = "Pick ignored: throw mode is running"
                continue

            with state_lock:
                last_error = None
            try:
                # The robot is stationary here. Read its current Base-frame pose
                # and immediately capture a fresh calibration-size image.
                current_flange_coords = wasab_arm_controller.get_flange_coords()
                plan_frame, _ = get_latest_frame(min_seq=frame_seq, timeout_sec=1.0)
                if plan_frame is None:
                    plan_frame, _ = get_latest_frame(timeout_sec=0.5)
                if plan_frame is None:
                    raise RuntimeError("Cannot capture a current camera frame for pick planning")
                payload = request_wasab_operation_plan(plan_frame, current_flange_coords)

                print(
                    "[LAPTOP RESPONSE]\n",
                    json.dumps(payload, ensure_ascii=False, indent=2),
                )
                with state_lock:
                    last_payload = payload
                    last_payload_expires_at = time.monotonic() + 3.0

                is_safe, reason, command = validate_server_plan(payload)
                if not is_safe or command is None:
                    with state_lock:
                        last_error = f"Plan rejected locally: {reason}"
                    print("[SAFETY]", last_error)
                    continue

                if config.DRY_RUN:
                    print(
                        "[DRY RUN] Laptop plan validated; no robot command sent:",
                        command,
                    )
                    continue

                reached = wasab_arm_controller.send_flange_coords_and_wait(command, abort_event=stop_request)
                if reached:
                    try:
                        wasab_arm_controller.close_gripper()
                    except RuntimeError as exc:
                        gripper_closed_on_target = False
                        with state_lock:
                            last_error = f"GRIPPER ERROR: {exc}"
                        print(last_error)
                        continue
                    gripper_closed_on_target = True
                    last_pick_flange_command = list(command)
                    with state_lock:
                        last_error = None
                else:
                    gripper_closed_on_target = False
                    with state_lock:
                        last_error = (
                            "Target pose timeout; gripper remains unchanged"
                        )

            except (WaSaBServiceError, RuntimeError, ValueError) as exc:
                with state_lock:
                    last_error = f"ERROR: {type(exc).__name__}: {exc}"
                print(last_error)

    finally:
        stream_stop.set()
        capture_stop.set()
        remote_stop.set()
        with frame_condition:
            frame_condition.notify_all()
        if remote_thread is not None and remote_thread.is_alive():
            remote_thread.join(timeout=1.0)
        if stream_thread is not None and stream_thread.is_alive():
            stream_thread.join(timeout=1.0)
        if capture_thread.is_alive():
            capture_thread.join(timeout=1.0)

        if throw_running.is_set():
            throw_abort.set()
            if not config.DRY_RUN:
                try:
                    wasab_arm_controller.stop_motion()
                except Exception:
                    pass

        if throw_thread is not None and throw_thread.is_alive():
            throw_thread.join(timeout=1.0)

        with cap_lock:
            cap.release()
        if show_window:
            cv2.destroyAllWindows()
        print("Client terminated")


if __name__ == "__main__":
    main()
