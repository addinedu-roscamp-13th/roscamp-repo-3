"""Raspberry Pi execution entry point for the laptop-local YOLO pick/place service.

Keys:
  g: capture a fresh 640x480 frame + current Flange pose -> laptop plan -> validate -> pick
  p: print current Flange pose
  q: toggle gripper close/open
  s / servo-release: release all servos so the arm can be moved by hand
  k / servo-focus: focus/enable all servos
  f / place: place the held object: home -> last picked pose -> open gripper
  m / move: move to configured manual Flange pose
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
import subprocess
import sys
import threading
import time
from functools import lru_cache
from pathlib import Path
from queue import Empty, Queue
from typing import Any, Callable

import cv2
import numpy as np

from . import config
from .api_client import (
    WaSaBServiceError,
    check_wasab_service_health,
    request_wasab_apriltag_detection,
    request_wasab_marker_pickup_plan,
    request_wasab_marker_place_plan,
    request_wasab_object_detection,
    request_wasab_operation_plan,
    post_robot_log,
    send_udp_streamer_frame,
    update_workspace_overlay,
    upload_palm_hitbox_capture,
    upload_streamer_frame,
    stream_wasab_arm_commands,
)
from .hand_gesture import OpenPalmTrigger
from .robot_controller import JOINT_LIMITS_DEG, WaSaBArmController


def _gift_restock_destination_path() -> Path:
    return (
        Path(__file__).resolve().parent.parent
        / "config"
        / f"gift_restock_destination.{config.ARM_ID}.json"
    )


def load_gift_restock_destination() -> list[float] | None:
    path = _gift_restock_destination_path()
    if not path.exists():
        return None
    try:
        values = json.loads(path.read_text(encoding="utf-8"))
        if (
            isinstance(values, list)
            and len(values) == 6
            and all(isinstance(value, (int, float)) for value in values)
        ):
            return [float(value) for value in values]
    except (OSError, ValueError):
        pass
    return None


def save_gift_restock_destination(values: list[float]) -> None:
    path = _gift_restock_destination_path()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(values, indent=2), encoding="utf-8")


def record_palm_hitbox_sample(
    palm_uv: tuple[float, float],
    frame,
    target_count: int,
) -> tuple[int, Path | None, tuple[int, int, int, int] | None]:
    """Persist Palm Check samples and render a heatbox after the target count."""
    output_dir = config.PALM_HITBOX_CALIBRATION_OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    samples_path = output_dir / f"palm_hitbox_samples.{config.ARM_ID}.json"
    image_path = output_dir / f"palm_hitbox_result.{config.ARM_ID}.png"

    samples: list[dict[str, float | str]] = []
    if samples_path.exists():
        try:
            loaded = json.loads(samples_path.read_text(encoding="utf-8"))
            if isinstance(loaded, list):
                samples = [item for item in loaded if isinstance(item, dict)]
        except (OSError, ValueError):
            samples = []

    if len(samples) < target_count:
        samples.append(
            {
                "u": round(float(palm_uv[0]), 3),
                "v": round(float(palm_uv[1]), 3),
                "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            }
        )
        temporary_path = samples_path.with_suffix(".json.tmp")
        temporary_path.write_text(
            json.dumps(samples, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        temporary_path.replace(samples_path)

    if len(samples) < target_count:
        return len(samples), None, None

    samples = samples[:target_count]
    height, width = frame.shape[:2]
    points = np.asarray(
        [[float(item["u"]), float(item["v"])] for item in samples],
        dtype=np.float32,
    )
    x1 = max(0, int(np.percentile(points[:, 0], 5)) - 20)
    y1 = max(0, int(np.percentile(points[:, 1], 5)) - 20)
    x2 = min(width - 1, int(np.percentile(points[:, 0], 95)) + 20)
    y2 = min(height - 1, int(np.percentile(points[:, 1], 95)) + 20)

    density = np.zeros((height, width), dtype=np.float32)
    for u, v in points:
        cv2.circle(
            density,
            (max(0, min(width - 1, int(round(u)))),
             max(0, min(height - 1, int(round(v))))),
            22,
            1.0,
            -1,
        )
    density = cv2.GaussianBlur(density, (0, 0), 24.0)
    density_u8 = cv2.normalize(
        density, None, 0, 255, cv2.NORM_MINMAX
    ).astype(np.uint8)
    heatmap = cv2.applyColorMap(density_u8, cv2.COLORMAP_TURBO)
    base = cv2.addWeighted(frame, 0.55, heatmap, 0.45, 0.0)
    for u, v in points:
        cv2.circle(base, (int(round(u)), int(round(v))), 3, (255, 255, 255), -1)
    cv2.rectangle(base, (x1, y1), (x2, y2), (0, 255, 0), 3)
    cv2.putText(
        base,
        f"PALM HITBOX  n={len(samples)}  ({x1},{y1})-({x2},{y2})",
        (16, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.68,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    cv2.imwrite(str(image_path), base)
    summary_path = output_dir / f"palm_hitbox_result.{config.ARM_ID}.json"
    summary_path.write_text(
        json.dumps(
            {
                "sample_count": len(samples),
                "recommended_hitbox_px": [x1, y1, x2, y2],
                "image_width": width,
                "image_height": height,
                "image_path": str(image_path),
            },
            ensure_ascii=False,
            indent=2,
        ),
        encoding="utf-8",
    )
    return len(samples), image_path, (x1, y1, x2, y2)


def load_palm_hitbox_target_samples() -> int:
    settings_path = (
        config.PALM_HITBOX_CALIBRATION_OUTPUT_DIR
        / f"palm_hitbox_settings.{config.ARM_ID}.json"
    )
    try:
        payload = json.loads(settings_path.read_text(encoding="utf-8"))
        value = int(payload["target_samples"])
        if 1 <= value <= 1000:
            return value
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return config.PALM_HITBOX_CALIBRATION_TARGET_SAMPLES


def save_palm_hitbox_target_samples(value: int) -> None:
    output_dir = config.PALM_HITBOX_CALIBRATION_OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    settings_path = output_dir / f"palm_hitbox_settings.{config.ARM_ID}.json"
    settings_path.write_text(
        json.dumps({"target_samples": value}, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def load_palm_hitbox_norm() -> tuple[float, float, float, float] | None:
    result_path = (
        config.PALM_HITBOX_CALIBRATION_OUTPUT_DIR
        / f"palm_hitbox_result.{config.ARM_ID}.json"
    )
    try:
        payload = json.loads(result_path.read_text(encoding="utf-8"))
        box = payload["recommended_hitbox_px"]
        if not isinstance(box, list) or len(box) != 4:
            return None
        x1, y1, x2, y2 = (float(value) for value in box)
        return (
            x1 / config.CAMERA_FRAME_WIDTH,
            y1 / config.CAMERA_FRAME_HEIGHT,
            x2 / config.CAMERA_FRAME_WIDTH,
            y2 / config.CAMERA_FRAME_HEIGHT,
        )
    except (OSError, ValueError, KeyError, TypeError):
        return None


def save_colored_palm_hitbox_capture(
    frame,
    hitbox: tuple[float, float, float, float],
) -> Path:
    """Save a separate capture with a colored, borderless hitbox range."""
    height, width = frame.shape[:2]
    x1 = max(0, min(width - 1, int(round(hitbox[0] * width))))
    y1 = max(0, min(height - 1, int(round(hitbox[1] * height))))
    x2 = max(x1 + 1, min(width, int(round(hitbox[2] * width))))
    y2 = max(y1 + 1, min(height, int(round(hitbox[3] * height))))

    colored = frame.copy()
    region_height, region_width = y2 - y1, x2 - x1
    yy, xx = np.mgrid[0:region_height, 0:region_width]
    nx = (xx - (region_width - 1) / 2.0) / max(1.0, region_width / 2.0)
    ny = (yy - (region_height - 1) / 2.0) / max(1.0, region_height / 2.0)
    intensity = np.clip(1.0 - np.sqrt(nx * nx + ny * ny), 0.0, 1.0)
    heat = cv2.applyColorMap(
        np.asarray(intensity * 255.0, dtype=np.uint8),
        cv2.COLORMAP_TURBO,
    )
    colored[y1:y2, x1:x2] = cv2.addWeighted(
        frame[y1:y2, x1:x2],
        0.45,
        heat,
        0.55,
        0.0,
    )

    output_dir = config.PALM_HITBOX_CALIBRATION_OUTPUT_DIR
    output_dir.mkdir(parents=True, exist_ok=True)
    output_path = output_dir / f"palm_hitbox_capture.{config.ARM_ID}.png"
    cv2.imwrite(str(output_path), colored)
    return output_path


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

    workspace = (
        f"allowed X={config.SAFE_X_MM[0]:.1f}..{config.SAFE_X_MM[1]:.1f}, "
        f"Y={config.SAFE_Y_MM[0]:.1f}..{config.SAFE_Y_MM[1]:.1f}, "
        f"Z={config.SAFE_Z_MM[0]:.1f}..{config.SAFE_Z_MM[1]:.1f} mm"
    )
    for axis, value, limits in zip(
        ("X", "Y", "Z"),
        command[:3],
        (config.SAFE_X_MM, config.SAFE_Y_MM, config.SAFE_Z_MM),
    ):
        if not _is_in_range(value, limits):
            nearest = limits[0] if value < limits[0] else limits[1]
            delta = abs(value - nearest)
            direction = "below minimum" if value < limits[0] else "above maximum"
            return (
                False,
                f"unsafe {axis}={value:.2f}mm ({delta:.2f}mm {direction}); "
                f"candidate XYZ={command[:3]}; {workspace}",
                None,
            )
    if any(abs(v) > config.SAFE_EULER_ABS_DEG for v in command[3:]):
        return False, "unsafe Euler value", None

    return True, "ok", command


def send_marker_pickup_pose_and_wait(
    wasab_arm_controller: WaSaBArmController,
    desired_pose: list[float],
    *,
    speed: int,
    mode: int,
    abort_event: threading.Event,
    position_tolerance_mm: float | None = None,
) -> bool:
    """Bias the Cartesian command but verify the desired physical pose."""
    command_pose = list(desired_pose)
    for index, offset_mm in enumerate(
        config.MARKER_PICKUP_COMMAND_COMPENSATION_XYZ_MM
    ):
        command_pose[index] = round(command_pose[index] + offset_mm, 2)

    is_safe, reason, command_pose = validate_server_plan(
        {"status": "ok", "plan": {"flange_command": command_pose}}
    )
    if not is_safe or command_pose is None:
        print(
            "[SAFETY] Compensated marker pickup command rejected:",
            f"{reason}; desired={desired_pose}",
        )
        return False

    print(
        "[MARKER PICKUP] Cartesian compensation:",
        f"desired={desired_pose}",
        f"command={command_pose}",
    )
    wasab_arm_controller.send_flange_coords(command_pose, speed=speed, mode=mode)
    return wasab_arm_controller.wait_until_flange_pose(
        desired_pose,
        abort_event=abort_event,
        position_tolerance_mm=position_tolerance_mm,
    )


def is_no_target_detection_response(payload: dict[str, Any]) -> bool:
    if payload.get("status") == "ok":
        return False
    message = str(payload.get("message", "")).lower()
    return (
        "no target detection" in message
        or "target object was not detected" in message
        or "target-label/confidence policy" in message
        or is_partial_edge_pick_response(payload)
    )


def is_partial_edge_pick_response(payload: dict[str, Any]) -> bool:
    if payload.get("status") == "ok":
        return False
    rejected = payload.get("rejected_detections")
    if isinstance(rejected, list):
        for item in rejected:
            if not isinstance(item, dict):
                continue
            reason = str(item.get("reject_reason", "")).lower()
            if "partial_object_near_image_edge" in reason:
                return True
    return "partial target detections near the image edge" in str(payload.get("message", "")).lower()


def best_rejected_detection_for_centering(payload: dict[str, Any]) -> dict[str, Any] | None:
    rejected = payload.get("rejected_detections")
    if not isinstance(rejected, list):
        return None

    candidates: list[dict[str, Any]] = []
    for item in rejected:
        if not isinstance(item, dict):
            continue
        reason = str(item.get("reject_reason", "")).lower()
        bbox = item.get("bbox")
        if "partial_object_near_image_edge" not in reason:
            continue
        if not isinstance(bbox, list) or len(bbox) != 4:
            continue
        candidates.append(item)
    if not candidates:
        return None

    return max(candidates, key=lambda item: float(item.get("confidence", 0.0) or 0.0))


def apply_auto_rotated_pick_xy_correction(
    payload: dict[str, Any],
    command: list[float],
) -> tuple[list[float], str | None]:
    if not config.PICK_AUTO_ROTATED_XY_CORRECTION_ENABLED:
        return command, None

    debug = payload.get("debug", {})
    if not isinstance(debug, dict) or not debug.get("gripper_auto_rotated"):
        return command, None

    try:
        rz_offset = abs(float(debug.get("gripper_rz_offset_deg", 0.0)))
    except (TypeError, ValueError):
        rz_offset = 0.0
    if rz_offset < config.PICK_AUTO_ROTATED_MIN_RZ_OFFSET_DEG:
        return command, None

    x_offset, y_offset = config.PICK_AUTO_ROTATED_BASE_XY_OFFSET_MM
    if abs(x_offset) < 1e-9 and abs(y_offset) < 1e-9:
        return command, None

    corrected = list(command)
    corrected[0] = round(corrected[0] + x_offset, 2)
    corrected[1] = round(corrected[1] + y_offset, 2)
    message = (
        "Auto-rotated pick XY correction: "
        f"dx={x_offset:.1f} mm, dy={y_offset:.1f} mm, rz_offset={rz_offset:.1f} deg"
    )
    return corrected, message


def _clamp(value: float, limits: tuple[float, float]) -> float:
    return max(limits[0], min(limits[1], value))


def make_place_marker_view_joint_angles(current_joint_angles: list[float]) -> list[float]:
    """Rotate J6 before marker search so the held object blocks less of the camera."""
    if len(current_joint_angles) != 6:
        raise ValueError("current_joint_angles must contain six values")
    target = [float(value) for value in current_joint_angles]
    wrist_index = 5
    wrist_limits = JOINT_LIMITS_DEG[wrist_index]
    requested = target[wrist_index] + config.PLACE_MARKER_VIEW_RZ_OFFSET_DEG
    if not wrist_limits[0] <= requested <= wrist_limits[1]:
        raise ValueError(
            f"place marker-view J6={requested:.1f} is outside allowed range {wrist_limits}"
        )
    target[wrist_index] = requested
    return [round(value, 2) for value in target]


def make_marker_search_view_joint_targets(current_joint_angles: list[float]) -> list[list[float]]:
    """Build configured J4/J5-style view poses for marker search."""
    if len(current_joint_angles) != 6:
        raise ValueError("current_joint_angles must contain six values")
    base = [float(value) for value in current_joint_angles]
    targets = []
    seen = set()
    for offset_row in config.MARKER_SEARCH_VIEW_JOINT_OFFSETS_DEG:
        target = list(base)
        for joint_id, offset in zip(config.MARKER_SEARCH_VIEW_JOINTS, offset_row):
            joint_index = joint_id - 1
            target[joint_index] = _clamp(
                base[joint_index] + offset,
                JOINT_LIMITS_DEG[joint_index],
            )
        rounded = tuple(round(value, 2) for value in target)
        if rounded not in seen:
            seen.add(rounded)
            targets.append(list(rounded))
    return targets or [[round(value, 2) for value in base]]


def format_marker_search_view_joints(target_angles: list[float]) -> str:
    return ", ".join(
        f"J{joint_id}={target_angles[joint_id - 1]:.1f}"
        for joint_id in config.MARKER_SEARCH_VIEW_JOINTS
    )


def execute_pick_approach(
    wasab_arm_controller: WaSaBArmController,
    target_coords: list[float],
    abort_event: threading.Event | None = None,
) -> bool:
    if not config.PICK_TWO_STAGE_APPROACH_ENABLED or config.PICK_APPROACH_LIFT_Z_MM <= 1e-9:
        print(
            "[PICK] direct low-speed approach:",
            f"speed={config.PICK_FINAL_APPROACH_SPEED}",
            f"mode={config.PICK_FINAL_APPROACH_MODE}",
        )
        return wasab_arm_controller.send_flange_coords_and_wait(
            target_coords,
            speed=config.PICK_FINAL_APPROACH_SPEED,
            mode=config.PICK_FINAL_APPROACH_MODE,
            abort_event=abort_event,
        )

    pre_pick = list(target_coords)
    pre_pick[2] = round(pre_pick[2] + config.PICK_APPROACH_LIFT_Z_MM, 2)
    is_safe, reason, safe_pre_pick = validate_server_plan(
        {"status": "ok", "plan": {"flange_command": pre_pick}}
    )
    if not is_safe or safe_pre_pick is None:
        wasab_arm_controller.last_wait_timeout_reason = f"pre-pick rejected: {reason}"
        print("[SAFETY] Pick pre-approach rejected:", reason, pre_pick)
        return False

    print(
        "[PICK] two-stage approach:",
        f"pre_z=+{config.PICK_APPROACH_LIFT_Z_MM:.1f}mm",
        f"approach_speed={config.PICK_APPROACH_SPEED}",
        f"final_speed={config.PICK_FINAL_APPROACH_SPEED}",
        f"final_mode={config.PICK_FINAL_APPROACH_MODE}",
    )
    print("[PICK] pre-pick target:", safe_pre_pick)
    if not wasab_arm_controller.send_flange_coords_and_wait(
        safe_pre_pick,
        speed=config.PICK_APPROACH_SPEED,
        abort_event=abort_event,
    ):
        return False

    print("[PICK] final-pick target:", target_coords)
    return wasab_arm_controller.send_flange_coords_and_wait(
        target_coords,
        speed=config.PICK_FINAL_APPROACH_SPEED,
        mode=config.PICK_FINAL_APPROACH_MODE,
        abort_event=abort_event,
    )


def execute_place_final_approach(
    wasab_arm_controller: WaSaBArmController,
    target_coords: list[float],
    abort_event: threading.Event | None = None,
) -> bool:
    if not config.PLACE_APPROACH_SLOWDOWN_ENABLED or len(config.PLACE_APPROACH_SPEEDS) <= 1:
        return wasab_arm_controller.send_flange_coords_and_wait(
            target_coords,
            speed=config.PLACE_APPROACH_SPEED,
            abort_event=abort_event,
        )

    start_coords = wasab_arm_controller.get_flange_coords()
    step_count = len(config.PLACE_APPROACH_SPEEDS)
    for index, speed in enumerate(config.PLACE_APPROACH_SPEEDS, start=1):
        ratio = index / step_count
        waypoint = [
            round(start + (target - start) * ratio, 2)
            for start, target in zip(start_coords, target_coords)
        ]
        print(
            "[PLACE] final approach step:",
            f"{index}/{step_count}",
            f"speed={speed}",
            waypoint,
        )
        if not wasab_arm_controller.send_flange_coords_and_wait(
            waypoint,
            speed=speed,
            abort_event=abort_event,
        ):
            return False
    return True


STOP_KEY = 1005
GESTURE_ON_KEY = 1006
GESTURE_OFF_KEY = 1007
PICKUP_TUNING_KEY = 1009
PICK_PLACE_KEY = 1010
PALM_CHECK_KEY = 1011
GIFT_SUPPLY_PICK_KEY = 1012
RECYCLE_KEY = 1013
HELP_KEY = 1014
VISION_SWEEP_ON_KEY = 1015
VISION_SWEEP_OFF_KEY = 1016
VISION_SWEEP_FACE_ON_KEY = 1017
VISION_SWEEP_FIRE_ON_KEY = 1018
VISION_SWEEP_TRACKING_ON_KEY = 1019
FIRE_SUPPRESS_CLOSE_KEY = 1020
FIRE_SUPPRESS_OPEN_KEY = 1021
PICK_TARGET_LABEL = "coca-cola"
RECYCLE_TARGET_LABELS = ("trash", "water")


REMOTE_COMMAND_TO_KEY = {
    "g": ord("g"),
    "pick": ord("g"),
    "pick-place": PICK_PLACE_KEY,
    "gift-supply-pick": GIFT_SUPPLY_PICK_KEY,
    "restock": GIFT_SUPPLY_PICK_KEY,
    "recycle": RECYCLE_KEY,
    "help": HELP_KEY,
    "pickup-tuning": PICKUP_TUNING_KEY,
    "p": ord("p"),
    "pose": ord("p"),
    "q": ord("q"),
    "gripper": ord("q"),
    "s": ord("s"),
    "servo-release": ord("s"),
    "k": ord("k"),
    "servo-focus": ord("k"),
    "f": ord("f"),
    "place": ord("f"),
    "m": ord("m"),
    "move": ord("m"),
    "w": ord("w"),
    "home": ord("w"),
    "gesture-on": GESTURE_ON_KEY,
    "gesture_on": GESTURE_ON_KEY,
    "gesture-off": GESTURE_OFF_KEY,
    "gesture_off": GESTURE_OFF_KEY,
    "vision-sweep-on": VISION_SWEEP_ON_KEY,
    "vision_sweep_on": VISION_SWEEP_ON_KEY,
    "vision-sweep-face-on": VISION_SWEEP_FACE_ON_KEY,
    "vision_sweep_face_on": VISION_SWEEP_FACE_ON_KEY,
    "vision-sweep-fire-on": VISION_SWEEP_FIRE_ON_KEY,
    "vision_sweep_fire_on": VISION_SWEEP_FIRE_ON_KEY,
    "vision-sweep-tracking-on": VISION_SWEEP_TRACKING_ON_KEY,
    "vision_sweep_tracking_on": VISION_SWEEP_TRACKING_ON_KEY,
    "vision-sweep-off": VISION_SWEEP_OFF_KEY,
    "vision_sweep_off": VISION_SWEEP_OFF_KEY,
    "fire-suppress-close": FIRE_SUPPRESS_CLOSE_KEY,
    "fire_suppress_close": FIRE_SUPPRESS_CLOSE_KEY,
    "fire-suppress-open": FIRE_SUPPRESS_OPEN_KEY,
    "fire_suppress_open": FIRE_SUPPRESS_OPEN_KEY,
    "palm-check": PALM_CHECK_KEY,
    "stop": STOP_KEY,
    "halt": STOP_KEY,
    "emergency-stop": STOP_KEY,
    "emergency_stop": STOP_KEY,
    "x": ord("x"),
    "exit": ord("x"),
    "calibration": ord("c"),
}


def remote_command_to_key(command: str) -> int | None:
    return REMOTE_COMMAND_TO_KEY.get(command.lower().strip())


def _wait_or_abort(period_sec: float, abort_event: threading.Event | None = None) -> bool:
    deadline = time.monotonic() + period_sec
    while time.monotonic() < deadline:
        if abort_event is not None and abort_event.is_set():
            return False
        time.sleep(min(0.005, max(0.0, deadline - time.monotonic())))
    return True


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


def make_pick_search_joint_targets(base_angles: list[float]) -> list[list[float]]:
    targets: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()

    def append_target(target: list[float]) -> None:
        key = tuple(round(value, 3) for value in target)
        if key in seen:
            return
        seen.add(key)
        targets.append(target)

    if config.PICK_SEARCH_GRID_ENABLED:
        first_joint, second_joint = config.PICK_SEARCH_GRID_JOINTS
        first_index = first_joint - 1
        second_index = second_joint - 1
        first_limits = JOINT_LIMITS_DEG[first_index]
        second_limits = JOINT_LIMITS_DEG[second_index]
        ordered_offsets = sorted(
            config.PICK_SEARCH_GRID_OFFSETS_DEG,
            key=lambda value: (abs(value), value),
        )
        for first_offset in ordered_offsets:
            for second_offset in ordered_offsets:
                target = list(base_angles)
                target[first_index] = _clamp(base_angles[first_index] + first_offset, first_limits)
                target[second_index] = _clamp(base_angles[second_index] + second_offset, second_limits)
                append_target(target)
        return targets

    for joint_id in config.PICK_SEARCH_JOINTS:
        joint_index = joint_id - 1
        joint_limits = JOINT_LIMITS_DEG[joint_index]
        for offset in config.PICK_SEARCH_OFFSETS_DEG:
            target = list(base_angles)
            target[joint_index] = _clamp(base_angles[joint_index] + offset, joint_limits)
            append_target(target)

    return targets


def make_pick_search_flange_targets() -> list[list[float]]:
    targets: list[list[float]] = []
    seen: set[tuple[float, ...]] = set()
    base = [float(value) for value in config.PICK_SEARCH_MAX_FLANGE_COORDS]

    if config.PICK_SEARCH_TRANSLATION_AXIS:
        axis_index = {"x": 0, "y": 1, "z": 2}[config.PICK_SEARCH_TRANSLATION_AXIS]
        axis_limits = {
            "x": config.SAFE_X_MM,
            "y": config.SAFE_Y_MM,
            "z": config.SAFE_Z_MM,
        }[config.PICK_SEARCH_TRANSLATION_AXIS]
        offsets = config.PICK_SEARCH_TRANSLATION_OFFSETS_MM
    else:
        axis_index = {"rx": 3, "ry": 4, "rz": 5}[config.PICK_SEARCH_ANGLE_AXIS]
        axis_limits = (-180.0, 180.0)
        offsets = config.PICK_SEARCH_ANGLE_OFFSETS_DEG

    for offset in offsets:
        target = list(base)
        target[axis_index] = _clamp(base[axis_index] + offset, axis_limits)
        key = tuple(round(value, 3) for value in target)
        if key in seen:
            continue
        seen.add(key)
        targets.append([round(value, 2) for value in target])

    # After checking both sides, widen the downward camera view by lifting Z
    # while keeping the home X/Y and flange orientation unchanged.
    if config.PICK_SEARCH_TRANSLATION_AXIS:
        for lift_offset in config.PICK_SEARCH_LIFT_OFFSETS_MM:
            target = list(base)
            target[2] = _clamp(base[2] + lift_offset, config.SAFE_Z_MM)
            key = tuple(round(value, 3) for value in target)
            if key in seen:
                continue
            seen.add(key)
            targets.append([round(value, 2) for value in target])

    return targets


def make_pick_centering_target_angles(
    current_angles: list[float],
    payload: dict[str, Any],
) -> tuple[list[float] | None, str]:
    detection = best_rejected_detection_for_centering(payload)
    if detection is None:
        return None, "no edge detection to center"

    bbox = detection.get("bbox")
    if not isinstance(bbox, list) or len(bbox) != 4:
        return None, "edge detection bbox is invalid"
    try:
        x1, y1, x2, y2 = (float(value) for value in bbox)
        image_width = float(payload.get("image_width", config.CAMERA_FRAME_WIDTH))
        image_height = float(payload.get("image_height", config.CAMERA_FRAME_HEIGHT))
    except (TypeError, ValueError):
        return None, "edge detection bbox is not numeric"
    if image_width <= 0.0 or image_height <= 0.0:
        return None, "image size is invalid"

    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    dx_px = center_x - image_width / 2.0
    dy_px = center_y - image_height / 2.0
    if abs(dx_px) <= config.PICK_CENTERING_DEADBAND_PX and abs(dy_px) <= config.PICK_CENTERING_DEADBAND_PX:
        return None, f"edge detection already near center dx={dx_px:.0f}px dy={dy_px:.0f}px"

    j1_delta = _clamp(
        dx_px * config.PICK_CENTERING_J1_GAIN_DEG_PER_PX,
        (-config.PICK_CENTERING_MAX_STEP_DEG, config.PICK_CENTERING_MAX_STEP_DEG),
    )
    j2_delta = _clamp(
        dy_px * config.PICK_CENTERING_J2_GAIN_DEG_PER_PX,
        (-config.PICK_CENTERING_MAX_STEP_DEG, config.PICK_CENTERING_MAX_STEP_DEG),
    )
    j5_delta = _clamp(
        dy_px * config.PICK_CENTERING_J5_GAIN_DEG_PER_PX,
        (-config.PICK_CENTERING_MAX_STEP_DEG, config.PICK_CENTERING_MAX_STEP_DEG),
    )

    target = list(current_angles)
    target[0] = _clamp(target[0] + j1_delta, JOINT_LIMITS_DEG[0])
    target[1] = _clamp(target[1] + j2_delta, JOINT_LIMITS_DEG[1])
    target[4] = _clamp(target[4] + j5_delta, JOINT_LIMITS_DEG[4])
    return (
        [round(value, 2) for value in target],
        (
            f"center edge target label={detection.get('label', '?')} "
            f"dx={dx_px:.0f}px dy={dy_px:.0f}px "
            f"dJ1={j1_delta:.2f} dJ2={j2_delta:.2f} dJ5={j5_delta:.2f}"
        ),
    )


@lru_cache(maxsize=1)
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


def detect_april_marker(
    frame,
    allowed_ids: set[int] | None = None,
) -> dict[str, Any] | None:
    detector, dictionary, parameters = _create_marker_detector()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if detector is not None:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is None or len(ids) == 0:
        return None

    ids_flat = ids.flatten().astype(int).tolist()
    effective_ids = config.MARKER_SEARCH_TARGET_IDS if allowed_ids is None else allowed_ids
    selected_indices = [
        index for index, marker_id in enumerate(ids_flat)
        if not effective_ids or marker_id in effective_ids
    ]
    if not selected_indices:
        return None

    selected = selected_indices[0]
    points = corners[selected].reshape(-1, 2).astype(float)
    center = points.mean(axis=0)
    x_min, y_min = points.min(axis=0)
    x_max, y_max = points.max(axis=0)
    marker_id = ids_flat[selected]
    marker_role = (
        "pickup"
        if marker_id in config.MARKER_PICKUP_IDS
        else "place"
        if marker_id in config.MARKER_PLACE_IDS
        else "target"
    )
    return {
        "id": marker_id,
        "role": marker_role,
        "ids": ids_flat,
        "center": [float(center[0]), float(center[1])],
        "corners": [[float(x), float(y)] for x, y in points],
        "bbox": [float(x_min), float(y_min), float(x_max), float(y_max)],
        "corner_count": int(len(points)),
    }


def detect_recycle_bin_color(
    frame: np.ndarray,
    color_name: str,
) -> dict[str, Any] | None:
    """Return the largest sufficiently sized red or blue region in the frame."""
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    saturation_min = config.RECYCLE_COLOR_SATURATION_MIN
    value_min = config.RECYCLE_COLOR_VALUE_MIN
    if color_name == "red":
        mask = cv2.bitwise_or(
            cv2.inRange(
                hsv,
                np.array([0, saturation_min, value_min], dtype=np.uint8),
                np.array([10, 255, 255], dtype=np.uint8),
            ),
            cv2.inRange(
                hsv,
                np.array([170, saturation_min, value_min], dtype=np.uint8),
                np.array([179, 255, 255], dtype=np.uint8),
            ),
        )
    elif color_name == "blue":
        mask = cv2.inRange(
            hsv,
            np.array([95, saturation_min, value_min], dtype=np.uint8),
            np.array([135, 255, 255], dtype=np.uint8),
        )
    else:
        raise ValueError(f"Unsupported recycle bin color: {color_name}")

    kernel = np.ones((5, 5), dtype=np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    area = float(cv2.contourArea(contour))
    image_area = float(frame.shape[0] * frame.shape[1])
    area_ratio = area / image_area if image_area > 0 else 0.0
    if area_ratio < config.RECYCLE_COLOR_MIN_AREA_RATIO:
        return None
    moments = cv2.moments(contour)
    if abs(moments["m00"]) <= 1e-9:
        return None
    center = [
        round(float(moments["m10"] / moments["m00"]), 1),
        round(float(moments["m01"] / moments["m00"]), 1),
    ]
    x, y, width, height = cv2.boundingRect(contour)
    return {
        "color": color_name,
        "center": center,
        "bbox": [int(x), int(y), int(x + width), int(y + height)],
        "area_ratio": round(area_ratio, 4),
    }


def build_dynamic_recycle_command(
    plan_payload: dict[str, Any],
    reference_flange_coords: list[float],
    max_xy_offset_mm: float,
) -> list[float]:
    """Use live Hand-Eye X/Y while pinning the measured release Z/orientation."""
    plan = plan_payload.get("plan")
    raw_command = plan.get("flange_command") if isinstance(plan, dict) else None
    try:
        planned = [float(value) for value in raw_command]
    except (TypeError, ValueError):
        planned = []
    if len(planned) != 6 or not np.isfinite(np.asarray(planned)).all():
        raise ValueError("dynamic recycle plan has an invalid flange_command")
    if len(reference_flange_coords) != 6:
        raise ValueError("dynamic recycle reference must contain six values")
    command = [
        planned[0],
        planned[1],
        float(reference_flange_coords[2]),
        *[float(value) for value in reference_flange_coords[3:]],
    ]
    delta_x = command[0] - float(reference_flange_coords[0])
    delta_y = command[1] - float(reference_flange_coords[1])
    if abs(delta_x) > max_xy_offset_mm or abs(delta_y) > max_xy_offset_mm:
        raise ValueError(
            "dynamic recycle XY exceeds measured-bin safety window: "
            f"delta=({delta_x:+.1f}, {delta_y:+.1f})mm "
            f"limit=±{max_xy_offset_mm:.1f}mm"
        )
    if not -260.0 <= command[0] <= 260.0:
        raise ValueError(f"dynamic recycle X={command[0]:.1f}mm exceeds ±260mm")
    if not -281.0 <= command[1] <= 281.0:
        raise ValueError(f"dynamic recycle Y={command[1]:.1f}mm exceeds ±281mm")
    return [round(value, 2) for value in command]


def execute_marker_search(
    wasab_arm_controller: WaSaBArmController,
    get_frame: Callable[..., tuple[object | None, int]],
    status_callback: Callable[[str], None],
    abort_event: threading.Event | None = None,
    allowed_ids: set[int] | None = None,
    detection_fn: Callable[[object], dict[str, Any] | None] | None = None,
    search_name: str = "Marker",
    positive_only: bool = False,
    pan_range_deg: float | None = None,
    use_view_joints: bool | None = None,
) -> tuple[bool, str, dict[str, Any] | None]:
    base_angles = wasab_arm_controller.get_joint_angles()
    view_joint_enabled = (
        config.MARKER_SEARCH_VIEW_JOINT_ENABLED
        if use_view_joints is None
        else bool(use_view_joints)
    )
    view_targets = (
        make_marker_search_view_joint_targets(base_angles)
        if view_joint_enabled
        else [list(base_angles)]
    )
    view_index = 0

    if view_joint_enabled:
        view_angles = view_targets[view_index]
        status_callback(
            f"Marker view preset: {format_marker_search_view_joints(view_angles)}"
        )
        print(
            "[MARKER] view preset:",
            format_marker_search_view_joints(view_angles),
        )
        wasab_arm_controller.send_joint_angles(
            view_angles,
            speed=config.MARKER_SEARCH_VIEW_JOINT_SPEED,
            async_command=True,
        )
        if not wasab_arm_controller.wait_until_joint_angles(
            view_angles,
            timeout_sec=config.MOVE_TIMEOUT_SEC,
            tolerance_deg=config.POSE_ANGLE_TOL_DEG,
            abort_event=abort_event,
        ):
            return False, "Marker view preset timeout", None

    base_angles = wasab_arm_controller.get_joint_angles()
    pan_index = config.MARKER_SEARCH_PAN_JOINT - 1
    joint_limits = JOINT_LIMITS_DEG[pan_index]
    effective_pan_range_deg = (
        config.MARKER_SEARCH_PAN_RANGE_DEG
        if pan_range_deg is None
        else float(pan_range_deg)
    )
    pan_low = (
        base_angles[pan_index]
        if positive_only
        else max(joint_limits[0], base_angles[pan_index] - effective_pan_range_deg)
    )
    pan_high = min(
        joint_limits[1],
        base_angles[pan_index] + effective_pan_range_deg,
    )
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
        f"target_ids={sorted(config.MARKER_SEARCH_TARGET_IDS if allowed_ids is None else allowed_ids) or 'any'}",
        f"J{config.MARKER_SEARCH_PAN_JOINT}={pan_low:.1f}..{pan_high:.1f}",
        f"direction={'positive-only' if positive_only else 'bidirectional'}",
        f"view_joints={config.MARKER_SEARCH_VIEW_JOINTS if view_joint_enabled else 'off'}",
    )

    while True:
        if abort_event is not None and abort_event.is_set():
            wasab_arm_controller.stop_motion()
            return False, "Marker search stopped", None
        if deadline is not None and time.monotonic() >= deadline:
            return False, "Marker search timeout", None

        frame, _ = get_frame(timeout_sec=period_sec)
        if frame is not None:
            detection = (
                detection_fn(frame)
                if detection_fn is not None
                else detect_april_marker(frame, allowed_ids=allowed_ids)
            )
            if detection is not None:
                message = (
                    f"{search_name} found: id={detection['id']} role={detection['role']} "
                    f"center=({detection['center'][0]:.0f}, {detection['center'][1]:.0f})"
                )
                print(
                    f"[{search_name.upper().replace(' ', '_')}]",
                    message,
                    "all_ids=",
                    detection.get("ids", []),
                )
                wasab_arm_controller.stop_motion()
                return True, message, detection

        target_angle += config.MARKER_SEARCH_STEP_DEG * search_dir
        hit_boundary = False
        if target_angle >= pan_high or target_angle <= pan_low:
            target_angle = _clamp(target_angle, (pan_low, pan_high))
            search_dir *= -1.0
            hit_boundary = True

        if view_joint_enabled:
            if hit_boundary:
                view_index = (view_index + 1) % len(view_targets)
            target_angles = list(view_targets[view_index])
            target_angles[pan_index] = target_angle
            wasab_arm_controller.send_joint_angles(
                target_angles,
                speed=min(config.MARKER_SEARCH_SPEED, config.MARKER_SEARCH_VIEW_JOINT_SPEED),
                async_command=True,
            )
            status_callback(
                f"Searching marker... J{config.MARKER_SEARCH_PAN_JOINT}={target_angle:.1f}, "
                f"{format_marker_search_view_joints(target_angles)}"
            )
        else:
            wasab_arm_controller.send_joint_angle(
                config.MARKER_SEARCH_PAN_JOINT,
                target_angle,
                config.MARKER_SEARCH_SPEED,
            )
            status_callback(f"Searching marker... J{config.MARKER_SEARCH_PAN_JOINT}={target_angle:.1f}")
        time.sleep(period_sec)


def detect_black_table(frame: object) -> dict[str, Any] | None:
    """Return the largest fully visible dark quadrilateral suitable for placement."""
    if frame is None or not hasattr(frame, "shape"):
        return None
    height, width = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.inRange(gray, 0, config.BLACK_TABLE_MAX_VALUE)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    frame_area = float(width * height)
    margin = config.BLACK_TABLE_BORDER_MARGIN_PX
    candidates: list[tuple[float, dict[str, Any]]] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        area_ratio = area / frame_area
        if not config.BLACK_TABLE_MIN_AREA_RATIO <= area_ratio <= config.BLACK_TABLE_MAX_AREA_RATIO:
            continue
        perimeter = cv2.arcLength(contour, True)
        polygon = cv2.approxPolyDP(contour, 0.025 * perimeter, True)
        if len(polygon) != 4 or not cv2.isContourConvex(polygon):
            continue
        points = polygon.reshape(4, 2)
        if margin > 0 and any(
            x <= margin or x >= width - 1 - margin or y <= margin or y >= height - 1 - margin
            for x, y in points
        ):
            continue
        rect = cv2.minAreaRect(contour)
        center_x, center_y = rect[0]
        box_width, box_height = rect[1]
        if min(box_width, box_height) < 20.0:
            continue
        angle = float(rect[2])
        if box_width < box_height:
            angle += 90.0
        x, y, bbox_width, bbox_height = cv2.boundingRect(contour)
        detection = {
            "id": -1,
            "ids": [],
            "role": "black_table",
            "bbox": [float(x), float(y), float(x + bbox_width), float(y + bbox_height)],
            "center": [float(center_x), float(center_y)],
            "corners": [[float(px), float(py)] for px, py in points],
            "area_ratio": round(area_ratio, 4),
            "grip_axis_image_deg": angle % 180.0,
            "object_plane_z_base_mm": config.BLACK_TABLE_SURFACE_Z_BASE_MM,
        }
        candidates.append((area, detection))
    return max(candidates, key=lambda item: item[0])[1] if candidates else None


def execute_black_table_search(
    wasab_arm_controller: WaSaBArmController,
    get_frame: Callable[..., tuple[object | None, int]],
    status_callback: Callable[[str], None],
    abort_event: threading.Event | None = None,
) -> tuple[bool, str, dict[str, Any] | None]:
    """Search along Base +X at the configured maximum camera Z."""
    search_pose = list(config.HOME_FLANGE_COORDS)
    search_pose[2] = config.BLACK_TABLE_SEARCH_Z_MM
    start_x = search_pose[0]
    offsets = [0.0]
    offset = config.BLACK_TABLE_SEARCH_X_STEP_MM
    while offset < config.BLACK_TABLE_SEARCH_X_RANGE_MM:
        offsets.append(offset)
        offset += config.BLACK_TABLE_SEARCH_X_STEP_MM
    offsets.append(config.BLACK_TABLE_SEARCH_X_RANGE_MM)

    for x_offset in offsets:
        if abort_event is not None and abort_event.is_set():
            wasab_arm_controller.stop_motion()
            return False, "Black table search stopped", None

        target = list(search_pose)
        target[0] = round(start_x + x_offset, 2)
        is_safe, reason, safe_target = validate_server_plan(
            {"status": "ok", "plan": {"flange_command": target}}
        )
        if not is_safe or safe_target is None:
            return False, f"Black table search rejected: {reason}", None
        status_callback(
            f"Searching black table... Base X +{x_offset:.0f}mm, "
            f"Z={safe_target[2]:.1f}mm"
        )
        print("[BLACK TABLE] max-Z search target:", safe_target)
        if not wasab_arm_controller.send_flange_coords_and_wait(
            safe_target,
            speed=config.BLACK_TABLE_SEARCH_START_SPEED,
            abort_event=abort_event,
        ):
            return False, "Black table max-Z search move timeout", None

        frame, _ = get_frame(timeout_sec=1.0)
        if frame is None:
            frame, _ = get_frame(timeout_sec=0.5)
        detection = detect_black_table(frame)
        if detection is not None:
            message = (
                "Black table found: "
                f"center=({detection['center'][0]:.0f}, {detection['center'][1]:.0f}), "
                f"fill={detection['area_ratio'] * 100.0:.1f}%, "
                f"Base X offset=+{x_offset:.0f}mm"
            )
            print("[BLACK_TABLE]", message)
            return True, message, detection

    return (
        False,
        "Black table did not fill at least "
        f"{config.BLACK_TABLE_MIN_AREA_RATIO * 100.0:.0f}% within "
        f"Home X..X+{config.BLACK_TABLE_SEARCH_X_RANGE_MM:.0f}mm",
        None,
    )


def request_pick_plan_from_current_view(
    wasab_arm_controller: WaSaBArmController,
    get_frame: Callable[..., tuple[object | None, int]],
    min_seq: int | None,
    timeout_sec: float,
    target_label: str | None = None,
) -> tuple[dict[str, Any], int]:
    current_flange_coords = wasab_arm_controller.get_flange_coords()
    plan_frame, frame_seq = get_frame(min_seq=min_seq, timeout_sec=timeout_sec)
    if plan_frame is None:
        plan_frame, frame_seq = get_frame(timeout_sec=0.5)
    if plan_frame is None:
        raise RuntimeError("Cannot capture a current camera frame for pick planning")
    upload_streamer_frame(plan_frame)
    return (
        request_wasab_operation_plan(
            plan_frame,
            current_flange_coords,
            target_label=target_label,
        ),
        frame_seq,
    )


def find_gift_supply_pick_plan(
    wasab_arm_controller: WaSaBArmController,
    get_frame: Callable[..., tuple[object | None, int]],
    min_seq: int | None,
    status_callback: Callable[[str], None],
    abort_event: threading.Event | None = None,
) -> tuple[dict[str, Any], int]:
    """Check once from Left HOME and remain HOME when no Coca-Cola is found."""
    base_angles = wasab_arm_controller.get_joint_angles()
    latest_payload: dict[str, Any] = {
        "status": "not_found",
        "message": "Target object was not detected.",
    }
    latest_seq = min_seq if min_seq is not None else 0
    found = False
    try:
        status_callback("Restock: checking for Coca-Cola from HOME")
        print(
            "[RESTOCK] check from current HOME; no search motion:",
            base_angles,
        )
        if config.GIFT_SUPPLY_SEARCH_SETTLE_SEC > 0 and not _wait_or_abort(
            config.GIFT_SUPPLY_SEARCH_SETTLE_SEC,
            abort_event,
        ):
            return latest_payload, latest_seq
        plan_frame, latest_seq = get_frame(
            min_seq=latest_seq,
            timeout_sec=config.GIFT_SUPPLY_SEARCH_FRAME_TIMEOUT_SEC,
        )
        if plan_frame is None:
            latest_payload = {
                "status": "not_found",
                "message": "No current camera frame for Restock detection.",
            }
        else:
            upload_streamer_frame(plan_frame)
            latest_payload = request_wasab_object_detection(
                plan_frame,
                PICK_TARGET_LABEL,
            )
        if latest_payload.get("status") == "ok":
            found = True
            print("[RESTOCK] Coca-Cola found from HOME")
        else:
            print("[RESTOCK] no Coca-Cola; remain HOME")
        return latest_payload, latest_seq
    finally:
        if not found and (abort_event is None or not abort_event.is_set()):
            wasab_arm_controller.send_joint_angles(
                base_angles,
                speed=config.GIFT_SUPPLY_SEARCH_SPEED,
                async_command=True,
            )
            wasab_arm_controller.wait_until_joint_angles(
                base_angles,
                timeout_sec=config.MOVE_TIMEOUT_SEC,
                tolerance_deg=max(config.POSE_ANGLE_TOL_DEG, 3.0),
                abort_event=abort_event,
            )


def find_pick_plan_by_joint_search(
    wasab_arm_controller: WaSaBArmController,
    get_frame: Callable[..., tuple[object | None, int]],
    first_payload: dict[str, Any],
    first_frame_seq: int,
    status_callback: Callable[[str], None],
    abort_event: threading.Event | None = None,
    target_label: str | None = None,
) -> dict[str, Any]:
    if config.DRY_RUN or not is_no_target_detection_response(first_payload):
        return first_payload
    centering_requested = (
        config.PICK_CENTERING_ENABLED
        and is_partial_edge_pick_response(first_payload)
    )
    if not config.PICK_SEARCH_ENABLED and not centering_requested:
        return first_payload

    base_angles = wasab_arm_controller.get_joint_angles()
    base_flange_coords = wasab_arm_controller.get_flange_coords()
    latest_payload = first_payload
    latest_frame_seq = first_frame_seq
    found_payload = False

    if config.PICK_CENTERING_ENABLED and is_partial_edge_pick_response(latest_payload):
        print("[PICK CENTERING] started")
        for attempt in range(1, config.PICK_CENTERING_MAX_ATTEMPTS + 1):
            if abort_event is not None and abort_event.is_set():
                wasab_arm_controller.stop_motion()
                status_callback("Pick centering stopped")
                return latest_payload

            current_angles = wasab_arm_controller.get_joint_angles()
            target_angles, centering_message = make_pick_centering_target_angles(
                current_angles,
                latest_payload,
            )
            if target_angles is None:
                print("[PICK CENTERING]", centering_message)
                break

            status_callback(f"Pick centering {attempt}/{config.PICK_CENTERING_MAX_ATTEMPTS}")
            print(
                f"[PICK CENTERING] attempt {attempt}/{config.PICK_CENTERING_MAX_ATTEMPTS}:",
                centering_message,
                "target=",
                target_angles,
            )
            wasab_arm_controller.send_joint_angles(
                target_angles,
                speed=config.PICK_CENTERING_SPEED,
                async_command=True,
            )
            if not wasab_arm_controller.wait_until_joint_angles(
                target_angles,
                timeout_sec=config.MOVE_TIMEOUT_SEC,
                tolerance_deg=max(config.POSE_ANGLE_TOL_DEG, 3.0),
                abort_event=abort_event,
            ):
                status_callback("Pick centering motion timeout")
                print("[PICK CENTERING] joint target timeout")
                break
            if config.PICK_CENTERING_SETTLE_SEC > 0:
                if not _wait_or_abort(config.PICK_CENTERING_SETTLE_SEC, abort_event):
                    wasab_arm_controller.stop_motion()
                    status_callback("Pick centering stopped")
                    return latest_payload

            latest_payload, latest_frame_seq = request_pick_plan_from_current_view(
                wasab_arm_controller,
                get_frame,
                latest_frame_seq,
                config.PICK_SEARCH_FRAME_TIMEOUT_SEC,
                target_label=target_label,
            )
            if latest_payload.get("status") == "ok":
                print("[PICK CENTERING] target centered and planned")
                status_callback(f"Pick centering found target at attempt {attempt}")
                found_payload = True
                return latest_payload
            if not is_partial_edge_pick_response(latest_payload):
                return latest_payload

    if not config.PICK_SEARCH_ENABLED:
        if (
            config.PICK_SEARCH_RETURN_TO_START
            and (abort_event is None or not abort_event.is_set())
        ):
            print("[PICK CENTERING] target not centered; returning to start")
            wasab_arm_controller.send_joint_angles(
                base_angles,
                speed=config.PICK_CENTERING_SPEED,
                async_command=True,
            )
            wasab_arm_controller.wait_until_joint_angles(
                base_angles,
                timeout_sec=config.MOVE_TIMEOUT_SEC,
                tolerance_deg=max(config.POSE_ANGLE_TOL_DEG, 3.0),
                abort_event=abort_event,
            )
        return latest_payload

    targets = (
        make_pick_search_flange_targets()
        if config.PICK_SEARCH_USE_FLANGE_POSE
        else make_pick_search_joint_targets(base_angles)
    )
    if len(targets) <= 1:
        return latest_payload

    first_target_index = 0 if config.PICK_SEARCH_USE_FLANGE_POSE else 1
    search_view_count = len(targets) - first_target_index
    status_callback(f"Pick search started: {search_view_count} alternate views")
    print(
        "[PICK SEARCH] started:",
        "mode=flange" if config.PICK_SEARCH_USE_FLANGE_POSE else "mode=joint",
        (
            f"axis={config.PICK_SEARCH_TRANSLATION_AXIS or config.PICK_SEARCH_ANGLE_AXIS} "
            f"offsets={config.PICK_SEARCH_TRANSLATION_OFFSETS_MM if config.PICK_SEARCH_TRANSLATION_AXIS else config.PICK_SEARCH_ANGLE_OFFSETS_DEG} "
            f"lift_offsets={config.PICK_SEARCH_LIFT_OFFSETS_MM if config.PICK_SEARCH_TRANSLATION_AXIS else []} "
            f"base={config.PICK_SEARCH_MAX_FLANGE_COORDS}"
        )
        if config.PICK_SEARCH_USE_FLANGE_POSE
        else (
            f"grid_joints={config.PICK_SEARCH_GRID_JOINTS} "
            f"grid_offsets={config.PICK_SEARCH_GRID_OFFSETS_DEG}"
            if config.PICK_SEARCH_GRID_ENABLED
            else f"joints={config.PICK_SEARCH_JOINTS} offsets={config.PICK_SEARCH_OFFSETS_DEG}"
        ),
    )

    try:
        for view_index, target in enumerate(targets[first_target_index:], start=1):
            if abort_event is not None and abort_event.is_set():
                wasab_arm_controller.stop_motion()
                status_callback("Pick search stopped")
                return latest_payload

            print(
                f"[PICK SEARCH] view {view_index}/{search_view_count}:",
                [round(value, 2) for value in target],
            )
            if config.PICK_SEARCH_USE_FLANGE_POSE:
                wasab_arm_controller.send_flange_coords(
                    target,
                    speed=config.PICK_SEARCH_SPEED,
                )
            else:
                wasab_arm_controller.send_joint_angles(
                    target,
                    speed=config.PICK_SEARCH_SPEED,
                    async_command=True,
                )
            if config.PICK_SEARCH_SETTLE_SEC > 0:
                if not _wait_or_abort(config.PICK_SEARCH_SETTLE_SEC, abort_event):
                    wasab_arm_controller.stop_motion()
                    status_callback("Pick search stopped")
                    return latest_payload

            status_callback(f"Pick search view {view_index}/{search_view_count}")
            payload, latest_frame_seq = request_pick_plan_from_current_view(
                wasab_arm_controller,
                get_frame,
                latest_frame_seq,
                config.PICK_SEARCH_FRAME_TIMEOUT_SEC,
                target_label=target_label,
            )
            latest_payload = payload
            if payload.get("status") == "ok":
                print("[PICK SEARCH] target found")
                status_callback(f"Pick search found target at view {view_index}")
                found_payload = True
                return payload
            if not is_no_target_detection_response(payload):
                return payload

        status_callback("Pick search finished: target not found")
        return latest_payload
    finally:
        if (
            config.PICK_SEARCH_RETURN_TO_START
            and not found_payload
            and (abort_event is None or not abort_event.is_set())
        ):
            if config.PICK_SEARCH_USE_FLANGE_POSE:
                wasab_arm_controller.send_flange_coords(
                    base_flange_coords,
                    speed=config.PICK_SEARCH_SPEED,
                )
            else:
                wasab_arm_controller.send_joint_angles(
                    base_angles,
                    speed=config.PICK_SEARCH_SPEED,
                    async_command=True,
                )


def draw_result(
    frame,
    payload: dict[str, Any] | None,
    error: str | None,
    marker_detection: dict[str, Any] | None = None,
    gesture_status: str | None = None,
) -> None:
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

    if marker_detection:
        marker_bbox = marker_detection.get("bbox")
        if isinstance(marker_bbox, list) and len(marker_bbox) == 4:
            x1, marker_y1, x2, marker_y2 = (
                int(round(float(value))) for value in marker_bbox
            )
            cv2.rectangle(frame, (x1, marker_y1), (x2, marker_y2), (0, 255, 0), 3)
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
    print(
        "Arm identity:", config.ARM_ID, f"role={config.ARM_ROLE}",
        f"dual={config.DUAL_ARM_ENABLED}", f"setup={config.ARM_SETUP_MODE}",
    )

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
    hand_gesture: OpenPalmTrigger | None = None
    if config.HAND_GESTURE_ENABLED:
        saved_hitbox = (
            load_palm_hitbox_norm()
            if config.PALM_HITBOX_CALIBRATION_ENABLED
            else None
        )
        hand_gesture = OpenPalmTrigger(
            stable_frames=config.HAND_GESTURE_STABLE_FRAMES,
            release_frames=config.HAND_GESTURE_RELEASE_FRAMES,
            cooldown_sec=config.HAND_GESTURE_COOLDOWN_SEC,
            min_detection_confidence=config.HAND_GESTURE_MIN_DETECTION_CONFIDENCE,
            min_tracking_confidence=config.HAND_GESTURE_MIN_TRACKING_CONFIDENCE,
            hold_sec=config.HAND_GESTURE_HOLD_SEC,
            min_palm_span_norm=config.HAND_GESTURE_MIN_PALM_SPAN_NORM,
            max_palm_span_norm=config.HAND_GESTURE_MAX_PALM_SPAN_NORM,
            edge_margin_norm=config.HAND_GESTURE_EDGE_MARGIN_NORM,
            min_palm_v_norm=config.HAND_GESTURE_MIN_PALM_V_NORM,
            hitbox=saved_hitbox,
        )
        print("[GESTURE] MediaPipe single-palm 3-second detection mode enabled")
        if saved_hitbox is not None:
            print("[PALM HITBOX] display overlay loaded:", saved_hitbox)
        if config.PALM_REFERENCE_ENABLED:
            print(
                "[GESTURE] palm reference:",
                f"uv={config.PALM_REFERENCE_PIXEL_UV}",
                f"flange={config.PALM_REFERENCE_FLANGE_COORDS}",
                f"reference_z={config.PALM_REFERENCE_Z_MM:.1f}mm",
            )

    if config.ARM_SETUP_MODE:
        print(
            "[SETUP MODE] Startup motion is disabled. Allowed: calibration, pose, home, "
            "servo release/focus, stop, exit."
        )
    elif config.DRY_RUN:
        print("[SAFETY] DRY_RUN=True: Startup home/open is skipped.")
    else:
        if config.HAND_GESTURE_ENABLED and config.GESTURE_HOME_ENABLED:
            print("[GESTURE] detection-only startup: robot motion is disabled")
        else:
            wasab_arm_controller.move_home_and_open_gripper()

    last_payload: dict[str, Any] | None = None
    last_payload_expires_at = 0.0
    last_marker_detection: dict[str, Any] | None = None
    last_marker_detection_expires_at = 0.0
    last_error: str | None = None
    gripper_closed_on_target = False
    last_pick_flange_command: list[float] | None = None
    last_pick_target_label: str | None = None
    last_pick_gripper_auto_rotated = False
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
    gesture_pick_cycle_requested = False
    gesture_detection_only = True
    auto_place_pending = False
    gift_restock_destination_command = load_gift_restock_destination()
    if gift_restock_destination_command is not None:
        print(
            "[GIFT SUPPLY] loaded original pickup destination:",
            gift_restock_destination_command,
        )
    last_gesture_process_at = 0.0
    last_recognized_palm_uv: tuple[float, float] | None = None
    last_marker_preview_at = 0.0
    gesture_runtime_enabled = config.HAND_GESTURE_START_ENABLED
    palm_hitbox_target_samples = load_palm_hitbox_target_samples()
    gesture_display_status = "SHOW ONE OPEN PALM" if gesture_runtime_enabled else "OFF"
    calibration_requested = False
    vision_sweep_enabled = False
    vision_sweep_center: list[float] | None = None
    vision_sweep_index = 0
    vision_sweep_next_at = 0.0
    vision_sweep_offsets: list[tuple[float, float]] = []
    vision_sweep_speed = 10
    vision_sweep_dwell_sec = 1.0
    vision_sweep_home: list[float] | None = None
    vision_sweep_yaw_limit = 90.0
    vision_sweep_pitch_limit = 50.0
    vision_track_mode = ""
    vision_track_speed = 10
    vision_track_timeout_sec = 1.0
    vision_track_deadzone = 0.05
    vision_track_kx = -8.0
    vision_track_ky = 6.0
    vision_track_yaw = 0.0
    vision_track_pitch = -15.0
    vision_last_target_at = 0.0
    vision_tracking = False

    def original_expanding_offsets(
        yaw_step: float,
        pitch_step: float,
        yaw_max: float,
        pitch_max: float,
    ) -> list[tuple[float, float]]:
        """Exact expanding-offset order from wasab_k3_mimic/sweep.py."""
        offsets = [(0.0, 0.0)]
        ring = 1
        while yaw_step * ring <= yaw_max or pitch_step * ring <= pitch_max:
            yaw = min(yaw_step * ring, yaw_max)
            pitch = min(pitch_step * ring, pitch_max)
            offsets.extend([
                (yaw, 0.0), (-yaw, 0.0), (0.0, pitch), (0.0, -pitch),
                (yaw, pitch), (-yaw, pitch), (yaw, -pitch), (-yaw, -pitch),
            ])
            ring += 1
        return offsets

    stop_lock = threading.Lock()

    def _set_last_error(message: str | None) -> None:
        nonlocal last_error
        with state_lock:
            last_error = message

    def clear_remote_command_queue() -> None:
        with remote_command_queue.mutex:
            remote_command_queue.queue.clear()

    def request_immediate_stop(source: str) -> None:
        nonlocal last_error
        with stop_lock:
            stop_request.set()
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
        stop_request.clear()
        clear_remote_command_queue()
        with state_lock:
            last_error = "STOP complete"

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

    def snapshot_overlay_state() -> tuple[
        dict[str, Any] | None,
        str | None,
        dict[str, Any] | None,
        str,
    ]:
        with state_lock:
            now = time.monotonic()
            payload = last_payload if now < last_payload_expires_at else None
            marker = last_marker_detection if now < last_marker_detection_expires_at else None
            return payload, last_error, marker, gesture_display_status

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

            payload_snapshot, error_snapshot, marker_snapshot, gesture_snapshot = snapshot_overlay_state()
            draw_result(
                stream_frame,
                payload_snapshot,
                error_snapshot,
                marker_snapshot,
                gesture_snapshot,
            )
            if gesture_runtime_enabled and hand_gesture is not None:
                stream_frame = hand_gesture.draw_skeleton(stream_frame)
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

    def workspace_overlay_worker() -> None:
        """Update overlay geometry without ever blocking camera frame delivery."""
        while not stream_stop.wait(1.0):
            try:
                update_workspace_overlay(
                    wasab_arm_controller.get_flange_coords()
                )
            except Exception as exc:
                if time.monotonic() - last_stream_error_at >= 5.0:
                    print("[WORKSPACE OVERLAY]", exc)

    stream_thread: threading.Thread | None = None
    workspace_thread: threading.Thread | None = None
    if config.CAMERA_STREAM_ENABLED:
        stream_thread = threading.Thread(target=streamer_upload_worker, daemon=True)
        stream_thread.start()
        if config.CAMERA_WORKSPACE_OVERLAY_ENABLED:
            workspace_thread = threading.Thread(
                target=workspace_overlay_worker,
                daemon=True,
            )
            workspace_thread.start()

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

    def operation_log_worker() -> None:
        last_reported: str | None = None
        post_robot_log("robot client connected")
        while not remote_stop.wait(0.5):
            with state_lock:
                current = last_error
            if current and current != last_reported:
                normalized = current.lower()
                level = (
                    "error"
                    if "error" in normalized
                    or any(
                        marker in normalized
                        for marker in (
                            "plan rejected",
                            "not detected",
                            "failed",
                            "timeout",
                            "aborted",
                        )
                    )
                    else "info"
                )
                post_robot_log(current, level=level)
                last_reported = current

    operation_thread = threading.Thread(target=operation_log_worker, daemon=True)
    operation_thread.start()

    print(
        "[READY] Robot client is running; waiting for camera frames and commands "
        "(g/pick, recycle, help, space/stop, w/home, x/exit)."
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

            payload_snapshot, error_snapshot, marker_snapshot, gesture_snapshot = snapshot_overlay_state()
            annotated = frame.copy()
            draw_result(
                annotated,
                payload_snapshot,
                error_snapshot,
                marker_snapshot,
                gesture_snapshot,
            )
            if gesture_runtime_enabled and hand_gesture is not None:
                annotated = hand_gesture.draw_skeleton(annotated)
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
                    if remote_command.startswith("palm-hitbox-target:"):
                        try:
                            requested_count = int(remote_command.split(":", 1)[1])
                        except ValueError:
                            requested_count = 0
                        if not 1 <= requested_count <= 1000:
                            with state_lock:
                                last_error = "Hitbox sample count must be within 1..1000"
                            continue
                        palm_hitbox_target_samples = requested_count
                        save_palm_hitbox_target_samples(requested_count)
                        with state_lock:
                            last_error = (
                                f"Palm hitbox target set to {requested_count} samples"
                            )
                        print(
                            "[PALM HITBOX] target samples updated:",
                            requested_count,
                        )
                        continue
                    if remote_command.startswith("vision-track:"):
                        parts = remote_command.split(":")
                        try:
                            _, target_feature, cx_text, cy_text, conf_text = parts
                            cx = float(cx_text)
                            cy = float(cy_text)
                            confidence = float(conf_text)
                        except (TypeError, ValueError):
                            with state_lock:
                                last_error = f"Invalid vision target: {remote_command}"
                            continue
                        if (
                            not vision_sweep_enabled
                            or target_feature != vision_track_mode
                            or confidence < 0.3
                        ):
                            continue
                        if target_feature == "fire-detect":
                            # wasab_통합 fire_search_node behavior: a confirmed
                            # flame stops SEARCH at the current pose.  Do not
                            # continuously servo J1/J4 toward the noisy flame
                            # centroid.  Refreshing this timestamp holds the
                            # pose; after face_timeout the normal SEARCH loop
                            # resumes around the last commanded position.
                            vision_last_target_at = time.monotonic()
                            vision_tracking = True
                            print(
                                "[VISION FIRE HOLD]",
                                f"pixel=({cx:.3f},{cy:.3f})",
                                f"confidence={confidence:.3f}",
                                "centering=disabled",
                            )
                            continue
                        ex = cx - 0.5
                        ey = cy - 0.5
                        if abs(ex) >= vision_track_deadzone:
                            vision_track_yaw = max(
                                -vision_sweep_yaw_limit,
                                min(
                                    vision_sweep_yaw_limit,
                                    vision_track_yaw - (vision_track_kx * ex),
                                ),
                            )
                        if abs(ey) >= vision_track_deadzone:
                            vision_track_pitch = max(
                                -vision_sweep_pitch_limit,
                                min(
                                    vision_sweep_pitch_limit,
                                    vision_track_pitch - (vision_track_ky * ey),
                                ),
                            )
                        target_angles = list(vision_sweep_home)
                        target_angles[0] = vision_track_yaw
                        target_angles[3] = vision_track_pitch
                        wasab_arm_controller.send_joint_angles(
                            target_angles,
                            speed=vision_track_speed,
                            async_command=True,
                        )
                        vision_last_target_at = time.monotonic()
                        vision_tracking = True
                        print(
                            "[VISION TRACK]",
                            f"mode={vision_track_mode}",
                            f"pixel=({cx:.3f},{cy:.3f})",
                            f"yaw={vision_track_yaw:.2f}",
                            f"pitch={vision_track_pitch:.2f}",
                        )
                        continue
                    mapped_key = remote_command_to_key(remote_command)
                    if mapped_key is None:
                        with state_lock:
                            last_error = f"Remote command ignored: {remote_command}"
                        continue
                    # Publish a state transition before executing a potentially
                    # blocking robot action.  Without this transition, two
                    # consecutive runs can finish with the same completion
                    # message and operation_log_worker may suppress the second
                    # one as a duplicate.  Gift Giving waits for that completion
                    # event before dispatching the next arm.
                    with state_lock:
                        last_error = f"Remote command started: {remote_command}"
                    key = mapped_key

            if config.ARM_SETUP_MODE:
                setup_allowed_keys = {
                    255, ord("c"), ord("p"), ord("s"), ord("k"), ord("w"), ord("x"), ord(" "), STOP_KEY,
                }
                if key not in setup_allowed_keys:
                    with state_lock:
                        last_error = (
                            "SETUP MODE blocked this command. Allowed: calibration, pose, home, "
                            "servo-release, servo-focus, stop, exit"
                        )
                    print("[SETUP MODE] blocked key:", key)
                    continue

            # Explicit local/remote commands take priority. Gesture mode only
            # recognizes a single open palm; it never schedules robot motion.
            if key == 255 and auto_place_pending:
                auto_place_pending = False
                key = ord("f")
                print("[PICK & PLACE] Pick complete; starting Place")
            elif (
                key == 255
                and gesture_runtime_enabled
                and hand_gesture is not None
                and not gripper_closed_on_target
            ):
                now = time.monotonic()
                if now - last_gesture_process_at >= 1.0 / config.HAND_GESTURE_PROCESS_FPS:
                    last_gesture_process_at = now
                    triggered, gesture_status = hand_gesture.process(frame)
                    with state_lock:
                        gesture_display_status = gesture_status
                    if triggered:
                        palm_center_norm = hand_gesture.get_palm_center_norm()
                        if palm_center_norm is not None:
                            last_recognized_palm_uv = (
                                palm_center_norm[0] * config.CAMERA_FRAME_WIDTH,
                                palm_center_norm[1] * config.CAMERA_FRAME_HEIGHT,
                            )
                            print(
                                "[PALM CHECK] saved place target:",
                                f"u={last_recognized_palm_uv[0]:.0f}",
                                f"v={last_recognized_palm_uv[1]:.0f}",
                            )
                        capture_hitbox = (
                            load_palm_hitbox_norm()
                            if config.PALM_HITBOX_CALIBRATION_ENABLED
                            else None
                        )
                        if capture_hitbox is not None:
                            hitbox_capture = save_colored_palm_hitbox_capture(
                                frame,
                                capture_hitbox,
                            )
                            try:
                                upload_palm_hitbox_capture(
                                    cv2.imread(str(hitbox_capture))
                                )
                                print(
                                    "[PALM HITBOX] 3-second capture saved:",
                                    hitbox_capture,
                                )
                            except WaSaBServiceError as exc:
                                print(
                                    "[PALM HITBOX] laptop capture upload failed:",
                                    exc,
                                )
                        if gesture_detection_only:
                            if (
                                config.PALM_HITBOX_CALIBRATION_ENABLED
                                and last_recognized_palm_uv is not None
                            ):
                                sample_count, result_image, recommended_box = (
                                    record_palm_hitbox_sample(
                                        last_recognized_palm_uv,
                                        frame,
                                        palm_hitbox_target_samples,
                                    )
                                )
                                print(
                                    "[PALM HITBOX]",
                                    f"sample {sample_count}/"
                                    f"{palm_hitbox_target_samples}",
                                )
                                if result_image is not None:
                                    print(
                                        "[PALM HITBOX] result image:",
                                        result_image,
                                        "recommended=",
                                        recommended_box,
                                    )
                                    if recommended_box is not None:
                                        hand_gesture.set_hitbox(
                                            (
                                                recommended_box[0]
                                                / config.CAMERA_FRAME_WIDTH,
                                                recommended_box[1]
                                                / config.CAMERA_FRAME_HEIGHT,
                                                recommended_box[2]
                                                / config.CAMERA_FRAME_WIDTH,
                                                recommended_box[3]
                                                / config.CAMERA_FRAME_HEIGHT,
                                            )
                                        )
                            print("[PALM CHECK] one palm recognized")
                            if (
                                config.ARM_ID == "right"
                                and config.HOME_JOINT_ANGLES is not None
                            ):
                                # Preserve the recognized palm coordinates, then
                                # clear the camera pose before Left Arm starts
                                # preparing the table.
                                hand_gesture.clear_skeleton()
                                gesture_runtime_enabled = False
                                print(
                                    "[PALM CHECK] recognized; returning Right Arm HOME:",
                                    config.HOME_JOINT_ANGLES,
                                )
                                wasab_arm_controller.send_joint_angles(
                                    config.HOME_JOINT_ANGLES,
                                    speed=config.MOVE_SPEED,
                                    async_command=True,
                                )
                                if not wasab_arm_controller.wait_until_joint_angles(
                                    config.HOME_JOINT_ANGLES,
                                    timeout_sec=config.MOVE_TIMEOUT_SEC,
                                    tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                    abort_event=stop_request,
                                ):
                                    with state_lock:
                                        last_error = (
                                            "Right HOME timeout after palm recognition"
                                    )
                                    print("[PALM CHECK]", last_error)
                                    continue
                                if not _wait_or_abort(
                                    config.HOME_SETTLE_SEC,
                                    stop_request,
                                ):
                                    continue
                                if config.HOME_SETTLE_SEC > 0:
                                    print(
                                        "[PALM CHECK] Right HOME camera settle:",
                                        f"{config.HOME_SETTLE_SEC:.1f}s",
                                    )
                                gesture_status = (
                                    "ONE PALM RECOGNIZED; right HOME ready"
                                )
                        else:
                            hand_gesture.clear_skeleton()
                            gesture_runtime_enabled = False
                            gesture_pick_cycle_requested = True
                            key = ord("g")
                        with state_lock:
                            last_error = gesture_status
                        print("[GESTURE]", gesture_status)

            if config.MARKER_PREVIEW_DETECTION_ENABLED:
                now = time.monotonic()
                if now - last_marker_preview_at >= 1.0 / config.MARKER_PREVIEW_DETECTION_FPS:
                    last_marker_preview_at = now
                    preview_marker = detect_april_marker(frame)
                    if preview_marker is not None:
                        with state_lock:
                            last_marker_detection = preview_marker
                            last_marker_detection_expires_at = now + max(
                                0.6,
                                2.0 / config.MARKER_PREVIEW_DETECTION_FPS,
                            )

            if key == GESTURE_OFF_KEY:
                gesture_runtime_enabled = False
                gesture_detection_only = True
                gesture_pick_cycle_requested = False
                auto_place_pending = False
                if hand_gesture is not None:
                    hand_gesture.clear_skeleton()
                with state_lock:
                    gesture_display_status = "OFF"
                    last_error = "Gesture recognition OFF"
                print("[GESTURE] recognition OFF")
                continue

            if key == VISION_SWEEP_OFF_KEY:
                vision_sweep_enabled = False
                vision_sweep_center = None
                vision_tracking = False
                vision_track_mode = ""
                with state_lock:
                    last_error = "Vision sweep OFF"
                print("[VISION SWEEP] OFF")
                continue

            if key in {
                VISION_SWEEP_ON_KEY,
                VISION_SWEEP_FACE_ON_KEY,
                VISION_SWEEP_FIRE_ON_KEY,
                VISION_SWEEP_TRACKING_ON_KEY,
            }:
                if config.DRY_RUN:
                    with state_lock:
                        last_error = "DRY RUN: Vision sweep not started"
                    continue
                # Canonical values from wasab_fire/wasab:
                # Both GUI features intentionally use the fire-search pose and
                # speed requested for this dual-arm deployment.
                if key == VISION_SWEEP_FACE_ON_KEY:
                    # wasab_통합 uses J6=-45, but the left-arm camera mount is
                    # rotated 90 degrees.  Apply the measured mount correction
                    # so the camera faces forward instead of sideways.
                    vision_sweep_home = [0.0, 0.0, 0.0, -15.0, 0.0, -45.0]
                    # 얼굴인식은 이동 중 모션 블러를 줄이고 각 위치에서 충분한
                    # 추론 프레임을 확보하도록 다른 비전 기능보다 천천히 탐색한다.
                    vision_sweep_speed = 15
                    vision_sweep_dwell_sec = 2.0
                    vision_track_mode = "face-recognition"
                    vision_track_speed = 25
                    vision_track_timeout_sec = 2.0
                    vision_track_deadzone = 0.0
                    vision_track_kx = -4.0
                    vision_track_ky = 3.0
                elif key == VISION_SWEEP_FIRE_ON_KEY:
                    # wasab_통합 run_fire_search.sh canonical overrides.
                    vision_sweep_home = [0.0, 0.0, 0.0, -15.0, 0.0, -45.0]
                    vision_sweep_speed = 10
                    vision_sweep_dwell_sec = 2.0
                    vision_track_mode = "fire-detect"
                    vision_track_speed = 10
                    vision_track_timeout_sec = 2.0
                    vision_track_deadzone = 0.05
                    vision_track_kx = -8.0
                    vision_track_ky = 6.0
                elif key == VISION_SWEEP_TRACKING_ON_KEY:
                    # Same left-camera mount correction as face recognition.
                    vision_sweep_home = [0.0, 0.0, 0.0, -15.0, 0.0, -45.0]
                    vision_sweep_speed = 30
                    vision_sweep_dwell_sec = 1.0
                    vision_track_mode = "tracking"
                    vision_track_speed = 25
                    vision_track_timeout_sec = 2.0
                    vision_track_deadzone = 0.0
                    vision_track_kx = -4.0
                    vision_track_ky = 3.0
                else:
                    vision_sweep_home = wasab_arm_controller.get_joint_angles()
                    vision_sweep_speed = 10
                    vision_sweep_dwell_sec = 1.0
                    vision_track_mode = ""
                vision_sweep_center = list(vision_sweep_home)
                vision_track_yaw = vision_sweep_center[0]
                vision_track_pitch = vision_sweep_center[3]
                vision_last_target_at = 0.0
                vision_tracking = False
                vision_sweep_offsets = original_expanding_offsets(
                    14.0, 6.0, vision_sweep_yaw_limit, vision_sweep_pitch_limit
                )
                vision_sweep_enabled = True
                vision_sweep_index = 0
                vision_sweep_next_at = 0.0
                with state_lock:
                    last_error = "Vision sweep ON"
                mode = (
                    "face" if key == VISION_SWEEP_FACE_ON_KEY
                    else "fire" if key == VISION_SWEEP_FIRE_ON_KEY
                    else "tracking" if key == VISION_SWEEP_TRACKING_ON_KEY
                    else "legacy"
                )
                print(
                    "[VISION SWEEP] ON",
                    f"mode={mode}",
                    f"step=(14,6) limits=(90,50) dwell={vision_sweep_dwell_sec:.1f}",
                    f"speed={vision_sweep_speed}",
                    "center:",
                    vision_sweep_center,
                )
                continue

            if key == GESTURE_ON_KEY:
                if hand_gesture is None:
                    with state_lock:
                        last_error = "Gesture ON rejected: [hand_gesture] enabled=false"
                    continue
                gesture_runtime_enabled = True
                gesture_detection_only = False
                with state_lock:
                    gesture_display_status = "SHOW ONE OPEN PALM"
                    last_error = "Gesture recognition ON; moving to recognition pose"
                if (
                    config.ARM_ID == "right"
                    and not config.DRY_RUN
                    and config.GESTURE_HOME_ENABLED
                ):
                    reached = wasab_arm_controller.move_gesture_home(
                        abort_event=stop_request,
                    )
                    with state_lock:
                        last_error = (
                            "Gesture recognition ready"
                            if reached
                            else "Gesture recognition pose timeout"
                        )
                print("[GESTURE] recognition ON; pick/place starts after recognition")
                continue

            if key == PALM_CHECK_KEY:
                if hand_gesture is None:
                    with state_lock:
                        last_error = "Palm check rejected: [hand_gesture] enabled=false"
                    continue
                gesture_runtime_enabled = True
                gesture_detection_only = True
                with state_lock:
                    gesture_display_status = "PALM CHECK: SHOW ONE OPEN PALM"
                    last_error = "Palm check armed; robot motion is disabled"
                print("[PALM CHECK] armed; detection only")
                continue

            if key in {STOP_KEY, ord(" ")}:
                vision_sweep_enabled = False
                vision_sweep_center = None
                gesture_pick_cycle_requested = False
                auto_place_pending = False
                request_immediate_stop("local" if key == ord(" ") else "remote")
                finish_stop_request()
                continue

            if stop_request.is_set():
                vision_sweep_enabled = False
                vision_sweep_center = None
                vision_tracking = False
                vision_track_mode = ""
                gesture_pick_cycle_requested = False
                auto_place_pending = False
                finish_stop_request()
                continue

            if key == ord("p"):
                try:
                    coords = wasab_arm_controller.get_flange_coords()
                    angles = wasab_arm_controller.get_joint_angles()
                    text = (
                        "POSE Flange: "
                        f"x={coords[0]:.1f}, y={coords[1]:.1f}, z={coords[2]:.1f}, "
                        f"rx={coords[3]:.2f}, ry={coords[4]:.2f}, rz={coords[5]:.2f}; "
                        f"angles={[round(value, 2) for value in angles]}"
                    )
                    print("[POSE]", coords)
                    print("[ANGLES]", [round(value, 2) for value in angles])
                    with state_lock:
                        last_error = text
                except Exception as exc:
                    with state_lock:
                        last_error = f"POSE ERROR: {type(exc).__name__}: {exc}"
                    print(last_error)
                continue

            if key == ord("q"):
                if config.DRY_RUN:
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

            if key in {FIRE_SUPPRESS_CLOSE_KEY, FIRE_SUPPRESS_OPEN_KEY}:
                close_requested = key == FIRE_SUPPRESS_CLOSE_KEY
                action = "closed" if close_requested else "opened"
                if config.DRY_RUN:
                    with state_lock:
                        last_error = f"DRY RUN: fire suppression gripper {action}"
                else:
                    try:
                        if close_requested:
                            # Match the integrated fire_search_node suppression
                            # sequence exactly: close/open twice, then stay closed.
                            for _ in range(2):
                                wasab_arm_controller.set_gripper_value(
                                    0, "fire suppress close", settle=False, speed=50
                                )
                                time.sleep(0.4)
                                wasab_arm_controller.set_gripper_value(
                                    100, "fire suppress open", settle=False, speed=50
                                )
                                time.sleep(0.4)
                            wasab_arm_controller.set_gripper_value(
                                0, "fire suppress final close", settle=False, speed=50
                            )
                        else:
                            wasab_arm_controller.set_gripper_value(
                                100, "fire suppress open", settle=False, speed=50
                            )
                        gripper_closed_on_target = close_requested
                        with state_lock:
                            last_error = f"Fire suppression: gripper {action}"
                        print(f"[FIRE SUPPRESS] gripper {action}")
                    except RuntimeError as exc:
                        with state_lock:
                            last_error = f"FIRE SUPPRESS GRIPPER ERROR: {exc}"
                        print(last_error)
                continue

            if key == ord("x"):
                request_immediate_stop("exit")
                break

            if key == ord("c"):
                calibration_requested = True
                with state_lock:
                    last_error = "Calibration requested; releasing camera and robot client"
                post_robot_log(last_error, source="calibration")
                break

            if key == ord("s"):
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
                if config.DRY_RUN:
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
                if not gripper_closed_on_target and not config.DRY_RUN:
                    actual_gripper_open = wasab_arm_controller.is_gripper_open()
                    if actual_gripper_open is False:
                        gripper_closed_on_target = True
                        with state_lock:
                            last_error = "Place state recovered: gripper is closed"
                        print("[PLACE] state recovered: gripper is closed")
                    elif actual_gripper_open is None:
                        with state_lock:
                            last_error = "Place requested: gripper state unknown; continuing"
                        print("[PLACE] gripper state unknown; continuing with AprilTag place")
                    else:
                        with state_lock:
                            last_error = "Place requested: gripper appears open; continuing"
                        print("[PLACE] gripper appears open; continuing with AprilTag place")

                if not config.PLACE_MOTION_ENABLED:
                    with state_lock:
                        last_error = "Place ignored: place motion is disabled"
                    print("[PLACE]", last_error)
                elif (
                    config.ARM_ID == "right"
                    and config.PALM_REFERENCE_ENABLED
                    and config.PALM_REFERENCE_FLANGE_COORDS is not None
                    and last_recognized_palm_uv is not None
                ):
                    try:
                        print("[PALM PLACE] pickup -> HOME -> palm approach")
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request
                        ):
                            with state_lock:
                                last_error = "Palm place aborted: HOME timeout"
                            continue
                        palm_place = list(config.PALM_REFERENCE_FLANGE_COORDS)
                        print(
                            "[PALM PLACE] recognized uv:",
                            [
                                round(last_recognized_palm_uv[0], 1),
                                round(last_recognized_palm_uv[1], 1),
                            ],
                        )
                        if (
                            config.PALM_REFERENCE_JOINT_ANGLES is None
                            or config.HOME_JOINT_ANGLES is None
                        ):
                            with state_lock:
                                last_error = (
                                    "Palm place requires measured HOME and Place joint angles"
                                )
                            continue
                        fraction = config.PALM_PLACE_APPROACH_JOINT_FRACTION
                        palm_approach_angles = [
                            round(home + (final - home) * fraction, 2)
                            for home, final in zip(
                                config.HOME_JOINT_ANGLES,
                                config.PALM_REFERENCE_JOINT_ANGLES,
                            )
                        ]
                        print(
                            "[PALM PLACE] joint approach:",
                            palm_approach_angles,
                        )
                        wasab_arm_controller.send_joint_angles(
                            palm_approach_angles,
                            speed=10,
                            async_command=True,
                        )
                        if not wasab_arm_controller.wait_until_joint_angles(
                            palm_approach_angles,
                            timeout_sec=config.MOVE_TIMEOUT_SEC,
                            tolerance_deg=max(config.POSE_ANGLE_TOL_DEG, 3.0),
                            abort_event=stop_request,
                        ):
                            with state_lock:
                                last_error = "Palm place joint approach timeout"
                            continue

                        print(
                            "[PALM PLACE] measured final joints:",
                            config.PALM_REFERENCE_JOINT_ANGLES,
                        )
                        wasab_arm_controller.send_joint_angles(
                            config.PALM_REFERENCE_JOINT_ANGLES,
                            speed=8,
                            async_command=True,
                        )
                        palm_reached = wasab_arm_controller.wait_until_joint_angles(
                            config.PALM_REFERENCE_JOINT_ANGLES,
                            timeout_sec=config.MOVE_TIMEOUT_SEC,
                            tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                            abort_event=stop_request,
                        )
                        if not palm_reached:
                            with state_lock:
                                last_error = "Palm place final pose timeout"
                            continue

                        if config.PALM_RELEASE_CONFIRMATION_ENABLED:
                            print(
                                "[PALM PLACE] final pose reached; waiting for an open "
                                "palm before release"
                            )
                            confirmation_deadline = (
                                time.monotonic()
                                + config.PALM_RELEASE_CONFIRMATION_TIMEOUT_SEC
                            )
                            confirmation_started_at: float | None = None
                            confirmation_seq: int | None = None
                            confirmed_palm_uv: tuple[float, float] | None = None
                            while (
                                not stop_request.is_set()
                                and time.monotonic() < confirmation_deadline
                            ):
                                confirmation_frame, confirmation_seq = get_latest_frame(
                                    min_seq=confirmation_seq,
                                    timeout_sec=0.5,
                                )
                                if confirmation_frame is None:
                                    confirmation_started_at = None
                                    continue
                                if config.PALM_RELEASE_CONFIRMATION_MODE == "hand":
                                    valid_palm, palm_center_norm, guidance = (
                                        hand_gesture.detect_hand_presence(
                                            confirmation_frame
                                        )
                                    )
                                else:
                                    valid_palm, palm_center_norm, guidance = (
                                        hand_gesture.detect_valid_open_palm(
                                            confirmation_frame
                                        )
                                    )
                                now = time.monotonic()
                                if valid_palm and palm_center_norm is not None:
                                    if confirmation_started_at is None:
                                        confirmation_started_at = now
                                    confirmed_palm_uv = (
                                        palm_center_norm[0]
                                        * config.CAMERA_FRAME_WIDTH,
                                        palm_center_norm[1]
                                        * config.CAMERA_FRAME_HEIGHT,
                                    )
                                    held_sec = now - confirmation_started_at
                                    with state_lock:
                                        gesture_display_status = (
                                            "HAND "
                                            f"{held_sec:.1f}/"
                                            f"{config.PALM_RELEASE_CONFIRMATION_HOLD_SEC:.1f}s"
                                        )
                                        last_error = (
                                            "Hand found near gripper; hold still"
                                        )
                                    if (
                                        held_sec
                                        >= config.PALM_RELEASE_CONFIRMATION_HOLD_SEC
                                    ):
                                        break
                                else:
                                    confirmation_started_at = None
                                    confirmed_palm_uv = None
                                    with state_lock:
                                        gesture_display_status = guidance
                                        last_error = (
                                            "Move one hand under the gripper"
                                        )
                            else:
                                confirmed_palm_uv = None

                            if confirmed_palm_uv is None:
                                with state_lock:
                                    gesture_display_status = "PALM NOT CONFIRMED"
                                    last_error = (
                                        "Palm release confirmation timeout; object "
                                        "remains held. Move one hand closer and "
                                        "press Place again"
                                    )
                                print("[PALM PLACE]", last_error)
                                continue
                            last_recognized_palm_uv = confirmed_palm_uv
                            print(
                                "[PALM PLACE] release palm confirmed:",
                                [
                                    round(confirmed_palm_uv[0], 1),
                                    round(confirmed_palm_uv[1], 1),
                                ],
                            )

                        if config.PLACE_RELEASE_PAUSE_SEC > 0:
                            time.sleep(config.PLACE_RELEASE_PAUSE_SEC)
                        wasab_arm_controller.open_gripper(
                            speed=config.PLACE_GRIPPER_OPEN_SPEED,
                            settle_sec=config.PLACE_GRIPPER_SETTLE_SEC,
                        )
                        gripper_closed_on_target = False
                        last_pick_flange_command = None
                        last_pick_target_label = None
                        last_pick_gripper_auto_rotated = False
                        print(
                            "[PALM PLACE] released; reverse to joint approach:",
                            palm_approach_angles,
                        )
                        wasab_arm_controller.send_joint_angles(
                            palm_approach_angles,
                            speed=8,
                            async_command=True,
                        )
                        retreat_reached = wasab_arm_controller.wait_until_joint_angles(
                            palm_approach_angles,
                            timeout_sec=config.MOVE_TIMEOUT_SEC,
                            tolerance_deg=max(config.POSE_ANGLE_TOL_DEG, 3.0),
                            abort_event=stop_request,
                        )
                        if not retreat_reached:
                            with state_lock:
                                last_error = (
                                    "Palm place released; reverse approach timeout"
                                )
                            continue
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request
                        ):
                            with state_lock:
                                last_error = "Palm place released; HOME return timeout"
                            continue
                        with state_lock:
                            last_error = "Palm place complete; HOME ready"
                        post_robot_log(
                            "Palm place complete; HOME ready",
                        )
                    except Exception as exc:
                        with state_lock:
                            last_error = f"PALM PLACE ERROR: {type(exc).__name__}: {exc}"
                        print(last_error)
                elif config.ARM_ID == "right" and config.PALM_REFERENCE_ENABLED:
                    with state_lock:
                        last_error = (
                            "Palm place rejected: recognize one palm for 3 seconds first"
                        )
                    print("[PALM PLACE]", last_error)
                elif not config.MARKER_SEARCH_ENABLED:
                    with state_lock:
                        last_error = "Place ignored: marker search is disabled"
                    print("[PLACE]", last_error)
                elif config.DRY_RUN:
                    frame_snapshot, _ = get_latest_frame(timeout_sec=0.5)
                    detection = (
                        detect_april_marker(frame_snapshot, allowed_ids=config.MARKER_PLACE_IDS)
                        if frame_snapshot is not None
                        else None
                    )
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
                        print("[PLACE] home -> marker-view rotate -> find marker -> marker place -> open gripper")
                        home_reached = wasab_arm_controller.move_home_keep_gripper_closed(abort_event=stop_request)
                        if not home_reached:
                            with state_lock:
                                last_error = "Place aborted: HOME return timeout"
                            continue

                        black_table_place = (
                            config.BLACK_TABLE_ENABLED
                            and (
                                last_pick_target_label == "april_tag_0_box"
                                or str(last_pick_target_label).lower() == "coca-cola"
                            )
                        )

                        if black_table_place and config.BLACK_TABLE_FIXED_PLACE_ENABLED:
                            # The fixed place starts from HOME. Save the exact
                            # departure joints so the post-release motion can
                            # replay the same joint-space path in reverse.
                            place_departure_angles = (
                                wasab_arm_controller.get_joint_angles()
                            )
                            fixed_place = list(config.BLACK_TABLE_FIXED_PLACE_FLANGE_COORDS)
                            is_safe, reason, fixed_place = validate_server_plan(
                                {
                                    "status": "ok",
                                    "plan": {"flange_command": fixed_place},
                                }
                            )
                            if not is_safe or fixed_place is None:
                                with state_lock:
                                    last_error = f"Fixed black-table place rejected: {reason}"
                                print("[SAFETY]", last_error)
                                continue

                            print("[BLACK TABLE] direct fixed place:", fixed_place)
                            fixed_place_approach = list(fixed_place)
                            fixed_place_approach[2] = round(
                                fixed_place_approach[2]
                                + config.BLACK_TABLE_APPROACH_LIFT_Z_MM,
                                2,
                            )
                            approach_safe, approach_reason, fixed_place_approach = (
                                validate_server_plan(
                                    {
                                        "status": "ok",
                                        "plan": {
                                            "flange_command": fixed_place_approach
                                        },
                                    }
                                )
                            )
                            if not approach_safe or fixed_place_approach is None:
                                with state_lock:
                                    last_error = (
                                        "Fixed black-table approach rejected: "
                                        f"{approach_reason}"
                                    )
                                print("[SAFETY]", last_error)
                                continue
                            print(
                                "[BLACK TABLE] gentle approach:",
                                fixed_place_approach,
                            )
                            joint_direct_place = bool(
                                config.BLACK_TABLE_FINAL_USE_JOINT_ANGLES
                                and config.BLACK_TABLE_FINAL_JOINT_ANGLES
                                is not None
                            )
                            approach_reached = joint_direct_place
                            approach_failure_reason: str | None = None
                            if joint_direct_place:
                                print(
                                    "[BLACK TABLE] skip Cartesian approach; "
                                    "use measured final joints directly"
                                )
                            for approach_attempt in range(
                                0
                                if joint_direct_place
                                else (
                                    config.BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_COUNT
                                    + 1
                                )
                            ):
                                approach_reached = (
                                    wasab_arm_controller.send_flange_coords_and_wait(
                                        fixed_place_approach,
                                        speed=config.BLACK_TABLE_FIXED_PLACE_SPEED,
                                        abort_event=stop_request,
                                        position_tolerance_mm=(
                                            config.BLACK_TABLE_FIXED_PLACE_APPROACH_TOL_MM
                                        ),
                                    )
                                )
                                if approach_reached:
                                    break
                                approach_failure_reason = (
                                    wasab_arm_controller.last_wait_timeout_reason
                                )
                                if (
                                    approach_attempt
                                    >= config.BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_COUNT
                                    or stop_request.is_set()
                                ):
                                    break
                                with state_lock:
                                    last_error = (
                                        "Fixed black-table approach timeout; "
                                        f"retrying {approach_attempt + 1}/"
                                        f"{config.BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_COUNT}"
                                    )
                                print(
                                    "[BLACK TABLE] approach failed; return HOME "
                                    "with gripper closed before retry:",
                                    approach_failure_reason,
                                )
                                if not wasab_arm_controller.move_home_keep_gripper_closed(
                                    abort_event=stop_request
                                ):
                                    approach_failure_reason = (
                                        "HOME recovery failed before retry"
                                    )
                                    break
                                if not _wait_or_abort(
                                    config.BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_INTERVAL_SEC,
                                    stop_request,
                                ):
                                    break
                                print(
                                    "[BLACK TABLE] retry fixed approach:",
                                    f"{approach_attempt + 1}/"
                                    f"{config.BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_COUNT}",
                                )
                            if not approach_reached:
                                with state_lock:
                                    last_error = (
                                        "Fixed black-table approach timeout after "
                                        f"{config.BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_COUNT + 1} "
                                        "attempt(s); object remains held"
                                        + (
                                            f": {approach_failure_reason}"
                                            if approach_failure_reason
                                            else ""
                                        )
                                    )
                                continue
                            if (
                                config.BLACK_TABLE_FINAL_USE_JOINT_ANGLES
                                and config.BLACK_TABLE_FINAL_JOINT_ANGLES is not None
                            ):
                                print(
                                    "[BLACK TABLE] direct fixed place joints:",
                                    config.BLACK_TABLE_FINAL_JOINT_ANGLES,
                                )
                                wasab_arm_controller.send_joint_angles(
                                    config.BLACK_TABLE_FINAL_JOINT_ANGLES,
                                    speed=config.BLACK_TABLE_FIXED_PLACE_FINAL_SPEED,
                                    async_command=True,
                                )
                                fixed_place_reached = (
                                    wasab_arm_controller.wait_until_joint_angles(
                                        config.BLACK_TABLE_FINAL_JOINT_ANGLES,
                                        timeout_sec=config.MOVE_TIMEOUT_SEC,
                                        tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                        abort_event=stop_request,
                                    )
                                )
                            else:
                                fixed_place_reached = (
                                    wasab_arm_controller.send_flange_coords_and_wait(
                                        fixed_place,
                                        speed=config.BLACK_TABLE_FIXED_PLACE_FINAL_SPEED,
                                        mode=1,
                                        abort_event=stop_request,
                                    )
                                )
                            if not fixed_place_reached:
                                reason = wasab_arm_controller.last_wait_timeout_reason
                                with state_lock:
                                    last_error = (
                                        "Fixed black-table place timeout"
                                        + (f": {reason}" if reason else "")
                                    )
                                continue
                            if config.PLACE_RELEASE_PAUSE_SEC > 0:
                                time.sleep(config.PLACE_RELEASE_PAUSE_SEC)
                            wasab_arm_controller.open_gripper(
                                speed=config.PLACE_GRIPPER_OPEN_SPEED,
                                settle_sec=config.PLACE_GRIPPER_SETTLE_SEC,
                            )
                            gripper_closed_on_target = False
                            last_pick_flange_command = None
                            last_pick_target_label = None
                            last_pick_gripper_auto_rotated = False

                            retreat_reached = joint_direct_place
                            if joint_direct_place:
                                print(
                                    "[BLACK TABLE] skip Cartesian retreat; "
                                    "reverse measured joint path to HOME"
                                )
                            else:
                                print(
                                    "[BLACK TABLE] gentle vertical retreat:",
                                    fixed_place_approach,
                                )
                                retreat_reached = (
                                    wasab_arm_controller.send_flange_coords_and_wait(
                                        fixed_place_approach,
                                        speed=(
                                            config.BLACK_TABLE_FIXED_PLACE_FINAL_SPEED
                                        ),
                                        mode=1,
                                        abort_event=stop_request,
                                        position_tolerance_mm=(
                                            config.BLACK_TABLE_FIXED_PLACE_APPROACH_TOL_MM
                                        ),
                                    )
                                )
                            if not retreat_reached and not stop_request.is_set():
                                try:
                                    current_retreat_pose = (
                                        wasab_arm_controller.get_flange_coords()
                                    )
                                except Exception as exc:
                                    current_retreat_pose = []
                                    print(
                                        "[BLACK TABLE] retreat pose read failed:",
                                        exc,
                                    )
                                minimum_safe_retreat_z = (
                                    fixed_place_approach[2]
                                    - config.BLACK_TABLE_FIXED_PLACE_RETREAT_Z_TOL_MM
                                )
                                if (
                                    len(current_retreat_pose) >= 3
                                    and current_retreat_pose[2]
                                    >= minimum_safe_retreat_z
                                ):
                                    retreat_reached = True
                                    retreat_warning = (
                                        "Vertical retreat safe Z reached; "
                                        "continuing reverse HOME path despite "
                                        "residual XY pose error"
                                    )
                                    print(
                                        "[BLACK TABLE]",
                                        retreat_warning,
                                        f"target_z={fixed_place_approach[2]:.2f}",
                                        f"current_z={current_retreat_pose[2]:.2f}",
                                    )
                                    post_robot_log(
                                        retreat_warning,
                                        level="warning",
                                    )
                            if not retreat_reached:
                                with state_lock:
                                    last_error = (
                                        "Object released, but vertical retreat timed out"
                                    )
                                continue

                            print(
                                "[BLACK TABLE] reverse place path to HOME joints:",
                                place_departure_angles,
                            )
                            wasab_arm_controller.send_joint_angles(
                                place_departure_angles,
                                speed=config.BLACK_TABLE_FIXED_PLACE_SPEED,
                                async_command=True,
                            )
                            if not wasab_arm_controller.wait_until_joint_angles(
                                place_departure_angles,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                abort_event=stop_request,
                            ):
                                with state_lock:
                                    last_error = (
                                        "Object released, but reverse path to HOME timed out"
                                    )
                                continue

                            completion_message = (
                                "Fixed black-table place complete; "
                                "reverse path returned HOME"
                            )
                            with state_lock:
                                last_error = completion_message
                            # This event advances the Dual Arm state machine.
                            # Send it synchronously at the successful operation
                            # boundary instead of relying on polling/dedup.
                            post_robot_log(
                                completion_message,
                            )
                            continue

                        if (
                            not black_table_place
                            and config.PLACE_MARKER_VIEW_ROTATE_ENABLED
                            and last_pick_gripper_auto_rotated
                        ):
                            try:
                                marker_view_angles = make_place_marker_view_joint_angles(
                                    wasab_arm_controller.get_joint_angles()
                                )
                            except ValueError as exc:
                                with state_lock:
                                    last_error = f"Place marker-view pose rejected locally: {exc}"
                                print("[SAFETY]", last_error)
                                continue

                            with state_lock:
                                last_error = (
                                    "Place marker-view rotate: "
                                    f"RZ +{config.PLACE_MARKER_VIEW_RZ_OFFSET_DEG:.1f} deg"
                                )
                            wasab_arm_controller.send_joint_angles(
                                marker_view_angles,
                                speed=config.PLACE_MARKER_VIEW_SPEED,
                                async_command=True,
                            )
                            marker_view_reached = wasab_arm_controller.wait_until_joint_angles(
                                marker_view_angles,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                abort_event=stop_request,
                            )
                            if not marker_view_reached:
                                with state_lock:
                                    last_error = "Place marker-view rotate timeout"
                                continue
                        elif config.PLACE_MARKER_VIEW_ROTATE_ENABLED:
                            print("[PLACE] marker-view rotate skipped: last pick did not auto-rotate gripper")

                        if black_table_place:
                            if config.BLACK_TABLE_SEARCH_MOVE_TO_START_ENABLED:
                                print(
                                    "[BLACK TABLE] move to search neighborhood:",
                                    config.BLACK_TABLE_SEARCH_START_JOINT_ANGLES,
                                )
                                wasab_arm_controller.send_joint_angles(
                                    config.BLACK_TABLE_SEARCH_START_JOINT_ANGLES,
                                    speed=config.BLACK_TABLE_SEARCH_START_SPEED,
                                    async_command=True,
                                )
                                if not wasab_arm_controller.wait_until_joint_angles(
                                    config.BLACK_TABLE_SEARCH_START_JOINT_ANGLES,
                                    timeout_sec=config.MOVE_TIMEOUT_SEC,
                                    tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                    abort_event=stop_request,
                                ):
                                    with state_lock:
                                        last_error = "Black-table search start pose timeout"
                                    continue
                            success, message, marker_detection = execute_black_table_search(
                                wasab_arm_controller,
                                get_latest_frame,
                                set_place_marker_status,
                                stop_request,
                            )
                        else:
                            success, message, marker_detection = execute_marker_search(
                                wasab_arm_controller,
                                get_latest_frame,
                                set_place_marker_status,
                                stop_request,
                                allowed_ids=config.MARKER_PLACE_IDS,
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
                            last_pick_target_label,
                        )
                        print(
                            "[MARKER PLACE RESPONSE]\n",
                            json.dumps(payload, ensure_ascii=False, indent=2),
                        )
                        with state_lock:
                            last_payload = payload
                            last_payload_expires_at = time.monotonic() + 3.0

                        if black_table_place:
                            plan = payload.get("plan")
                            if isinstance(plan, dict):
                                flange_command = plan.get("flange_command")
                                if isinstance(flange_command, list) and len(flange_command) == 6:
                                    flange_command[:] = (
                                        config.BLACK_TABLE_FIXED_PLACE_FLANGE_COORDS
                                    )
                                    print(
                                        "[BLACK TABLE] detected; operator-verified final pose:",
                                        flange_command,
                                    )

                        is_safe, reason, safe_command = validate_server_plan(payload)
                        if not is_safe or safe_command is None:
                            with state_lock:
                                last_error = f"Marker place pose rejected locally: {reason}"
                            print("[SAFETY]", last_error)
                            continue

                        if black_table_place:
                            table_approach = list(safe_command)
                            table_approach[2] = round(
                                table_approach[2]
                                + config.BLACK_TABLE_APPROACH_LIFT_Z_MM,
                                2,
                            )
                            is_safe, reason, table_approach = validate_server_plan(
                                {
                                    "status": "ok",
                                    "plan": {"flange_command": table_approach},
                                }
                            )
                            if not is_safe or table_approach is None:
                                with state_lock:
                                    last_error = (
                                        f"Black-table approach rejected locally: {reason}"
                                    )
                                print("[SAFETY]", last_error)
                                continue
                            print(
                                "[BLACK TABLE] center approach:",
                                f"detection={marker_detection}",
                                f"approach={table_approach}",
                                f"place={safe_command}",
                            )
                            if not wasab_arm_controller.send_flange_coords_and_wait(
                                table_approach,
                                speed=config.PLACE_APPROACH_SPEED,
                                abort_event=stop_request,
                            ):
                                with state_lock:
                                    last_error = "Black-table center approach timeout"
                                continue

                        if (
                            black_table_place
                            and config.BLACK_TABLE_FINAL_USE_JOINT_ANGLES
                            and config.BLACK_TABLE_FINAL_JOINT_ANGLES is not None
                        ):
                            print(
                                "[BLACK TABLE] final joint pose:",
                                config.BLACK_TABLE_FINAL_JOINT_ANGLES,
                            )
                            wasab_arm_controller.send_joint_angles(
                                config.BLACK_TABLE_FINAL_JOINT_ANGLES,
                                speed=config.BLACK_TABLE_FIXED_PLACE_SPEED,
                                async_command=True,
                            )
                            place_reached = wasab_arm_controller.wait_until_joint_angles(
                                config.BLACK_TABLE_FINAL_JOINT_ANGLES,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                abort_event=stop_request,
                            )
                        else:
                            place_reached = execute_place_final_approach(
                                wasab_arm_controller,
                                safe_command,
                                abort_event=stop_request,
                            )
                        if not place_reached:
                            reason = wasab_arm_controller.last_wait_timeout_reason
                            with state_lock:
                                last_error = (
                                    "Marker place pose timeout"
                                    + (f": {reason}" if reason else "")
                                    + "; gripper remains closed"
                                )
                            continue

                        if config.PLACE_RELEASE_PAUSE_SEC > 0:
                            time.sleep(config.PLACE_RELEASE_PAUSE_SEC)
                        wasab_arm_controller.open_gripper(
                            speed=config.PLACE_GRIPPER_OPEN_SPEED,
                            settle_sec=config.PLACE_GRIPPER_SETTLE_SEC,
                        )
                        gripper_closed_on_target = False
                        last_pick_flange_command = None
                        last_pick_target_label = None
                        last_pick_gripper_auto_rotated = False
                        if config.GESTURE_HOME_ENABLED:
                            with state_lock:
                                last_error = "Place complete; returning to gesture home"
                            wasab_arm_controller.move_gesture_home(
                                abort_event=stop_request,
                            )
                        with state_lock:
                            last_error = "Place complete; gesture home ready"
                    except Exception as exc:
                        with state_lock:
                            last_error = f"PLACE ERROR: {type(exc).__name__}: {exc}"
                        print(last_error)
                continue

            if key == ord("m"):
                if not config.MANUAL_MOTION_ENABLED:
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

            if key == ord("w"):
                vision_sweep_enabled = False
                vision_sweep_center = None
                if config.DRY_RUN:
                    with state_lock:
                        last_error = "DRY RUN: Home command not sent"
                else:
                    reached = wasab_arm_controller.move_home_keep_gripper_closed(abort_event=stop_request)
                    gripper_closed_on_target = False
                    with state_lock:
                        last_error = None if reached else "HOME return timeout"
                continue

            if key == 255 and vision_sweep_enabled and vision_sweep_center is not None:
                now = time.monotonic()
                if (
                    vision_tracking
                    and now - vision_last_target_at <= vision_track_timeout_sec
                ):
                    continue
                if vision_tracking:
                    # Original SEARCH resumes around the last tracked pose.
                    vision_sweep_center[0] = vision_track_yaw
                    vision_sweep_center[3] = vision_track_pitch
                    vision_sweep_index = 0
                    vision_sweep_next_at = 0.0
                    vision_tracking = False
                    print(
                        "[VISION SEARCH] target lost; resume around",
                        f"yaw={vision_track_yaw:.2f}",
                        f"pitch={vision_track_pitch:.2f}",
                    )
                if now >= vision_sweep_next_at:
                    yaw_offset, pitch_offset = vision_sweep_offsets[vision_sweep_index]
                    target_angles = list(vision_sweep_center)
                    target_angles[0] = max(
                        -vision_sweep_yaw_limit,
                        min(vision_sweep_yaw_limit, target_angles[0] + yaw_offset),
                    )
                    target_angles[3] = max(
                        -vision_sweep_pitch_limit,
                        min(vision_sweep_pitch_limit, target_angles[3] + pitch_offset),
                    )
                    try:
                        wasab_arm_controller.send_joint_angles(
                            target_angles,
                            speed=vision_sweep_speed,
                            async_command=True,
                        )
                    except Exception as exc:
                        vision_sweep_enabled = False
                        vision_sweep_center = None
                        with state_lock:
                            last_error = f"Vision sweep stopped: {exc}"
                    else:
                        print(
                            "[VISION SWEEP]",
                            f"step={vision_sweep_index}",
                            f"yaw_offset={yaw_offset}",
                            f"pitch_offset={pitch_offset}",
                        )
                        vision_sweep_index = (
                            vision_sweep_index + 1
                        ) % len(vision_sweep_offsets)
                        vision_track_yaw = target_angles[0]
                        vision_track_pitch = target_angles[3]
                        vision_sweep_next_at = now + vision_sweep_dwell_sec
                continue

            if key not in {
                ord("g"),
                PICK_PLACE_KEY,
                PICKUP_TUNING_KEY,
                GIFT_SUPPLY_PICK_KEY,
                RECYCLE_KEY,
                HELP_KEY,
            }:
                continue

            pick_place_requested = key == PICK_PLACE_KEY
            pickup_tuning_requested = key == PICKUP_TUNING_KEY
            gift_supply_requested = key == GIFT_SUPPLY_PICK_KEY
            recycle_requested = key == RECYCLE_KEY
            help_requested = key == HELP_KEY
            if recycle_requested and (
                config.ARM_ID != "left"
                or not config.RECYCLE_ENABLED
            ):
                completion_message = "Recycle failed: disabled or not Left Arm"
                with state_lock:
                    last_error = completion_message
                post_robot_log(completion_message)
                continue
            if help_requested and (
                config.ARM_ID != "left"
                or not config.MARKER_PICKUP_ENABLED
                or config.MARKER_PICKUP_MARKER_ID != 0
            ):
                completion_message = (
                    "Help failed: AprilTag ID 0 pickup is disabled or not Left Arm"
                )
                with state_lock:
                    last_error = completion_message
                post_robot_log(completion_message)
                continue
            if gift_supply_requested and (
                config.ARM_ID != "left"
                or not config.GIFT_SUPPLY_SEARCH_ENABLED
            ):
                completion_message = (
                    "Gift supply failed: disabled or not Left Arm"
                )
                with state_lock:
                    last_error = completion_message
                post_robot_log(
                    completion_message,
                )
                continue
            if pickup_tuning_requested and not (
                config.MARKER_PICKUP_ENABLED
                and config.ARM_ID == config.MARKER_PICKUP_ARM_ID
            ):
                with state_lock:
                    last_error = "Pickup tuning is enabled only for the configured marker-pickup arm"
                continue

            gesture_triggered_pick = gesture_pick_cycle_requested
            auto_place_after_this_pick = (
                pick_place_requested
                or help_requested
                or (
                    not pickup_tuning_requested
                    and gesture_triggered_pick
                    and config.HAND_GESTURE_AUTO_PLACE
                )
            )
            gesture_pick_cycle_requested = False

            with state_lock:
                last_error = None
            try:
                if (
                    gesture_triggered_pick
                    and config.ARM_ID == "right"
                    and config.GESTURE_HOME_ENABLED
                    and not config.DRY_RUN
                ):
                    print("[GESTURE PICK] align to verified HOME joints before pickup")
                    if not wasab_arm_controller.move_home_keep_gripper_closed(
                        abort_event=stop_request,
                    ):
                        with state_lock:
                            last_error = "Gesture pickup aborted: HOME timeout"
                        continue
                if not config.DRY_RUN:
                    wasab_arm_controller.ensure_gripper_open()
                    gripper_closed_on_target = False
                if gift_supply_requested and not config.DRY_RUN:
                    print("[GIFT SUPPLY] return to Left HOME before narrow J5 scan")
                    if not wasab_arm_controller.move_home_keep_gripper_closed(
                        abort_event=stop_request,
                    ):
                        with state_lock:
                            last_error = "Gift supply HOME timeout"
                        continue

                if (
                    (pickup_tuning_requested or help_requested)
                    and config.MARKER_PICKUP_ENABLED
                    and config.ARM_ID == config.MARKER_PICKUP_ARM_ID
                ):
                    if help_requested and not config.DRY_RUN:
                        print(
                            "[HELP] move to HOME and detect AprilTag ID 0 "
                            "without joint search"
                        )
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request,
                        ):
                            with state_lock:
                                last_error = "Help failed: HOME timeout"
                            continue
                    print(
                        (
                            "[HELP] find movable AprilTag ID 0 object, "
                            "then pick and Place:"
                            if help_requested
                            else (
                                "[MARKER PICKUP] find movable ID 0 box, "
                                "then dynamic pick:"
                            )
                        ),
                        f"id={config.MARKER_PICKUP_MARKER_ID}",
                        f"pose={config.MARKER_PICKUP_FLANGE_COORDS}",
                    )
                    search_start_angles = wasab_arm_controller.get_joint_angles()
                    pan_offsets = [0.0]
                    if (
                        not help_requested
                        and config.MARKER_PICKUP_JOINT_SEARCH_ENABLED
                    ):
                        offset = config.MARKER_PICKUP_JOINT_SEARCH_STEP_DEG
                        while offset <= config.MARKER_PICKUP_NEGATIVE_RANGE_DEG + 1e-9:
                            pan_offsets.append(-offset)
                            offset += config.MARKER_PICKUP_JOINT_SEARCH_STEP_DEG
                        offset = config.MARKER_PICKUP_JOINT_SEARCH_STEP_DEG
                        while offset <= config.MARKER_PICKUP_POSITIVE_RANGE_DEG + 1e-9:
                            pan_offsets.append(offset)
                            offset += config.MARKER_PICKUP_JOINT_SEARCH_STEP_DEG

                    pan_index = config.MARKER_PICKUP_PAN_JOINT - 1
                    j5_index = 4
                    marker_detection = None
                    searched_targets: set[tuple[float, ...]] = set()
                    help_j5_offsets = (
                        [0.0]
                        if help_requested
                        else config.MARKER_PICKUP_J5_OFFSETS_DEG
                    )
                    for j5_offset in help_j5_offsets:
                        for pan_offset in pan_offsets:
                            search_angles = list(search_start_angles)
                            search_angles[pan_index] = _clamp(
                                search_start_angles[pan_index] + pan_offset,
                                JOINT_LIMITS_DEG[pan_index],
                            )
                            search_angles[j5_index] = _clamp(
                                search_start_angles[j5_index] + j5_offset,
                                JOINT_LIMITS_DEG[j5_index],
                            )
                            target_key = tuple(round(float(value), 3) for value in search_angles)
                            if target_key in searched_targets:
                                continue
                            searched_targets.add(target_key)

                            moved = any(
                                abs(target - start) > 1e-6
                                for target, start in zip(search_angles, search_start_angles)
                            )
                            if moved and not config.DRY_RUN:
                                with state_lock:
                                    last_error = (
                                        f"Pickup marker angle search: "
                                        f"J{config.MARKER_PICKUP_PAN_JOINT}={search_angles[pan_index]:.1f}, "
                                        f"J5={search_angles[j5_index]:.1f}"
                                    )
                                print("[MARKER PICKUP] joint search target:", search_angles)
                                wasab_arm_controller.send_joint_angles(
                                    search_angles,
                                    speed=config.MARKER_PICKUP_JOINT_SEARCH_SPEED,
                                    async_command=True,
                                )
                                reached = wasab_arm_controller.wait_until_joint_angles(
                                    search_angles,
                                    timeout_sec=config.MOVE_TIMEOUT_SEC,
                                    tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                    abort_event=stop_request,
                                )
                                if not reached:
                                    continue
                                if not _wait_or_abort(
                                    config.MARKER_PICKUP_JOINT_SEARCH_SETTLE_SEC,
                                    stop_request,
                                ):
                                    break

                            marker_frame, frame_seq = get_latest_frame(
                                min_seq=frame_seq,
                                timeout_sec=config.MARKER_PICKUP_JOINT_SEARCH_FRAME_TIMEOUT_SEC,
                            )
                            if marker_frame is None:
                                continue
                            if help_requested:
                                marker_result = request_wasab_apriltag_detection(
                                    marker_frame,
                                    config.MARKER_PICKUP_MARKER_ID,
                                )
                                raw_marker_detection = marker_result.get(
                                    "detection"
                                )
                                marker_detection = (
                                    raw_marker_detection
                                    if marker_result.get("status") == "ok"
                                    and isinstance(raw_marker_detection, dict)
                                    else None
                                )
                            else:
                                marker_detection = detect_april_marker(
                                    marker_frame,
                                    allowed_ids={
                                        config.MARKER_PICKUP_MARKER_ID
                                    },
                                )
                            if marker_detection is not None:
                                print(
                                    "[MARKER PICKUP] ID 0 box marker found:",
                                    f"id={marker_detection['id']}",
                                    f"J{config.MARKER_PICKUP_PAN_JOINT}={search_angles[pan_index]:.1f}",
                                    f"J5={search_angles[j5_index]:.1f}",
                                )
                                break
                        if marker_detection is not None or stop_request.is_set():
                            break

                    if marker_detection is None:
                        if config.MARKER_PICKUP_RETURN_TO_START_IF_MISSING and not config.DRY_RUN:
                            wasab_arm_controller.send_joint_angles(
                                search_start_angles,
                                speed=config.MARKER_PICKUP_JOINT_SEARCH_SPEED,
                                async_command=True,
                            )
                            wasab_arm_controller.wait_until_joint_angles(
                                search_start_angles,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                abort_event=stop_request,
                            )
                        with state_lock:
                            last_error = f"Pickup tray marker ID {config.MARKER_PICKUP_MARKER_ID} not visible"
                        print("[MARKER PICKUP]", last_error)
                        continue
                    with state_lock:
                        last_marker_detection = marker_detection
                        last_marker_detection_expires_at = time.monotonic() + 5.0
                    if config.DRY_RUN:
                        print("[DRY RUN] marker pickup command not sent")
                        continue
                    if config.MARKER_PICKUP_DYNAMIC_ALIGNMENT:
                        current_flange_coords = wasab_arm_controller.get_flange_coords()
                        payload = request_wasab_marker_pickup_plan(
                            marker_detection,
                            current_flange_coords,
                            config.MARKER_PICKUP_PLANE_Z_BASE_MM,
                            config.MARKER_PICKUP_TARGET_Z_OFFSET_MM,
                            config.MARKER_PICKUP_TARGET_BASE_OFFSET_MM,
                            list(config.MARKER_PICKUP_FLANGE_COORDS[3:]),
                        )
                        plan = payload.get("plan") if isinstance(payload, dict) else None
                        planned_raw = plan.get("flange_command") if isinstance(plan, dict) else None
                        try:
                            planned_command = [float(value) for value in planned_raw]
                        except (TypeError, ValueError):
                            planned_command = []
                        if len(planned_command) != 6:
                            with state_lock:
                                last_error = "Marker pickup plan rejected: invalid flange_command"
                            print("[SAFETY]", last_error)
                            continue
                        # A movable ID 0 box uses the complete hand-eye-derived
                        # Cartesian target. Local safety validation remains the
                        # final gate before motion.
                        command = list(planned_command)
                        command[0] = round(
                            command[0] + config.MARKER_PICKUP_GRASP_X_CLEARANCE_MM,
                            2,
                        )
                        command[2] = round(
                            command[2] + config.MARKER_PICKUP_GRASP_Z_CLEARANCE_MM,
                            2,
                        )
                        corrected_payload = {"status": "ok", "plan": {"flange_command": command}}
                        is_safe, reason, command = validate_server_plan(corrected_payload)
                        if not is_safe or command is None:
                            with state_lock:
                                last_error = (
                                    f"Marker-aligned pickup rejected locally: {reason}; "
                                    f"candidate={corrected_payload['plan']['flange_command']}"
                                )
                            print("[SAFETY]", last_error)
                            continue
                    else:
                        command = list(config.MARKER_PICKUP_FLANGE_COORDS)
                        command[0] = round(
                            command[0] + config.MARKER_PICKUP_GRASP_X_CLEARANCE_MM,
                            2,
                        )
                        command[2] = round(
                            command[2] + config.MARKER_PICKUP_GRASP_Z_CLEARANCE_MM,
                            2,
                        )

                    if pickup_tuning_requested:
                        # Tuning must always use the operator-measured reference,
                        # not a view-dependent hand-eye XY estimate.
                        tuning_command = list(config.MARKER_PICKUP_FLANGE_COORDS)
                        tuning_command[2] = round(
                            tuning_command[2] + config.MARKER_PICKUP_TUNING_CLEARANCE_Z_MM,
                            2,
                        )
                        tuning_payload = {
                            "status": "ok",
                            "plan": {"flange_command": tuning_command},
                        }
                        is_safe, reason, tuning_command = validate_server_plan(tuning_payload)
                        if not is_safe or tuning_command is None:
                            with state_lock:
                                last_error = f"Pickup tuning pose rejected: {reason}"
                            print("[SAFETY]", last_error)
                            continue
                        print(
                            "[PICKUP TUNING] approach-only pose; gripper will stay open:",
                            tuning_command,
                        )
                        reached = send_marker_pickup_pose_and_wait(
                            wasab_arm_controller,
                            tuning_command,
                            speed=config.MARKER_PICKUP_APPROACH_SPEED,
                            mode=config.MOVE_MODE,
                            abort_event=stop_request,
                        )
                        with state_lock:
                            last_error = (
                                "Pickup tuning ready: adjust manually and press Pose"
                                if reached
                                else "Pickup tuning pose timeout"
                            )
                        continue

                    approach_command = list(command)
                    approach_command[2] = round(
                        approach_command[2]
                        + config.MARKER_PICKUP_APPROACH_LIFT_Z_MM,
                        2,
                    )
                    print(
                        "[MARKER PICKUP] marker-aligned vertical approach:",
                        f"approach={approach_command}",
                        f"pick={command}",
                    )
                    reached = send_marker_pickup_pose_and_wait(
                        wasab_arm_controller,
                        approach_command,
                        speed=config.MARKER_PICKUP_APPROACH_SPEED,
                        mode=config.MOVE_MODE,
                        abort_event=stop_request,
                        position_tolerance_mm=(
                            config.MARKER_PICKUP_APPROACH_POSITION_TOL_MM
                        ),
                    )
                    if reached:
                        if (
                            help_requested
                            or config.MARKER_PICKUP_USE_JOINT_TARGET
                        ):
                            final_pick_joint_angles = (
                                config.HELP_PICK_JOINT_ANGLES
                                if help_requested
                                else config.MARKER_PICKUP_TARGET_JOINT_ANGLES
                            )
                            final_pick_speed = (
                                config.HELP_PICK_SPEED
                                if help_requested
                                else config.MARKER_PICKUP_DESCENT_SPEED
                            )
                            print(
                                "[HELP] final pickup joint target:"
                                if help_requested
                                else "[MARKER PICKUP] final joint target:",
                                final_pick_joint_angles,
                            )
                            wasab_arm_controller.send_joint_angles(
                                final_pick_joint_angles,
                                speed=final_pick_speed,
                                async_command=True,
                            )
                            reached = wasab_arm_controller.wait_until_joint_angles(
                                final_pick_joint_angles,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                abort_event=stop_request,
                            )
                            if reached:
                                final_pick_command = (
                                    wasab_arm_controller.get_flange_coords()
                                )
                                print(
                                    "[MARKER PICKUP] joint target pose:",
                                    final_pick_command,
                                )
                                if config.MARKER_PICKUP_FINE_DESCENT_Z_MM > 0:
                                    final_pick_command[2] = round(
                                        final_pick_command[2]
                                        - config.MARKER_PICKUP_FINE_DESCENT_Z_MM,
                                        2,
                                    )
                                    print(
                                        "[MARKER PICKUP] final fine Base Z descent:",
                                        final_pick_command,
                                    )
                                    reached = send_marker_pickup_pose_and_wait(
                                        wasab_arm_controller,
                                        final_pick_command,
                                        speed=config.MARKER_PICKUP_DESCENT_SPEED,
                                        mode=1,
                                        abort_event=stop_request,
                                    )
                        else:
                            reached = send_marker_pickup_pose_and_wait(
                                wasab_arm_controller,
                                command,
                                speed=config.MARKER_PICKUP_DESCENT_SPEED,
                                mode=1,
                                abort_event=stop_request,
                            )
                            final_pick_command = list(command)
                    if not reached:
                        with state_lock:
                            last_error = "Marker pickup pose timeout; gripper remains open"
                        continue
                    wasab_arm_controller.close_gripper()
                    gripper_closed_on_target = True
                    lift_command = list(final_pick_command)
                    lift_command[2] = round(
                        lift_command[2]
                        + config.MARKER_PICKUP_POST_PICK_LIFT_Z_MM,
                        2,
                    )
                    print(
                        "[MARKER PICKUP] post-pick Base Z lift:",
                        lift_command,
                    )
                    lift_reached = send_marker_pickup_pose_and_wait(
                        wasab_arm_controller,
                        lift_command,
                        speed=config.MARKER_PICKUP_POST_PICK_LIFT_SPEED,
                        mode=1,
                        abort_event=stop_request,
                    )
                    if not lift_reached:
                        with state_lock:
                            last_error = "Marker pickup succeeded, but post-pick Z lift timed out"
                        continue
                    last_pick_flange_command = final_pick_command
                    last_pick_target_label = (
                        f"april_tag_{config.MARKER_PICKUP_MARKER_ID}_box"
                    )
                    last_pick_gripper_auto_rotated = False
                    with state_lock:
                        last_error = f"Marker pickup complete: ID {config.MARKER_PICKUP_MARKER_ID}"
                    if help_requested:
                        print(
                            "[HELP] pickup complete; HOME -> dedicated Place:",
                            config.HELP_PLACE_JOINT_ANGLES,
                        )
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request,
                        ):
                            with state_lock:
                                last_error = (
                                    "Help pickup complete, but HOME approach "
                                    "before Place timed out"
                                )
                            continue
                        wasab_arm_controller.send_joint_angles(
                            config.HELP_PLACE_JOINT_ANGLES,
                            speed=config.HELP_PLACE_SPEED,
                            async_command=True,
                        )
                        help_place_reached = (
                            wasab_arm_controller.wait_until_joint_angles(
                                config.HELP_PLACE_JOINT_ANGLES,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                abort_event=stop_request,
                            )
                        )
                        if not help_place_reached:
                            with state_lock:
                                last_error = (
                                    "Help Place final joint pose timeout; "
                                    "object remains held"
                                )
                            continue
                        if config.PLACE_RELEASE_PAUSE_SEC > 0:
                            time.sleep(config.PLACE_RELEASE_PAUSE_SEC)
                        wasab_arm_controller.open_gripper(
                            speed=config.PLACE_GRIPPER_OPEN_SPEED,
                            settle_sec=config.PLACE_GRIPPER_SETTLE_SEC,
                        )
                        gripper_closed_on_target = False
                        last_pick_flange_command = None
                        last_pick_target_label = None
                        last_pick_gripper_auto_rotated = False
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request,
                        ):
                            with state_lock:
                                last_error = (
                                    "Help Place released, but HOME return timed out"
                                )
                            continue
                        with state_lock:
                            last_error = "Help complete: ID 0 picked and placed"
                        post_robot_log(
                            "Help complete: AprilTag ID 0 object picked and placed"
                        )
                        continue
                    if auto_place_after_this_pick:
                        auto_place_pending = True
                    continue

                if config.PICK_SEARCH_MOVE_TO_START_ENABLED and not config.DRY_RUN:
                    print(
                        "[PICK SEARCH] move to configured start pose:",
                        config.PICK_SEARCH_START_FLANGE_COORDS,
                    )
                    reached = wasab_arm_controller.send_flange_coords_and_wait(
                        config.PICK_SEARCH_START_FLANGE_COORDS,
                        speed=config.PICK_SEARCH_SPEED,
                        abort_event=stop_request,
                    )
                    if not reached:
                        _set_last_error("Pick-search start pose timeout")
                        continue
                    frame, frame_seq = get_latest_frame(
                        min_seq=frame_seq,
                        timeout_sec=config.PICK_SEARCH_FRAME_TIMEOUT_SEC,
                    )

                # Gift Giving has its own narrow Left HOME/J5-only scan. Normal
                # Pick keeps the existing stationary/search behavior.
                if gift_supply_requested:
                    payload, plan_frame_seq = find_gift_supply_pick_plan(
                        wasab_arm_controller,
                        get_latest_frame,
                        frame_seq,
                        lambda message: _set_last_error(message),
                        stop_request,
                    )
                elif recycle_requested:
                    payload = {"status": "not_found"}
                    plan_frame_seq = frame_seq
                    for recycle_label in RECYCLE_TARGET_LABELS:
                        payload, plan_frame_seq = request_pick_plan_from_current_view(
                            wasab_arm_controller,
                            get_latest_frame,
                            plan_frame_seq,
                            1.0,
                            target_label=recycle_label,
                        )
                        if payload.get("status") == "ok":
                            print(
                                "[RECYCLE] detected:",
                                recycle_label,
                            )
                            break
                else:
                    payload, plan_frame_seq = request_pick_plan_from_current_view(
                        wasab_arm_controller,
                        get_latest_frame,
                        frame_seq,
                        1.0,
                        target_label=PICK_TARGET_LABEL,
                    )
                requested_pick_label = (
                    str(payload.get("detection", {}).get("label"))
                    if recycle_requested
                    and isinstance(payload.get("detection"), dict)
                    and payload.get("detection", {}).get("label") is not None
                    else RECYCLE_TARGET_LABELS[-1]
                    if recycle_requested
                    else PICK_TARGET_LABEL
                )
                if gift_supply_requested and payload.get("status") != "ok":
                    completion_message = (
                        "Restock complete: no Coca-Cola found; returned HOME"
                    )
                    with state_lock:
                        last_error = completion_message
                    post_robot_log(
                        completion_message,
                    )
                    print("[RESTOCK]", last_error)
                    continue
                for stationary_attempt in range(
                    1,
                    (
                        0
                        if gift_supply_requested
                        else config.PICK_STATIONARY_RETRY_COUNT
                    )
                    + 1,
                ):
                    if not is_no_target_detection_response(payload):
                        break
                    if stop_request.is_set():
                        break
                    if config.PICK_STATIONARY_RETRY_INTERVAL_SEC > 0:
                        if not _wait_or_abort(
                            config.PICK_STATIONARY_RETRY_INTERVAL_SEC,
                            stop_request,
                        ):
                            break
                    print(
                        "[PICK] stationary detection retry:",
                        f"{stationary_attempt}/{config.PICK_STATIONARY_RETRY_COUNT}",
                    )
                    payload, plan_frame_seq = request_pick_plan_from_current_view(
                        wasab_arm_controller,
                        get_latest_frame,
                        plan_frame_seq,
                        1.0,
                        target_label=requested_pick_label,
                    )
                if not gift_supply_requested:
                    payload = find_pick_plan_by_joint_search(
                        wasab_arm_controller,
                        get_latest_frame,
                        payload,
                        plan_frame_seq,
                        lambda message: _set_last_error(message),
                        stop_request,
                        target_label=requested_pick_label,
                    )

                print(
                    "[LAPTOP RESPONSE]\n",
                    json.dumps(payload, ensure_ascii=False, indent=2),
                )
                with state_lock:
                    last_payload = payload
                    last_payload_expires_at = time.monotonic() + 3.0

                plan_for_validation = payload
                if (
                    config.ARM_ID == "left"
                    and payload.get("status") == "ok"
                    and any(
                        abs(value) > 1e-9
                        for value in config.PICK_PLAN_BASE_XY_OFFSET_MM
                    )
                ):
                    raw_plan = payload.get("plan")
                    raw_command = (
                        raw_plan.get("flange_command")
                        if isinstance(raw_plan, dict)
                        else None
                    )
                    if isinstance(raw_command, list) and len(raw_command) == 6:
                        corrected_command = [float(value) for value in raw_command]
                        corrected_command[0] = round(
                            corrected_command[0]
                            + config.PICK_PLAN_BASE_XY_OFFSET_MM[0],
                            2,
                        )
                        corrected_command[1] = round(
                            corrected_command[1]
                            + config.PICK_PLAN_BASE_XY_OFFSET_MM[1],
                            2,
                        )
                        plan_for_validation = {
                            "status": "ok",
                            "plan": {"flange_command": corrected_command},
                        }
                        print(
                            "[PICK] Left plan Base XY correction:",
                            f"offset={config.PICK_PLAN_BASE_XY_OFFSET_MM}",
                            f"raw={raw_command[:3]}",
                            f"corrected={corrected_command[:3]}",
                        )
                if (
                    config.ARM_ID == "left"
                    and config.PICK_FIXED_REFERENCE_ENABLED
                    and config.PICK_FIXED_REFERENCE_FLANGE_COORDS is not None
                    and payload.get("status") == "ok"
                ):
                    fixed_command = list(
                        config.PICK_FIXED_REFERENCE_FLANGE_COORDS
                    )
                    if config.PICK_VISUAL_XY_CORRECTION_ENABLED:
                        raw_plan = payload.get("plan")
                        raw_command = (
                            raw_plan.get("flange_command")
                            if isinstance(raw_plan, dict)
                            else None
                        )
                        if (
                            isinstance(raw_command, list)
                            and len(raw_command) == 6
                        ):
                            visual_delta_xy: list[float] = []
                            for axis in (0, 1):
                                raw_value = float(raw_command[axis])
                                reference_value = (
                                    config.PICK_VISUAL_REFERENCE_PLAN_XY_MM[
                                        axis
                                    ]
                                )
                                visual_delta_xy.append(
                                    _clamp(
                                        raw_value - reference_value,
                                        (
                                            -config.PICK_VISUAL_XY_CORRECTION_MAX_MM,
                                            config.PICK_VISUAL_XY_CORRECTION_MAX_MM,
                                        ),
                                    )
                                )
                            fixed_command[0] = round(
                                fixed_command[0] + visual_delta_xy[0],
                                2,
                            )
                            fixed_command[1] = round(
                                fixed_command[1] + visual_delta_xy[1],
                                2,
                            )
                            unclamped_visual_y = fixed_command[1]
                            fixed_command[1] = round(
                                _clamp(fixed_command[1], (-281.0, 281.0)),
                                2,
                            )
                            print(
                                "[PICK CENTERING] Left visual XY correction:",
                                f"raw_plan_xy={raw_command[:2]}",
                                "reference_plan_xy="
                                f"{config.PICK_VISUAL_REFERENCE_PLAN_XY_MM}",
                                f"delta_xy={visual_delta_xy}",
                                f"Y_clamp={unclamped_visual_y:.2f}"
                                f"->{fixed_command[1]:.2f}",
                            )
                    plan_for_validation = {
                        "status": "ok",
                        "plan": {"flange_command": fixed_command},
                    }
                    print(
                        "[PICK] Left measured fixed grasp reference:",
                        fixed_command,
                    )
                if (
                    config.ARM_ID == "right"
                    and config.RIGHT_PICK_MOTION_STRATEGY == "hybrid"
                    and config.RIGHT_PICK_REFERENCE_ENABLED
                    and config.RIGHT_PICK_REFERENCE_FLANGE_COORDS is not None
                    and payload.get("status") == "ok"
                ):
                    raw_plan = payload.get("plan")
                    raw_command = (
                        raw_plan.get("flange_command")
                        if isinstance(raw_plan, dict)
                        else None
                    )
                    if isinstance(raw_command, list) and len(raw_command) == 6:
                        try:
                            raw_command = [float(value) for value in raw_command]
                        except (TypeError, ValueError):
                            raw_command = None
                    if raw_command is not None:
                        reference = list(config.RIGHT_PICK_REFERENCE_FLANGE_COORDS)
                        max_xy = config.RIGHT_PICK_IK_CORRECTION_MAX_XY_MM
                        hybrid_command = list(reference)
                        for axis in (0, 1):
                            correction = max(
                                -max_xy,
                                min(max_xy, raw_command[axis] - reference[axis]),
                            )
                            hybrid_command[axis] = round(
                                reference[axis] + correction,
                                2,
                            )
                        plan_for_validation = {
                            "status": "ok",
                            "plan": {"flange_command": hybrid_command},
                        }
                        print(
                            "[PICK] Right hybrid target: measured joint approach "
                            "then bounded final IK correction:",
                            f"raw_xy={raw_command[:2]}",
                            f"reference_xy={reference[:2]}",
                            f"max_correction=±{max_xy:.1f}mm",
                            f"final={hybrid_command}",
                        )

                is_safe, reason, command = validate_server_plan(plan_for_validation)
                if (
                    (not is_safe or command is None)
                    and config.ARM_ID == "right"
                    and config.RIGHT_PICK_REFERENCE_ENABLED
                    and config.RIGHT_PICK_USE_REFERENCE_XYZ
                    and config.RIGHT_PICK_REFERENCE_FLANGE_COORDS is not None
                    and payload.get("status") == "ok"
                ):
                    reference = list(config.RIGHT_PICK_REFERENCE_FLANGE_COORDS)
                    referenced_payload = {
                        "status": "ok",
                        "plan": {"flange_command": reference},
                    }
                    is_safe, reason, command = validate_server_plan(
                        referenced_payload
                    )
                    if is_safe and command is not None:
                        print(
                            "[PICK] server XYZ was outside the local envelope; "
                            "using verified Right pickup reference before safety check:",
                            command,
                        )
                if not is_safe or command is None:
                    with state_lock:
                        last_error = f"Plan rejected locally: {reason}"
                    print("[SAFETY]", last_error)
                    continue

                command, correction_message = apply_auto_rotated_pick_xy_correction(payload, command)
                if correction_message is not None:
                    corrected_payload = {"status": "ok", "plan": {"flange_command": command}}
                    is_safe, reason, safe_corrected_command = validate_server_plan(corrected_payload)
                    if not is_safe or safe_corrected_command is None:
                        with state_lock:
                            last_error = f"Auto-rotated pick correction rejected locally: {reason}"
                        print("[SAFETY]", last_error)
                        continue
                    command = safe_corrected_command
                    print("[PICK]", correction_message, "corrected_command=", command)

                if (
                    config.ARM_ID == "right"
                    and config.RIGHT_PICK_REFERENCE_ENABLED
                    and config.RIGHT_PICK_REFERENCE_FLANGE_COORDS is not None
                ):
                    reference = config.RIGHT_PICK_REFERENCE_FLANGE_COORDS
                    if config.RIGHT_PICK_USE_REFERENCE_XYZ:
                        if config.RIGHT_PICK_MOTION_STRATEGY == "hybrid":
                            # The hybrid planner already bounded X/Y around the
                            # measured reference. Preserve that visual
                            # correction and pin only the verified grasp Z.
                            command[2] = reference[2]
                        else:
                            command[:3] = reference[:3]
                    elif config.RIGHT_PICK_USE_REFERENCE_Z:
                        command[2] = reference[2]
                    if config.RIGHT_PICK_USE_REFERENCE_ORIENTATION:
                        command[3:] = reference[3:]
                    referenced_payload = {
                        "status": "ok",
                        "plan": {"flange_command": command},
                    }
                    is_safe, reason, safe_reference_command = validate_server_plan(
                        referenced_payload
                    )
                    if not is_safe or safe_reference_command is None:
                        with state_lock:
                            last_error = (
                                f"Right pickup reference rejected locally: {reason}"
                            )
                        print("[SAFETY]", last_error)
                        continue
                    command = safe_reference_command
                    print(
                        "[PICK] applied Right pickup reference Z/orientation "
                        "while preserving bounded hybrid XY:"
                        if config.RIGHT_PICK_MOTION_STRATEGY == "hybrid"
                        else "[PICK] applied complete Right pickup reference pose:",
                        command,
                    )

                if config.DRY_RUN:
                    print(
                        "[DRY RUN] Laptop plan validated; no robot command sent:",
                        command,
                    )
                    continue

                if gift_supply_requested:
                    print(
                        "[RESTOCK] Coca-Cola detected; move once to the "
                        "Z-adjusted pickup flange target:",
                        config.GIFT_SUPPLY_PICKUP_FLANGE_COORDS,
                    )
                    reached = (
                        wasab_arm_controller.send_flange_coords_and_wait(
                            config.GIFT_SUPPLY_PICKUP_FLANGE_COORDS,
                            speed=config.GIFT_SUPPLY_SEARCH_SPEED,
                            mode=0,
                            abort_event=stop_request,
                        )
                    )
                elif (
                    config.ARM_ID == "right"
                    and config.RIGHT_PICK_MOTION_STRATEGY == "hybrid"
                    and config.RIGHT_PICK_REFERENCE_JOINT_ANGLES is not None
                    and config.HOME_JOINT_ANGLES is not None
                ):
                    direct_grasp_at_stage1 = False
                    measured_approach = (
                        config.RIGHT_PICK_APPROACH_FLANGE_COORDS is not None
                    )
                    measured_joint_approach = (
                        config.RIGHT_PICK_APPROACH_JOINT_ANGLES is not None
                    )
                    if measured_joint_approach:
                        pre_pick_angles = list(
                            config.RIGHT_PICK_APPROACH_JOINT_ANGLES
                        )
                        if config.RIGHT_PICK_VISUAL_JOINT_CORRECTION_ENABLED:
                            detection = payload.get("detection")
                            midpoint_uv = (
                                detection.get("midpoint_uv")
                                if isinstance(detection, dict)
                                else None
                            )
                            if (
                                isinstance(midpoint_uv, list)
                                and len(midpoint_uv) == 2
                            ):
                                dx_px = float(midpoint_uv[0]) - (
                                    config.RIGHT_PICK_VISUAL_REFERENCE_UV[0]
                                )
                                dy_px = float(midpoint_uv[1]) - (
                                    config.RIGHT_PICK_VISUAL_REFERENCE_UV[1]
                                )
                                max_delta = (
                                    config.RIGHT_PICK_VISUAL_JOINT_CORRECTION_MAX_DEG
                                )
                                joint_deltas = [
                                    _clamp(
                                        dx_px
                                        * config.RIGHT_PICK_VISUAL_J1_GAIN_DEG_PER_PX,
                                        (-max_delta, max_delta),
                                    ),
                                    _clamp(
                                        dy_px
                                        * config.RIGHT_PICK_VISUAL_J2_GAIN_DEG_PER_PX,
                                        (-max_delta, max_delta),
                                    ),
                                    _clamp(
                                        dy_px
                                        * config.RIGHT_PICK_VISUAL_J5_GAIN_DEG_PER_PX,
                                        (-max_delta, max_delta),
                                    ),
                                ]
                                for joint_index, delta in zip(
                                    (0, 1, 4),
                                    joint_deltas,
                                ):
                                    pre_pick_angles[joint_index] = round(
                                        pre_pick_angles[joint_index] + delta,
                                        2,
                                    )
                                print(
                                    "[PICK CENTERING] Right visual joint correction:",
                                    f"uv={midpoint_uv}",
                                    "reference_uv="
                                    f"{config.RIGHT_PICK_VISUAL_REFERENCE_UV}",
                                    f"pixel_delta={[dx_px, dy_px]}",
                                    f"joint_delta={joint_deltas}",
                                    f"target={pre_pick_angles}",
                                )
                    elif measured_approach:
                        measured_approach_command = list(
                            config.RIGHT_PICK_APPROACH_FLANGE_COORDS
                        )
                    else:
                        fraction = config.RIGHT_PICK_APPROACH_JOINT_FRACTION
                        pre_pick_angles = [
                            round(home + (final - home) * fraction, 2)
                            for home, final in zip(
                                config.HOME_JOINT_ANGLES,
                                config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                            )
                        ]
                    print(
                        "[PICK] Right hybrid stage 1/2 measured joint approach:"
                        if measured_joint_approach
                        else "[PICK] Right hybrid stage 1/2 measured flange approach:"
                        if measured_approach
                        else "[PICK] Right hybrid stage 1/3 joint approach:",
                        (
                            pre_pick_angles
                            if measured_joint_approach
                            else measured_approach_command
                            if measured_approach
                            else pre_pick_angles
                        ),
                        f"speed={config.RIGHT_PICK_JOINT_SPEED}",
                    )
                    if measured_joint_approach:
                        wasab_arm_controller.send_joint_angles(
                            pre_pick_angles,
                            speed=config.RIGHT_PICK_JOINT_SPEED,
                            async_command=True,
                        )
                        approach_reached = (
                            wasab_arm_controller.wait_until_joint_angles(
                                pre_pick_angles,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=max(
                                    config.POSE_ANGLE_TOL_DEG,
                                    3.0,
                                ),
                                abort_event=stop_request,
                            )
                        )
                    elif measured_approach:
                        wasab_arm_controller.send_flange_coords(
                            measured_approach_command,
                            speed=config.RIGHT_PICK_JOINT_SPEED,
                            mode=0,
                        )
                        approach_reached = (
                            wasab_arm_controller.wait_until_flange_pose(
                                measured_approach_command,
                                abort_event=stop_request,
                                position_tolerance_mm=(
                                    config.RIGHT_PICK_APPROACH_POSITION_TOL_MM
                                ),
                                angle_tolerance_deg=(
                                    config.RIGHT_PICK_APPROACH_ANGLE_TOL_DEG
                                ),
                            )
                        )
                    else:
                        wasab_arm_controller.send_joint_angles(
                            pre_pick_angles,
                            speed=config.RIGHT_PICK_JOINT_SPEED,
                            async_command=True,
                        )
                        approach_reached = (
                            wasab_arm_controller.wait_until_joint_angles(
                                pre_pick_angles,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=max(
                                    config.POSE_ANGLE_TOL_DEG,
                                    3.0,
                                ),
                                abort_event=stop_request,
                            )
                        )
                    if not approach_reached:
                        with state_lock:
                            last_error = "Right hybrid stage-1 approach timeout"
                        continue
                    if not _wait_or_abort(
                        config.RIGHT_PICK_APPROACH_SETTLE_SEC,
                        stop_request,
                    ):
                        continue
                    if config.RIGHT_PICK_APPROACH_SETTLE_SEC > 0:
                        print(
                            "[PICK] Right stage-1 settle:",
                            f"{config.RIGHT_PICK_APPROACH_SETTLE_SEC:.1f}s",
                        )
                    if measured_joint_approach:
                        # The taught stage-1 joint pose is the actual grasp pose.
                        # Reuse its measured flange pose so the common completion
                        # path closes the gripper without any additional motion.
                        command = (
                            wasab_arm_controller.get_flange_coords()
                        )
                        direct_grasp_at_stage1 = True
                        measured_joint_approach = False
                        measured_approach = True
                        print(
                            "[PICK] Right taught joint pose reached; "
                            "skip IK/XY/Z correction and close gripper:",
                            command,
                        )
                    if not measured_approach:
                        raised_command = list(command)
                        raised_command[2] = round(
                            raised_command[2] + config.PICK_APPROACH_LIFT_Z_MM,
                            2,
                        )
                        is_safe, reason, safe_raised_command = validate_server_plan(
                            {
                                "status": "ok",
                                "plan": {"flange_command": raised_command},
                            }
                        )
                        if not is_safe or safe_raised_command is None:
                            with state_lock:
                                last_error = (
                                    "Right raised pre-pick rejected locally: "
                                    f"{reason}"
                                )
                            print("[SAFETY]", last_error)
                            continue
                        raised_command = safe_raised_command
                        print(
                            "[PICK] Right hybrid stage 2/3 raised pre-pick:",
                            raised_command,
                            f"z_lift=+{config.PICK_APPROACH_LIFT_Z_MM:.1f}mm",
                            f"speed={config.PICK_APPROACH_SPEED}",
                        )
                        wasab_arm_controller.send_flange_coords(
                            raised_command,
                            speed=config.PICK_APPROACH_SPEED,
                            mode=0,
                        )
                        raised_reached = (
                            wasab_arm_controller.wait_until_flange_pose(
                                raised_command,
                                abort_event=stop_request,
                                position_tolerance_mm=(
                                    config.RIGHT_PICK_IK_POSITION_TOL_MM
                                ),
                                angle_tolerance_deg=(
                                    config.RIGHT_PICK_IK_ANGLE_TOL_DEG
                                ),
                            )
                        )
                        if not raised_reached:
                            with state_lock:
                                last_error = "Right raised pre-pick approach timeout"
                            continue
                    command[1] = round(
                        command[1]
                        + config.RIGHT_PICK_FINAL_DESCENT_Y_OFFSET_MM,
                        2,
                    )
                    is_safe, reason, safe_final_command = validate_server_plan(
                        {
                            "status": "ok",
                            "plan": {"flange_command": command},
                        }
                    )
                    if not is_safe or safe_final_command is None:
                        with state_lock:
                            last_error = (
                                "Right final-descent Y correction rejected locally: "
                                f"{reason}"
                            )
                        print("[SAFETY]", last_error)
                        continue
                    command = safe_final_command
                    if (
                        measured_joint_approach
                        and config.RIGHT_PICK_FINAL_XY_ALIGN_LIFT_Z_MM > 0
                    ):
                        current_before_lift = (
                            wasab_arm_controller.get_flange_coords()
                        )
                        z_lift_command = list(current_before_lift)
                        z_lift_command[2] = round(
                            command[2]
                            + config.RIGHT_PICK_FINAL_XY_ALIGN_LIFT_Z_MM,
                            2,
                        )
                        print(
                            "[PICK] Right hybrid stage 2/4 Z-only clearance lift:",
                            z_lift_command,
                        )
                        wasab_arm_controller.send_flange_coords(
                            z_lift_command,
                            speed=config.PICK_APPROACH_SPEED,
                            mode=1,
                        )
                        z_lift_reached = (
                            wasab_arm_controller.wait_until_flange_pose(
                                z_lift_command,
                                timeout_sec=(
                                    config.RIGHT_PICK_FINAL_ATTEMPT_TIMEOUT_SEC
                                ),
                                abort_event=stop_request,
                                position_tolerance_mm=(
                                    config.RIGHT_PICK_APPROACH_POSITION_TOL_MM
                                ),
                                angle_tolerance_deg=(
                                    config.RIGHT_PICK_APPROACH_ANGLE_TOL_DEG
                                ),
                            )
                        )
                        if not z_lift_reached:
                            print(
                                "[PICK] Right Z-only MoveL lift failed; "
                                "retrying the same clearance pose with MoveJ"
                            )
                            wasab_arm_controller.send_flange_coords(
                                z_lift_command,
                                speed=config.PICK_APPROACH_SPEED,
                                mode=0,
                            )
                            z_lift_reached = (
                                wasab_arm_controller.wait_until_flange_pose(
                                    z_lift_command,
                                    timeout_sec=max(
                                        10.0,
                                        config.RIGHT_PICK_FINAL_ATTEMPT_TIMEOUT_SEC,
                                    ),
                                    abort_event=stop_request,
                                    position_tolerance_mm=(
                                        config.RIGHT_PICK_APPROACH_POSITION_TOL_MM
                                    ),
                                    angle_tolerance_deg=(
                                        config.RIGHT_PICK_APPROACH_ANGLE_TOL_DEG
                                    ),
                                )
                            )
                        if not z_lift_reached:
                            with state_lock:
                                last_error = (
                                    "Right clearance lift failed with both "
                                    "MoveL and MoveJ"
                                )
                            continue
                        xy_align_command = list(command)
                        xy_align_command[2] = round(
                            xy_align_command[2]
                            + config.RIGHT_PICK_FINAL_XY_ALIGN_LIFT_Z_MM,
                            2,
                        )
                        is_safe, reason, safe_xy_align_command = (
                            validate_server_plan(
                                {
                                    "status": "ok",
                                    "plan": {
                                        "flange_command": xy_align_command
                                    },
                                }
                            )
                        )
                        if not is_safe or safe_xy_align_command is None:
                            with state_lock:
                                last_error = (
                                    "Right raised XY alignment rejected locally: "
                                    f"{reason}"
                                )
                            print("[SAFETY]", last_error)
                            continue
                        xy_align_command = safe_xy_align_command
                        print(
                            "[PICK] Right hybrid stage 3/4 raised XY alignment:",
                            xy_align_command,
                            f"z_lift=+"
                            f"{config.RIGHT_PICK_FINAL_XY_ALIGN_LIFT_Z_MM:.1f}mm",
                        )
                        wasab_arm_controller.send_flange_coords(
                            xy_align_command,
                            speed=config.PICK_APPROACH_SPEED,
                            mode=0,
                        )
                        xy_align_reached = (
                            wasab_arm_controller.wait_until_flange_pose(
                                xy_align_command,
                                timeout_sec=(
                                    config.RIGHT_PICK_FINAL_ATTEMPT_TIMEOUT_SEC
                                ),
                                abort_event=stop_request,
                                position_tolerance_mm=(
                                    config.RIGHT_PICK_APPROACH_POSITION_TOL_MM
                                ),
                                angle_tolerance_deg=(
                                    config.RIGHT_PICK_APPROACH_ANGLE_TOL_DEG
                                ),
                            )
                        )
                        if not xy_align_reached:
                            with state_lock:
                                last_error = (
                                    "Right raised XY alignment timeout"
                                )
                            continue
                    print(
                        "[PICK] Right hybrid stage 4/4 Z-only final descent:"
                        if (
                            measured_joint_approach
                            and config.RIGHT_PICK_FINAL_XY_ALIGN_LIFT_Z_MM > 0
                        )
                        else "[PICK] Right hybrid stage 2/2 straight final descent:"
                        if measured_approach
                        else "[PICK] Right hybrid stage 3/3 final IK correction:",
                        command,
                        "final_y_offset="
                        f"{config.RIGHT_PICK_FINAL_DESCENT_Y_OFFSET_MM:+.1f}mm",
                        f"speed={config.PICK_FINAL_APPROACH_SPEED}",
                    )
                    if direct_grasp_at_stage1:
                        print(
                            "[PICK] Right stage-1 joint target accepted; "
                            "closing gripper without another pose command"
                        )
                        reached = True
                    else:
                        wasab_arm_controller.send_flange_coords(
                            command,
                            speed=config.PICK_FINAL_APPROACH_SPEED,
                            mode=1 if measured_approach else 0,
                        )
                        motion_started = _wait_or_abort(
                            config.RIGHT_PICK_IK_MIN_MOTION_SEC,
                            stop_request,
                        )
                        reached = (
                            motion_started
                            and wasab_arm_controller.wait_until_flange_pose(
                                command,
                                timeout_sec=(
                                    config.RIGHT_PICK_FINAL_ATTEMPT_TIMEOUT_SEC
                                ),
                                abort_event=stop_request,
                                position_tolerance_mm=(
                                    config.RIGHT_PICK_IK_POSITION_TOL_MM
                                ),
                                angle_tolerance_deg=(
                                    config.RIGHT_PICK_IK_ANGLE_TOL_DEG
                                ),
                            )
                        )
                    retry_index = 0
                    while (
                        not reached
                        and not stop_request.is_set()
                        and retry_index < config.RIGHT_PICK_FINAL_RETRY_COUNT
                    ):
                        retry_index += 1
                        if config.RIGHT_PICK_REFERENCE_JOINT_ANGLES is None:
                            break
                        print(
                            "[PICK] Right fast final joint correction "
                            f"{retry_index}/{config.RIGHT_PICK_FINAL_RETRY_COUNT}:",
                            config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                        )
                        wasab_arm_controller.send_joint_angles(
                            config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                            speed=config.PICK_FINAL_APPROACH_SPEED,
                            async_command=True,
                        )
                        reached = (
                            wasab_arm_controller.wait_until_joint_angles(
                                config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                                timeout_sec=(
                                    config.RIGHT_PICK_FINAL_JOINT_RETRY_TIMEOUT_SEC
                                ),
                                tolerance_deg=max(
                                    config.POSE_ANGLE_TOL_DEG,
                                    3.0,
                                ),
                                abort_event=stop_request,
                            )
                        )
                elif (
                    config.ARM_ID == "right"
                    and config.RIGHT_PICK_MOTION_STRATEGY == "joint"
                    and config.RIGHT_PICK_USE_JOINT_TARGET
                    and config.RIGHT_PICK_REFERENCE_JOINT_ANGLES is not None
                ):
                    if (
                        config.PICK_TWO_STAGE_APPROACH_ENABLED
                        and config.HOME_JOINT_ANGLES is not None
                    ):
                        fraction = config.RIGHT_PICK_APPROACH_JOINT_FRACTION
                        pre_pick_angles = [
                            round(home + (final - home) * fraction, 2)
                            for home, final in zip(
                                config.HOME_JOINT_ANGLES,
                                config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                            )
                        ]
                        print(
                            "[PICK] Right two-stage joint pre-pick:",
                            pre_pick_angles,
                            f"speed={config.PICK_APPROACH_SPEED}",
                        )
                        wasab_arm_controller.send_joint_angles(
                            pre_pick_angles,
                            speed=config.PICK_APPROACH_SPEED,
                            async_command=True,
                        )
                        if not wasab_arm_controller.wait_until_joint_angles(
                            pre_pick_angles,
                            timeout_sec=config.MOVE_TIMEOUT_SEC,
                            tolerance_deg=max(config.POSE_ANGLE_TOL_DEG, 3.0),
                            abort_event=stop_request,
                        ):
                            with state_lock:
                                last_error = "Right pre-pick approach timeout"
                            continue
                    print(
                        "[PICK] Right final measured joint target:",
                        config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                    )
                    wasab_arm_controller.send_joint_angles(
                        config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                        speed=config.PICK_FINAL_APPROACH_SPEED,
                        async_command=True,
                    )
                    reached = wasab_arm_controller.wait_until_joint_angles(
                        config.RIGHT_PICK_REFERENCE_JOINT_ANGLES,
                        timeout_sec=config.MOVE_TIMEOUT_SEC,
                        tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                        abort_event=stop_request,
                    )
                else:
                    reached = execute_pick_approach(
                        wasab_arm_controller,
                        command,
                        abort_event=stop_request,
                    )
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
                    if (
                        config.ARM_ID == "left"
                        and pick_place_requested
                        and not gift_supply_requested
                    ):
                        # Preserve the very first source pickup coordinate.
                        # Normal Place clears last_pick_flange_command, but the
                        # final restock operation must return a new can here.
                        gift_restock_destination_command = list(command)
                        save_gift_restock_destination(
                            gift_restock_destination_command
                        )
                        print(
                            "[GIFT SUPPLY] saved original pickup destination:",
                            gift_restock_destination_command,
                        )
                    if recycle_requested:
                        detection = payload.get("detection", {})
                        picked_label = (
                            str(detection.get("label", "")).strip().lower()
                            if isinstance(detection, dict)
                            else ""
                        )
                        recycle_destination = {
                            "trash": (
                                "red",
                                config.RECYCLE_RED_FLANGE_COORDS,
                                config.RECYCLE_RED_JOINT_ANGLES,
                            ),
                            "water": (
                                "blue",
                                config.RECYCLE_BLUE_FLANGE_COORDS,
                                config.RECYCLE_BLUE_JOINT_ANGLES,
                            ),
                        }.get(picked_label)
                        if recycle_destination is None:
                            with state_lock:
                                last_error = (
                                    "Recycle stopped: picked class must be trash or water"
                                )
                            print("[RECYCLE]", last_error)
                            continue

                        bin_color, bin_flange_coords, bin_joint_angles = (
                            recycle_destination
                        )
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request
                        ):
                            with state_lock:
                                last_error = (
                                    "Recycle failed: HOME approach timeout while holding object"
                                )
                            continue

                        print(
                            "[RECYCLE] move to 3-second bin observation pose:",
                            f"flange={config.RECYCLE_VIEW_FLANGE_COORDS}",
                            f"joints={config.RECYCLE_VIEW_JOINT_ANGLES}",
                        )
                        wasab_arm_controller.send_joint_angles(
                            config.RECYCLE_VIEW_JOINT_ANGLES,
                            speed=config.RECYCLE_MOTION_SPEED,
                            async_command=True,
                        )
                        if not wasab_arm_controller.wait_until_joint_angles(
                            config.RECYCLE_VIEW_JOINT_ANGLES,
                            timeout_sec=config.MOVE_TIMEOUT_SEC,
                            tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                            abort_event=stop_request,
                        ):
                            with state_lock:
                                last_error = (
                                    "Recycle failed: bin observation pose timeout"
                                )
                            continue

                        color_detection = None
                        color_frame_seq = frame_seq
                        observation_started_at = time.monotonic()
                        observation_deadline = (
                            observation_started_at
                            + config.RECYCLE_VIEW_OBSERVE_SEC
                        )
                        color_attempt = 0
                        while time.monotonic() < observation_deadline:
                            if stop_request.is_set():
                                break
                            color_attempt += 1
                            remaining_sec = observation_deadline - time.monotonic()
                            color_frame, color_frame_seq = get_latest_frame(
                                min_seq=color_frame_seq,
                                timeout_sec=max(0.05, min(0.5, remaining_sec)),
                            )
                            if color_frame is None:
                                continue
                            candidate = detect_recycle_bin_color(
                                color_frame,
                                bin_color,
                            )
                            print(
                                "[RECYCLE] 3-second bin observation "
                                f"frame={color_attempt}:",
                                candidate,
                            )
                            if candidate is not None and (
                                color_detection is None
                                or candidate["area_ratio"]
                                > color_detection["area_ratio"]
                            ):
                                color_detection = candidate
                        if color_detection is None:
                            with state_lock:
                                last_error = (
                                    f"Recycle stopped: {bin_color} bin was not visible; "
                                    "object remains held at the observation pose"
                                )
                            post_robot_log(last_error)
                            continue

                        print(
                            "[RECYCLE] target bin center confirmed:",
                            color_detection,
                            "measured_final_flange=",
                            bin_flange_coords,
                            "measured_final_joints=",
                            bin_joint_angles,
                        )
                        use_dynamic_bin_target = (
                            config.RECYCLE_DYNAMIC_COLOR_TARGET
                            and not (
                                bin_color == "blue"
                                and config.RECYCLE_BLUE_FIXED_TARGET
                            )
                            and not (
                                bin_color == "red"
                                and config.RECYCLE_RED_FIXED_TARGET
                            )
                        )
                        if use_dynamic_bin_target:
                            current_flange_coords = (
                                wasab_arm_controller.get_flange_coords()
                            )
                            dynamic_payload = request_wasab_marker_place_plan(
                                {
                                    "role": "recycle_bin",
                                    "bbox": color_detection["bbox"],
                                    "center": color_detection["center"],
                                    "object_plane_z_base_mm": (
                                        config.RECYCLE_BIN_PLANE_Z_BASE_MM
                                    ),
                                    "flange_orientation_deg": list(
                                        bin_flange_coords[3:]
                                    ),
                                },
                                current_flange_coords,
                                picked_label,
                                target_base_offset_mm=[0.0, 0.0, 0.0],
                            )
                            try:
                                recycle_command = build_dynamic_recycle_command(
                                    dynamic_payload,
                                    bin_flange_coords,
                                    config.RECYCLE_DYNAMIC_MAX_XY_OFFSET_MM,
                                )
                            except ValueError as exc:
                                with state_lock:
                                    last_error = f"Recycle dynamic target rejected: {exc}"
                                print("[RECYCLE]", last_error)
                                continue
                            print(
                                "[RECYCLE] live pixel -> Base target:",
                                f"pixel={color_detection['center']}",
                                f"command={recycle_command}",
                            )
                            reached_bin = (
                                wasab_arm_controller.send_flange_coords_and_wait(
                                    recycle_command,
                                    speed=config.RECYCLE_MOTION_SPEED,
                                    mode=0,
                                    abort_event=stop_request,
                                )
                            )
                        elif (
                            bin_color == "blue"
                            and config.RECYCLE_BLUE_FIXED_TARGET
                        ):
                            print(
                                "[RECYCLE] direct measured blue joint target:",
                                bin_joint_angles,
                            )
                            wasab_arm_controller.send_joint_angles(
                                bin_joint_angles,
                                speed=config.RECYCLE_BLUE_MOTION_SPEED,
                                async_command=True,
                            )
                            reached_bin = wasab_arm_controller.wait_until_joint_angles(
                                bin_joint_angles,
                                timeout_sec=config.MOVE_TIMEOUT_SEC,
                                tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                abort_event=stop_request,
                            )
                        else:
                            print(
                                "[RECYCLE] fixed rollback mode:",
                                bin_joint_angles,
                            )
                            wasab_arm_controller.send_joint_angles(
                                bin_joint_angles,
                                speed=config.RECYCLE_MOTION_SPEED,
                                async_command=True,
                            )
                            reached_bin = (
                                wasab_arm_controller.wait_until_joint_angles(
                                    bin_joint_angles,
                                    timeout_sec=config.MOVE_TIMEOUT_SEC,
                                    tolerance_deg=config.POSE_ANGLE_TOL_DEG,
                                    abort_event=stop_request,
                                )
                            )
                        if not reached_bin:
                            with state_lock:
                                last_error = (
                                    f"Recycle failed: {bin_color} bin final pose timeout"
                                )
                            continue
                        wasab_arm_controller.open_gripper(
                            speed=config.PLACE_GRIPPER_OPEN_SPEED,
                            settle_sec=config.PLACE_GRIPPER_SETTLE_SEC,
                        )
                        gripper_closed_on_target = False
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request
                        ):
                            with state_lock:
                                last_error = (
                                    f"Recycle released {picked_label}; HOME return timeout"
                                )
                            continue
                        completion_message = (
                            f"Recycle complete: {picked_label} -> {bin_color} bin"
                        )
                        with state_lock:
                            last_error = completion_message
                        post_robot_log(completion_message)
                        last_pick_flange_command = None
                        last_pick_target_label = None
                        last_pick_gripper_auto_rotated = False
                        continue
                    if gift_supply_requested:
                        print(
                            "[RESTOCK] picked Coca-Cola; move to the "
                            "Z-adjusted place flange target:",
                            config.GIFT_SUPPLY_RESTOCK_PLACE_FLANGE_COORDS,
                        )
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request
                        ):
                            with state_lock:
                                last_error = "Gift supply failed: HOME approach timeout"
                            continue
                        marker_detection = None
                        marker_result: dict[str, Any] = {
                            "status": "not_found"
                        }
                        marker_seq = 0
                        for marker_attempt in range(1, 4):
                            marker_frame, marker_seq = get_latest_frame(
                                min_seq=marker_seq,
                                timeout_sec=1.0,
                            )
                            if marker_frame is None:
                                continue
                            marker_result = (
                                request_wasab_apriltag_detection(
                                    marker_frame,
                                    target_id=3,
                                )
                            )
                            marker_detection = marker_result.get(
                                "detection"
                            )
                            print(
                                "[RESTOCK] laptop AprilTag ID 3 detection "
                                f"{marker_attempt}/3:",
                                marker_result,
                            )
                            if isinstance(marker_detection, dict):
                                break
                            if not _wait_or_abort(0.3, stop_request):
                                break
                        if marker_seq == 0:
                            with state_lock:
                                last_error = (
                                    "Restock failed: no camera frame for ID 3"
                                )
                            continue
                        if not isinstance(marker_detection, dict):
                            with state_lock:
                                last_error = (
                                    "Restock failed: AprilTag ID 3 was not "
                                    "visible from HOME"
                                )
                            continue
                        current_flange_coords = (
                            wasab_arm_controller.get_flange_coords()
                        )
                        restock_place_payload = (
                            request_wasab_marker_place_plan(
                                marker_detection,
                                current_flange_coords,
                                "Coca-Cola",
                                target_base_offset_mm=[0.0, 0.0, 0.0],
                            )
                        )
                        print(
                            "[RESTOCK] AprilTag ID 3 place response:\n",
                            json.dumps(
                                restock_place_payload,
                                ensure_ascii=False,
                                indent=2,
                            ),
                        )
                        place_safe, place_reason, restock_place_command = (
                            validate_server_plan(restock_place_payload)
                        )
                        if not place_safe or restock_place_command is None:
                            with state_lock:
                                last_error = (
                                    "Restock ID 3 place rejected: "
                                    f"{place_reason}"
                                )
                            continue
                        restock_place_command[0] = round(
                            restock_place_command[0] - 5.0,
                            2,
                        )
                        restock_place_command[2] = round(
                            restock_place_command[2] - 10.0,
                            2,
                        )
                        restock_place_command[4] = round(
                            restock_place_command[4] + 5.0,
                            2,
                        )
                        place_safe, place_reason, restock_place_command = (
                            validate_server_plan(
                                {
                                    "status": "ok",
                                    "plan": {
                                        "flange_command": restock_place_command
                                    },
                                }
                            )
                        )
                        if not place_safe or restock_place_command is None:
                            with state_lock:
                                last_error = (
                                    "Restock adjusted ID 3 place rejected: "
                                    f"{place_reason}"
                                )
                            continue
                        print(
                            "[RESTOCK] adjusted ID 3 place target:",
                            restock_place_command,
                            "X=-5.0mm",
                            "Z=-10.0mm",
                            "RY=+5.0deg",
                        )
                        if not execute_place_final_approach(
                            wasab_arm_controller,
                            restock_place_command,
                            abort_event=stop_request,
                        ):
                            with state_lock:
                                last_error = (
                                    "Restock failed: ID 3 place timeout"
                                )
                            continue
                        wasab_arm_controller.open_gripper(
                            speed=config.PLACE_GRIPPER_OPEN_SPEED,
                            settle_sec=config.PLACE_GRIPPER_SETTLE_SEC,
                        )
                        gripper_closed_on_target = False
                        if not wasab_arm_controller.move_home_keep_gripper_closed(
                            abort_event=stop_request
                        ):
                            with state_lock:
                                last_error = (
                                    "Gift supply released; HOME return timeout"
                                )
                            continue
                        completion_message = (
                            "Restock complete; Coca-Cola placed at AprilTag ID 3"
                        )
                        with state_lock:
                            last_error = completion_message
                        post_robot_log(completion_message)
                        last_pick_flange_command = None
                        last_pick_target_label = None
                        last_pick_gripper_auto_rotated = False
                        continue
                    last_pick_flange_command = list(command)
                    detection = payload.get("detection", {})
                    last_pick_target_label = (
                        str(detection.get("label"))
                        if isinstance(detection, dict) and detection.get("label") is not None
                        else None
                    )
                    debug = payload.get("debug", {})
                    last_pick_gripper_auto_rotated = bool(
                        isinstance(debug, dict) and debug.get("gripper_auto_rotated")
                    )
                    completion_message = None
                    with state_lock:
                        last_error = completion_message
                    if auto_place_after_this_pick:
                        auto_place_pending = True
                        print("[PICK & PLACE] Pickup complete; Place queued")
                        if config.ARM_ID == "right":
                            post_robot_log(
                                "Right pickup complete; Place starting"
                            )
                else:
                    gripper_closed_on_target = False
                    last_pick_target_label = None
                    last_pick_gripper_auto_rotated = False
                    with state_lock:
                        timeout_detail = (
                            wasab_arm_controller.last_wait_timeout_reason
                            or "no detailed robot timeout reason"
                        )
                        last_error = (
                            "Target pose timeout; gripper remains unchanged: "
                            f"{timeout_detail}"
                        )

            except (WaSaBServiceError, RuntimeError, ValueError) as exc:
                with state_lock:
                    last_error = f"ERROR: {type(exc).__name__}: {exc}"
                print(last_error)

    finally:
        stream_stop.set()
        capture_stop.set()
        remote_stop.set()
        if hand_gesture is not None:
            hand_gesture.close()
        with frame_condition:
            frame_condition.notify_all()
        if remote_thread is not None and remote_thread.is_alive():
            remote_thread.join(timeout=1.0)
        if operation_thread.is_alive():
            operation_thread.join(timeout=1.0)
        if stream_thread is not None and stream_thread.is_alive():
            stream_thread.join(timeout=1.0)
        if workspace_thread is not None and workspace_thread.is_alive():
            workspace_thread.join(timeout=1.0)
        if capture_thread.is_alive():
            capture_thread.join(timeout=1.0)

        with cap_lock:
            cap.release()
        if show_window:
            cv2.destroyAllWindows()
        print("Client terminated")

    if calibration_requested:
        script_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "auto_marker.py"))
        post_robot_log(f"starting {script_path}", source="calibration")
        process = subprocess.Popen(
            [sys.executable, "-u", script_path],
            cwd=os.path.dirname(script_path),
            env={**os.environ, "WASAB_CALIBRATION_HEADLESS": "1"},
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
        assert process.stdout is not None
        for line in process.stdout:
            message = line.rstrip()
            if message:
                print(message)
                post_robot_log(message, source="auto_marker.py")
        return_code = process.wait()
        post_robot_log(
            f"auto_marker.py finished (exit={return_code})",
            level="info" if return_code == 0 else "error",
            source="calibration",
        )


if __name__ == "__main__":
    main()
