"""딥러닝 서버에서만 실행하는 기하/좌표변환 코드.

흐름:
YOLO bbox center pixel → distortion 보정 ray → ^bT_c → Base Z 평면 교차
→ TCP 목표점 → Flange 명령.
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from app.settings import Settings


class WaSaBOperationPlanError(RuntimeError):
    """보정값, 입력 pose 또는 ray-plane 교차가 유효하지 않을 때 발생."""


@dataclass(frozen=True)
class WaSaBCalibration:
    K: np.ndarray
    dist: np.ndarray
    T_flange_camera: np.ndarray  # ^fT_c: camera frame point → flange frame point
    selected_method: str | None
    euler_order: str


def rot_x(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]], dtype=np.float64)


def rot_y(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]], dtype=np.float64)


def rot_z(rad: float) -> np.ndarray:
    c, s = np.cos(rad), np.sin(rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def euler_to_rotation_matrix(rx_deg: float, ry_deg: float, rz_deg: float, order: str = "zyx") -> np.ndarray:
    """기존 MyCobot 코드와 동일한 zyx convention: Rz(rz) @ Ry(ry) @ Rx(rx)."""
    if order != "zyx":
        raise WaSaBOperationPlanError(f"Unsupported Euler order: {order}; only 'zyx' is supported")
    rx, ry, rz = np.deg2rad([rx_deg, ry_deg, rz_deg])
    rotation_map = {"x": rot_x(rx), "y": rot_y(ry), "z": rot_z(rz)}
    R = np.eye(3, dtype=np.float64)
    for axis in order:
        R = R @ rotation_map[axis]
    return R


def wasab_arm_pose_to_T_base_flange(coords: list[float], euler_order: str) -> np.ndarray:
    """Base 기준 MyCobot flange pose [x,y,z,rx,ry,rz] → ^bT_f (mm, degree)."""
    if len(coords) != 6:
        raise WaSaBOperationPlanError("flange_coords must contain six values")
    pose = np.asarray(coords, dtype=np.float64)
    if not np.isfinite(pose).all():
        raise WaSaBOperationPlanError("flange_coords contains non-finite values")
    x, y, z, rx, ry, rz = pose
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = euler_to_rotation_matrix(float(rx), float(ry), float(rz), euler_order)
    T[:3, 3] = [x, y, z]
    return T


def load_wasab_calibration(settings: Settings) -> WaSaBCalibration:
    if not settings.intrinsic_file.exists():
        raise FileNotFoundError(f"Intrinsic file not found: {settings.intrinsic_file}")
    if not settings.handeye_result_json.exists():
        raise FileNotFoundError(f"Hand-eye JSON not found: {settings.handeye_result_json}")

    intrinsic = np.load(str(settings.intrinsic_file))
    K = np.asarray(intrinsic["K"], dtype=np.float64)
    dist = np.asarray(intrinsic["dist"], dtype=np.float64)
    if K.shape != (3, 3):
        raise WaSaBOperationPlanError(f"Intrinsic K must have shape (3,3), got {K.shape}")

    with settings.handeye_result_json.open("r", encoding="utf-8") as f:
        handeye: dict[str, Any] = json.load(f)

    if handeye.get("calibration_mode") != "eye_in_hand":
        raise WaSaBOperationPlanError("Hand-eye JSON is not an eye_in_hand result")
    calibration_order = str(handeye.get("selected_euler_order", "")).lower()
    if calibration_order != settings.euler_order:
        raise WaSaBOperationPlanError(
            f"Euler order mismatch: calibration={calibration_order}, server={settings.euler_order}"
        )

    selected = handeye.get("selected")
    if not isinstance(selected, dict) or "T_gripper_camera" not in selected:
        raise WaSaBOperationPlanError("selected.T_gripper_camera is missing in hand-eye JSON")
    T_flange_camera = np.asarray(selected["T_gripper_camera"], dtype=np.float64)
    if T_flange_camera.shape != (4, 4):
        raise WaSaBOperationPlanError("T_gripper_camera must have shape (4,4)")
    if not np.isfinite(T_flange_camera).all():
        raise WaSaBOperationPlanError("T_gripper_camera contains non-finite values")

    return WaSaBCalibration(
        K=K,
        dist=dist,
        T_flange_camera=T_flange_camera,
        selected_method=handeye.get("selected_method"),
        euler_order=calibration_order,
    )


def detection_bbox_center_pixel(bbox: list[float]) -> tuple[float, float]:
    if len(bbox) != 4:
        raise WaSaBOperationPlanError("bbox must be [x1,y1,x2,y2]")
    x1, y1, x2, y2 = (float(v) for v in bbox)
    if x2 <= x1 or y2 <= y1:
        raise WaSaBOperationPlanError("bbox must have positive width and height")
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0


def _bbox_major_axis_deg(bbox: list[float]) -> float:
    x1, y1, x2, y2 = bbox
    return 0.0 if (x2 - x1) >= (y2 - y1) else 90.0


def _long_object_end_grip_pixel(
    detection: dict[str, Any],
    bbox: list[float],
    settings: Settings,
) -> tuple[float, float, str] | None:
    if not settings.long_object_end_grip_enabled:
        return None

    x1, y1, x2, y2 = bbox
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    if width <= 1e-9 or height <= 1e-9:
        return None

    aspect_ratio = max(width / height, height / width)
    if aspect_ratio < settings.long_object_end_grip_aspect_ratio:
        return None

    endpoints = detection.get("grip_axis_endpoints")
    if isinstance(endpoints, list) and len(endpoints) == 2:
        try:
            first = np.asarray(endpoints[0], dtype=np.float64)
            second = np.asarray(endpoints[1], dtype=np.float64)
        except (TypeError, ValueError):
            first = second = None
        if (
            first is not None
            and second is not None
            and first.shape == (2,)
            and second.shape == (2,)
            and np.isfinite(first).all()
            and np.isfinite(second).all()
        ):
            side = settings.long_object_end_grip_side
            if side == "upper":
                endpoint, opposite = (first, second) if first[1] <= second[1] else (second, first)
            elif side == "right":
                endpoint, opposite = (first, second) if first[0] >= second[0] else (second, first)
            elif side == "left":
                endpoint, opposite = (first, second) if first[0] <= second[0] else (second, first)
            elif side == "negative":
                endpoint, opposite = first, second
            else:
                endpoint, opposite = (first, second) if first[1] >= second[1] else (second, first)
                side = "lower" if side not in {"lower", "positive"} else side

            # Reuse the existing fraction as "how far from center toward the end".
            # 0.35 means roughly 15% of the endpoint-to-endpoint length inward
            # from the end. Keep a conservative minimum inset to avoid corners.
            center_offset_fraction = min(0.48, max(0.0, settings.long_object_end_grip_offset_fraction))
            inset_fraction = max(0.18, min(0.30, 0.5 - center_offset_fraction))
            target = endpoint + (opposite - endpoint) * inset_fraction
            margin = 2.0
            u = float(np.clip(target[0], x1 + margin, x2 - margin))
            v = float(np.clip(target[1], y1 + margin, y2 - margin))
            return u, v, f"long_object_end_grip_{side}_endpoint"

    center = detection.get("center")
    if isinstance(center, list) and len(center) == 2:
        try:
            center_u, center_v = (float(center[0]), float(center[1]))
        except (TypeError, ValueError):
            center_u, center_v = detection_bbox_center_pixel(bbox)
        else:
            if not np.isfinite([center_u, center_v]).all():
                center_u, center_v = detection_bbox_center_pixel(bbox)
    else:
        center_u, center_v = detection_bbox_center_pixel(bbox)

    try:
        axis_deg = float(detection.get("grip_axis_image_deg"))
    except (TypeError, ValueError):
        axis_deg = _bbox_major_axis_deg(bbox)

    axis_rad = np.deg2rad(axis_deg)
    axis = np.array([np.cos(axis_rad), np.sin(axis_rad)], dtype=np.float64)
    if float(np.linalg.norm(axis)) < 1e-9:
        return None

    side = settings.long_object_end_grip_side
    if side == "upper":
        sign = -1.0 if axis[1] >= 0.0 else 1.0
    elif side == "right":
        sign = 1.0 if axis[0] >= 0.0 else -1.0
    elif side == "left":
        sign = -1.0 if axis[0] >= 0.0 else 1.0
    elif side == "negative":
        sign = -1.0
    else:
        sign = 1.0 if axis[1] >= 0.0 else -1.0
        side = "lower" if side not in {"lower", "positive"} else side

    offset_fraction = min(0.48, max(0.0, settings.long_object_end_grip_offset_fraction))
    offset_px = max(width, height) * offset_fraction
    target = np.array([center_u, center_v], dtype=np.float64) + sign * axis * offset_px

    margin = 2.0
    u = float(np.clip(target[0], x1 + margin, x2 - margin))
    v = float(np.clip(target[1], y1 + margin, y2 - margin))
    return u, v, f"long_object_end_grip_{side}"


def detection_target_center_pixel(
    detection: dict[str, Any],
    bbox: list[float],
    settings: Settings,
) -> tuple[float, float, str]:
    end_grip_target = _long_object_end_grip_pixel(detection, bbox, settings)
    if end_grip_target is not None:
        return end_grip_target

    center = detection.get("center")
    if isinstance(center, list) and len(center) == 2:
        try:
            u, v = (float(center[0]), float(center[1]))
        except (TypeError, ValueError):
            pass
        else:
            if np.isfinite([u, v]).all():
                return u, v, str(detection.get("grip_center_source") or "detection_center")

    u, v = detection_bbox_center_pixel(bbox)
    return u, v, "bbox_center"


def image_pixel_to_camera_ray(u: float, v: float, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    pixel = np.array([[[u, v]]], dtype=np.float64)
    undistorted = cv2.undistortPoints(pixel, K, dist).reshape(2)
    ray = np.array([undistorted[0], undistorted[1], 1.0], dtype=np.float64)
    length = float(np.linalg.norm(ray))
    if length < 1e-12:
        raise WaSaBOperationPlanError("Invalid camera ray")
    return ray / length


def target_z_offset_for_label(label: str, settings: Settings) -> float:
    if label in settings.target_z_offsets_mm:
        return settings.target_z_offsets_mm[label]

    label_lower = label.lower()
    for configured_label, offset in settings.target_z_offsets_mm.items():
        if configured_label.lower() == label_lower:
            return offset

    return settings.default_target_z_offset_mm


def pick_target_base_offset_for_label(
    label: str,
    settings: Settings,
) -> tuple[float, float, float]:
    if label in settings.pick_target_base_offsets_mm:
        return settings.pick_target_base_offsets_mm[label]

    label_lower = label.lower()
    for configured_label, offset in settings.pick_target_base_offsets_mm.items():
        if configured_label.lower() == label_lower:
            return offset

    return (0.0, 0.0, 0.0)


def _normalize_angle_deg(angle: float) -> float:
    return (float(angle) + 180.0) % 360.0 - 180.0


def _gripper_rz_offset_for_detection(
    detection: dict[str, Any],
    bbox: list[float],
    settings: Settings,
) -> tuple[float, float, bool, float | None, str]:
    x1, y1, x2, y2 = bbox
    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    if height <= 1e-9:
        return 0.0, 0.0, False, None, "invalid_bbox"

    aspect_ratio = width / height
    if not settings.gripper_auto_rotate_long_bbox_enabled:
        return aspect_ratio, 0.0, False, None, "disabled"
    if aspect_ratio < settings.gripper_auto_rotate_aspect_ratio and (1.0 / aspect_ratio) < settings.gripper_auto_rotate_aspect_ratio:
        return aspect_ratio, 0.0, False, None, "near_square_bbox"

    raw_axis_angle = detection.get("grip_axis_image_deg")
    if raw_axis_angle is None:
        object_axis_image_deg = 0.0 if width >= height else 90.0
        source = "bbox_major_axis"
    else:
        object_axis_image_deg = float(raw_axis_angle) % 180.0
        source = str(detection.get("grip_axis_source") or "image_axis")

    # Baseline pick RZ is treated as the angle that grips a vertically oriented
    # object in the camera image. Horizontal objects therefore need +90 deg,
    # and diagonal objects get the corresponding intermediate correction.
    gripper_rz_offset_deg = _normalize_angle_deg(90.0 - object_axis_image_deg)
    return aspect_ratio, gripper_rz_offset_deg, True, object_axis_image_deg, source


def compute_wasab_operation_plan(
    *,
    detection: dict[str, Any],
    current_flange_coords: list[float],
    calibration: WaSaBCalibration,
    settings: Settings,
) -> dict[str, Any]:
    """서버에서 bbox 하나를 최종 MyCobot Flange 명령으로 변환합니다."""
    bbox = [float(v) for v in detection["bbox"]]
    u, v, target_center_source = detection_target_center_pixel(detection, bbox, settings)

    # ^bT_c = ^bT_f @ ^fT_c
    T_base_flange = wasab_arm_pose_to_T_base_flange(current_flange_coords, settings.euler_order)
    T_base_camera = T_base_flange @ calibration.T_flange_camera

    ray_camera = image_pixel_to_camera_ray(u, v, calibration.K, calibration.dist)
    camera_origin_base = T_base_camera[:3, 3]
    ray_direction_base = T_base_camera[:3, :3] @ ray_camera
    ray_direction_base /= np.linalg.norm(ray_direction_base)

    denominator = float(ray_direction_base[2])
    if abs(denominator) < 1e-9:
        raise WaSaBOperationPlanError("Center ray is nearly parallel to OBJECT_PLANE_Z_BASE_MM")

    try:
        object_plane_z_base_mm = float(
            detection.get("object_plane_z_base_mm", settings.object_plane_z_base_mm)
        )
    except (TypeError, ValueError) as exc:
        raise WaSaBOperationPlanError("object_plane_z_base_mm must be numeric") from exc
    if not np.isfinite(object_plane_z_base_mm):
        raise WaSaBOperationPlanError("object_plane_z_base_mm must be finite")
    ray_scale_mm = (
        object_plane_z_base_mm - float(camera_origin_base[2])
    ) / denominator
    if ray_scale_mm <= 0.0:
        raise WaSaBOperationPlanError(
            "Ray-plane intersection is behind the camera; check Hand-Eye result and plane Z"
        )

    tcp_plane_intersection_base = camera_origin_base + ray_scale_mm * ray_direction_base
    tcp_target_base = tcp_plane_intersection_base.copy()
    label = str(detection.get("label", ""))
    target_z_offset_mm = target_z_offset_for_label(label, settings)
    try:
        target_z_offset_add_mm = float(detection.get("target_z_offset_add_mm", 0.0))
    except (TypeError, ValueError) as exc:
        raise WaSaBOperationPlanError("target_z_offset_add_mm must be numeric") from exc
    if not np.isfinite(target_z_offset_add_mm):
        raise WaSaBOperationPlanError("target_z_offset_add_mm must be finite")
    applied_target_z_offset_mm = target_z_offset_mm + target_z_offset_add_mm
    tcp_target_base[2] += applied_target_z_offset_mm
    target_base_offset_raw = detection.get("target_base_offset_mm", [0.0, 0.0, 0.0])
    try:
        target_base_offset_mm = np.asarray(target_base_offset_raw, dtype=np.float64)
    except (TypeError, ValueError) as exc:
        raise WaSaBOperationPlanError("target_base_offset_mm must be three numeric values") from exc
    if target_base_offset_mm.shape != (3,):
        raise WaSaBOperationPlanError("target_base_offset_mm must contain exactly three values")
    if not np.isfinite(target_base_offset_mm).all():
        raise WaSaBOperationPlanError("target_base_offset_mm values must be finite")
    tcp_target_base += target_base_offset_mm

    # 목표 자세는 설정값이 있으면 보정된 pick orientation을 사용합니다.
    # 촬영 시점 카메라 위치 계산은 위에서 current_flange_coords를 그대로 사용합니다.
    orientation_override = detection.get("flange_orientation_deg")
    if orientation_override is not None:
        try:
            orientation_values = [float(value) for value in orientation_override]
        except (TypeError, ValueError) as exc:
            raise WaSaBOperationPlanError("flange_orientation_deg must contain three numbers") from exc
        if len(orientation_values) != 3 or not np.isfinite(orientation_values).all():
            raise WaSaBOperationPlanError("flange_orientation_deg must contain three finite values")
        rx, ry, rz = orientation_values
    elif settings.pick_flange_orientation_deg is not None:
        rx, ry, rz = settings.pick_flange_orientation_deg
    else:
        rx, ry, rz = (float(v) for v in current_flange_coords[3:])
    (
        bbox_aspect_ratio,
        gripper_rz_offset_deg,
        gripper_auto_rotated,
        object_axis_image_deg,
        gripper_angle_source,
    ) = _gripper_rz_offset_for_detection(
        detection,
        bbox,
        settings,
    )
    rz = _normalize_angle_deg(rz + gripper_rz_offset_deg)
    R_base_flange = euler_to_rotation_matrix(rx, ry, rz, settings.euler_order)
    tcp_offset_flange = np.asarray(settings.tcp_offset_flange_to_tcp_mm, dtype=np.float64)
    tcp_offset_base = R_base_flange @ tcp_offset_flange
    flange_target_base = tcp_target_base - tcp_offset_base

    flange_command = [
        round(float(flange_target_base[0]), 2),
        round(float(flange_target_base[1]), 2),
        round(float(flange_target_base[2]), 2),
        round(rx, 2),
        round(ry, 2),
        round(rz, 2),
    ]

    return {
        "detection": {
            "label": str(detection["label"]),
            "class_id": int(detection["class_id"]),
            "confidence": round(float(detection["confidence"]), 6),
            "bbox": [round(value, 3) for value in bbox],
            "midpoint_uv": [round(u, 3), round(v, 3)],
            "target_center_source": target_center_source,
        },
        "plan": {
            "tcp_plane_intersection_base_mm": [round(float(value), 3) for value in tcp_plane_intersection_base],
            "tcp_target_base_mm": [round(float(value), 3) for value in tcp_target_base],
            "flange_target_base_mm": [round(float(value), 3) for value in flange_target_base],
            "flange_command": flange_command,
        },
        "debug": {
            "camera_origin_base_mm": [round(float(value), 3) for value in camera_origin_base],
            "ray_direction_base": [round(float(value), 7) for value in ray_direction_base],
            "ray_scale_mm": round(float(ray_scale_mm), 3),
            "target_z_offset_mm": round(float(applied_target_z_offset_mm), 3),
            "base_target_z_offset_mm": round(float(target_z_offset_mm), 3),
            "target_z_offset_add_mm": round(float(target_z_offset_add_mm), 3),
            "target_base_offset_mm": [round(float(value), 3) for value in target_base_offset_mm],
            "target_center_source": target_center_source,
            "pick_flange_orientation_source": (
                "configured" if settings.pick_flange_orientation_deg is not None else "current_flange"
            ),
            "bbox_aspect_ratio_width_over_height": round(float(bbox_aspect_ratio), 3),
            "gripper_auto_rotated": bool(gripper_auto_rotated),
            "gripper_rz_offset_deg": round(float(gripper_rz_offset_deg), 3),
            "object_axis_image_deg": (
                round(float(object_axis_image_deg), 3)
                if object_axis_image_deg is not None
                else None
            ),
            "gripper_angle_source": gripper_angle_source,
            "tcp_target_z_delta_from_plane_mm": round(
                float(tcp_target_base[2] - object_plane_z_base_mm),
                3,
            ),
            "tcp_target_z_delta_from_intersection_mm": round(
                float(tcp_target_base[2] - tcp_plane_intersection_base[2]),
                3,
            ),
            "tcp_offset_in_base_mm": [round(float(value), 3) for value in tcp_offset_base],
        },
    }
