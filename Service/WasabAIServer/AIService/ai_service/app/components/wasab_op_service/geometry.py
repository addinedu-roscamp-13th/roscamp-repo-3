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


def image_pixel_to_camera_ray(u: float, v: float, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    pixel = np.array([[[u, v]]], dtype=np.float64)
    undistorted = cv2.undistortPoints(pixel, K, dist).reshape(2)
    ray = np.array([undistorted[0], undistorted[1], 1.0], dtype=np.float64)
    length = float(np.linalg.norm(ray))
    if length < 1e-12:
        raise WaSaBOperationPlanError("Invalid camera ray")
    return ray / length


def compute_wasab_operation_plan(
    *,
    detection: dict[str, Any],
    current_flange_coords: list[float],
    calibration: WaSaBCalibration,
    settings: Settings,
) -> dict[str, Any]:
    """서버에서 bbox 하나를 최종 MyCobot Flange 명령으로 변환합니다."""
    bbox = [float(v) for v in detection["bbox"]]
    u, v = detection_bbox_center_pixel(bbox)

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

    ray_scale_mm = (
        settings.object_plane_z_base_mm - float(camera_origin_base[2])
    ) / denominator
    if ray_scale_mm <= 0.0:
        raise WaSaBOperationPlanError(
            "Ray-plane intersection is behind the camera; check Hand-Eye result and plane Z"
        )

    tcp_target_base = camera_origin_base + ray_scale_mm * ray_direction_base
    label = str(detection.get("label", ""))
    target_z_offset_mm = settings.target_z_offsets_mm.get(
        label,
        settings.toothbrush_target_z_offset_mm,
    )
    tcp_target_base[2] += target_z_offset_mm

    # 목표 자세는 현재 flange 자세를 유지한다는 기존 로직을 그대로 사용합니다.
    rx, ry, rz = (float(v) for v in current_flange_coords[3:])
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
        },
        "plan": {
            "tcp_target_base_mm": [round(float(value), 3) for value in tcp_target_base],
            "flange_target_base_mm": [round(float(value), 3) for value in flange_target_base],
            "flange_command": flange_command,
        },
        "debug": {
            "camera_origin_base_mm": [round(float(value), 3) for value in camera_origin_base],
            "ray_direction_base": [round(float(value), 7) for value in ray_direction_base],
            "ray_scale_mm": round(float(ray_scale_mm), 3),
            "target_z_offset_mm": round(float(target_z_offset_mm), 3),
            "tcp_offset_in_base_mm": [round(float(value), 3) for value in tcp_offset_base],
        },
    }
