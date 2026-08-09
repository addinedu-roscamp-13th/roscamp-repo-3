#!/usr/bin/env python3
# encoding: utf-8
"""노트북 로컬 YOLO 검출 + 3D 파지계획 FastAPI 서비스.

라즈베리파이는 SSH 터널 없이 노트북의 LAN IP로 프레임과 현재 Flange pose를
전송한다. ``/detect``, ``/grasp-plan``, ``/v1/grasp-plan`` 요청 형식과
응답 형식은 기존 원격 딥러닝 서버와 호환된다.
"""
from __future__ import annotations

import json
import os
import signal
import socket
import struct
import subprocess
import threading
import time
from contextlib import asynccontextmanager
from dataclasses import replace
from datetime import datetime
from pathlib import Path
from threading import Condition, Lock
from typing import Any, Optional

import cv2
import numpy as np
from fastapi import FastAPI, File, Form, HTTPException, Query, UploadFile
from fastapi.responses import HTMLResponse, Response, StreamingResponse
from pydantic import BaseModel
from ultralytics import YOLO

from app.components.wasab_op_service.geometry import (
    WaSaBCalibration,
    WaSaBOperationPlanError,
    compute_wasab_operation_plan,
    euler_to_rotation_matrix,
    load_wasab_calibration,
    pick_target_base_offset_for_label,
    target_z_offset_for_label,
    wasab_arm_pose_to_T_base_flange,
)
from app.settings import DEFAULT_CONFIG_PATH, settings
from app.components.wasab_web_service.commands import COMMAND_ALIASES
from app.components.wasab_web_service.dual_arm import dual_arm_status, runtime as dual_arm_runtime


# ============================================================
# 1. 응답 형식: 기존 /detect API 호환
# ============================================================

class WaSaBObjectDetection(BaseModel):
    label: str
    class_id: int
    confidence: float
    bbox: list[float]      # [x1, y1, x2, y2]
    center: list[float]    # [u, v]
    width: float
    height: float
    grip_axis_image_deg: Optional[float] = None
    grip_axis_source: Optional[str] = None
    grip_center_source: Optional[str] = None
    grip_axis_endpoints: Optional[list[list[float]]] = None
    grip_axis_endpoints_source: Optional[str] = None


class WaSaBInferResponse(BaseModel):
    status: str
    image_width: int
    image_height: int
    inference_ms: float
    detections: list[WaSaBObjectDetection]
    saved_dir: Optional[str] = None
    raw_image_path: Optional[str] = None
    annotated_image_path: Optional[str] = None
    result_json_path: Optional[str] = None


class MarkerPlacePlanRequest(BaseModel):
    request_id: Optional[str] = None
    flange_coords: list[float]
    marker_detection: dict[str, Any]
    picked_target_label: Optional[str] = None
    target_base_offset_mm: Optional[list[float]] = None


class MarkerPickupPlanRequest(BaseModel):
    request_id: Optional[str] = None
    flange_coords: list[float]
    marker_detection: dict[str, Any]
    marker_plane_z_base_mm: float = 46.1
    target_z_offset_mm: float = 30.0
    target_base_offset_mm: list[float] = [0.0, 0.0, 0.0]
    flange_orientation_deg: list[float]


class LatestFrameGraspPlanRequest(BaseModel):
    request_id: Optional[str] = None
    flange_coords: list[float]


class TargetZOffsetsUpdate(BaseModel):
    default_target_z_offset_mm: Optional[float] = None
    class_target_z_offsets_mm: dict[str, float] | None = None


class TcpOffsetUpdate(BaseModel):
    tcp_offset_flange_to_tcp_mm: list[float]

class HandoverZoneUpdate(BaseModel):
    x_min_norm: float
    x_max_norm: float
    y_min_norm: float
    y_max_norm: float


class RobotLogEvent(BaseModel):
    level: str = "info"
    message: str
    source: str = "robot-client"


class FireResponseRequest(BaseModel):
    response: str


class WorkspaceOverlayUpdate(BaseModel):
    flange_coords: list[float]
    safe_x_mm: list[float]
    safe_y_mm: list[float]
    safe_z_mm: list[float]
    object_plane_z_base_mm: float
    target_z_offset_mm: float
    target_base_offset_mm: list[float]
    flange_orientation_deg: list[float]


OVERLAP_SUPPRESSION_IOU_THRESHOLD = 0.7
PICK_BBOX_EDGE_MARGIN_PX = 30.0
APRILTAG_PICKUP_IDS = {0, 1, 3, 4, 6}
APRILTAG_PLACE_IDS = {7, 8, 9}


# ============================================================
# 2. 서버 상태
# ============================================================

class WaSaBServiceState:
    model: YOLO
    calibration: WaSaBCalibration
    inference_lock: Lock
    latest_frame_lock: Lock
    latest_frame_jpeg: bytes | None
    latest_frame_meta: dict[str, Any] | None
    arm_frames: dict[str, dict[str, Any]]
    arm_detection_overlays: dict[str, dict[str, Any]]
    arm_workspace_overlays: dict[str, dict[str, Any] | None]
    detection_overlay_until: float
    detection_overlay_detections: list[WaSaBObjectDetection]
    detection_overlay_summary: str | None
    command_lock: Lock
    command_condition: Condition
    command_queue: list[dict[str, Any]]
    command_seq: int
    operation_logs: list[dict[str, Any]]
    gesture_enabled: bool
    target_z_offsets_lock: Lock
    tcp_offset_lock: Lock
    udp_stream_stop: threading.Event | None
    udp_stream_thread: threading.Thread | None


wasab_service_state = WaSaBServiceState()

AI_SERVICE_ROOT = Path(__file__).resolve().parents[4]
ARM_FEATURE_ROOT = AI_SERVICE_ROOT / "face-recog"
ARM_FEATURE_NAMES = {"fire-detect", "face-recognition", "tracking"}
ARM_VISION_RATE_HZ = 8.0
FACE_FEATURE_PYTHON = Path("/home/ane/dev_ws/.venv-face/bin/python")
arm_feature_lock = Lock()
arm_feature_processes: dict[tuple[str, str], tuple[subprocess.Popen, Any]] = {}
arm_fire_prompts: dict[str, dict[str, Any] | None] = {"left": None, "right": None}
arm_face_prompts: dict[str, dict[str, Any] | None] = {"left": None, "right": None}
arm_face_last_detection: dict[str, dict[str, Any] | None] = {
    "left": None,
    "right": None,
}


def _fire_response_path(arm_id: str) -> Path:
    return Path("/tmp") / f"wasab_{arm_id}_fire_response.txt"


def _enqueue_vision_target(
    arm_id: str,
    feature: str,
    cx: float,
    cy: float,
    confidence: float,
) -> None:
    # wasab_통합 search_node/fire_search_node both use 0.3 as the tracking
    # confidence threshold.  Do not move the arm for early 0.2 vote ramps.
    if confidence < 0.3:
        return
    command = (
        f"vision-track:{feature}:{cx:.6f}:{cy:.6f}:{confidence:.6f}"
    )
    now = time.time()
    with wasab_service_state.command_lock:
        # Tracking coordinates are real-time state, not a FIFO workload.
        wasab_service_state.command_queue[:] = [
            item
            for item in wasab_service_state.command_queue
            if not (
                item.get("arm_id") == arm_id
                and str(item.get("command", "")).startswith("vision-track:")
            )
        ]
        wasab_service_state.command_seq += 1
        wasab_service_state.command_queue.append({
            "id": wasab_service_state.command_seq,
            "command": command,
            "timestamp": now,
            "timestamp_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
            "source": f"vision:{feature}",
            "arm_id": arm_id,
        })
        wasab_service_state.command_condition.notify_all()


def _arm_feature_log_worker(
    arm_id: str,
    feature: str,
    log_path: Path,
    start_offset: int,
    process: subprocess.Popen,
) -> None:
    """Forward live detection summaries into the AdminGUI operation log."""
    try:
        with log_path.open("r", encoding="utf-8", errors="replace") as stream:
            stream.seek(start_offset)
            while process.poll() is None:
                line = stream.readline()
                if not line:
                    time.sleep(0.15)
                    continue
                message = line.strip()
                if not message:
                    continue
                if message.startswith("VISION_TARGET "):
                    try:
                        values = dict(
                            token.split("=", 1)
                            for token in message.split()[1:]
                        )
                        _enqueue_vision_target(
                            arm_id,
                            feature,
                            float(values["cx"]),
                            float(values["cy"]),
                            float(values["conf"]),
                        )
                    except (KeyError, ValueError):
                        pass
                    continue
                if feature == "face-recognition" and message.startswith("FACE_RESULT "):
                    try:
                        values = dict(
                            token.split("=", 1)
                            for token in message.split()[1:]
                        )
                        registered = values["registered"] == "1"
                        teacher = values.get("name", "None")
                        identity = (
                            f"registered:{teacher}" if registered else "unregistered"
                        )
                        now = time.monotonic()
                        with arm_feature_lock:
                            previous = arm_face_last_detection[arm_id]
                            should_prompt = (
                                previous is None
                                or previous["identity"] != identity
                                or now - previous["last_seen"] >= 5.0
                            )
                            arm_face_last_detection[arm_id] = {
                                "identity": identity,
                                "last_seen": now,
                            }
                            if should_prompt:
                                arm_face_prompts[arm_id] = {
                                    "id": time.time_ns(),
                                    "arm_id": arm_id,
                                    "registered": registered,
                                    "name": teacher if registered else None,
                                    "title": "얼굴인식 결과",
                                    "message": (
                                        f"등록된 사람입니다: {teacher}"
                                        if registered
                                        else "미등록된 사람입니다."
                                    ),
                                }
                    except (KeyError, ValueError):
                        pass
                    continue
                if feature == "face-recognition":
                    visible = "[perception]: faces=" in message
                elif feature == "tracking":
                    visible = "unknown_face_tracker]: faces=" in message
                else:
                    visible = "[fire_event]" in message
                if not visible:
                    continue
                level = "warning" if feature == "fire-detect" else "info"
                with wasab_service_state.command_lock:
                    _append_operation_log_locked(
                        level,
                        message,
                        f"vision:{arm_id}:{feature}",
                    )
                if feature == "fire-detect":
                    release_gripper = False
                    with arm_feature_lock:
                        if "진압을 시작할까요?" in message:
                            arm_fire_prompts[arm_id] = {
                                "id": time.time_ns(),
                                "arm_id": arm_id,
                                "kind": "suppress",
                                "title": "화재 감지",
                                "message": "진압을 시작할까요?",
                                "yes_label": "진압 시작",
                                "no_label": "아니오",
                            }
                        elif any(text in message for text in ("진압 성공", "진압 실패")):
                            result = "진압에 성공했습니다." if "진압 성공" in message else "진압에 실패했습니다."
                            arm_fire_prompts[arm_id] = {
                                "id": time.time_ns(),
                                "arm_id": arm_id,
                                "kind": "post-action",
                                "title": "상황 종료",
                                "message": f"{result} 다음 동작을 선택해 주세요.",
                                "yes_label": "순찰 재개",
                                "no_label": "홈으로 복귀",
                            }
                            release_gripper = True
                        elif "화재를 진압합니다" in message:
                            arm_fire_prompts[arm_id] = None
                        elif "순찰을 재개합니다" in message:
                            # Decline/response-timeout path: no suppression was
                            # started, so resume patrol and ensure a stale
                            # gripper state cannot remain closed.
                            arm_fire_prompts[arm_id] = None
                            release_gripper = True
                    if release_gripper:
                        enqueue_wasab_arm_command("fire-suppress-open", arm_id)
    except OSError as exc:
        with wasab_service_state.command_lock:
            _append_operation_log_locked(
                "error",
                f"feature log unavailable: {exc}",
                f"vision:{arm_id}:{feature}",
            )


def _arm_feature_command(arm_id: str, feature: str) -> list[str]:
    camera_stream = (
        f"http://{settings.host}:{settings.port}"
        f"/camera-frame/stream.mjpg?arm_id={arm_id}"
    )
    if feature == "fire-detect":
        command = [
        "/usr/bin/python3",
        str(
            ARM_FEATURE_ROOT
            / ("left_arm_fire_detect.py" if arm_id == "left" else "fire_detect_node.py")
        ),
        "--source",
        camera_stream,
        "--console-domain",
        "50",
        "--response-file",
        str(_fire_response_path(arm_id)),
        "--rate",
        str(ARM_VISION_RATE_HZ),
        ]
        if arm_id == "left":
            command.append("--flip")
        return command
    if feature == "tracking":
        return [
            "/usr/bin/python3",
            str(ARM_FEATURE_ROOT / "unknown_face_tracker_node.py"),
            "--source",
            camera_stream,
            "--no-mirror",
            "--known-only",
            "--rate",
            str(ARM_VISION_RATE_HZ),
        ]
    return [
        str(FACE_FEATURE_PYTHON),
        "-u",
        str(ARM_FEATURE_ROOT / "perception_node.py"),
        "--source",
        camera_stream,
        "--rate",
        str(ARM_VISION_RATE_HZ),
    ] + ([] if arm_id == "left" else ["--no-mirror"])


def _arm_feature_is_running(arm_id: str, feature: str) -> bool:
    key = (arm_id, feature)
    entry = arm_feature_processes.get(key)
    if entry is None:
        return False
    if entry[0].poll() is None:
        return True
    arm_feature_processes.pop(key, None)
    entry[1].close()
    return False


def _stop_arm_feature(arm_id: str, feature: str) -> None:
    entry = arm_feature_processes.pop((arm_id, feature), None)
    if entry is None:
        return
    if feature == "fire-detect":
        arm_fire_prompts[arm_id] = None
        _fire_response_path(arm_id).unlink(missing_ok=True)
    elif feature == "face-recognition":
        arm_face_prompts[arm_id] = None
        arm_face_last_detection[arm_id] = None
    process, log_file = entry
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=1.0)
        except ProcessLookupError:
            pass
    log_file.close()


def _start_arm_feature(arm_id: str, feature: str) -> subprocess.Popen:
    command = _arm_feature_command(arm_id, feature)
    log_path = Path("/tmp") / f"wasab_{arm_id}_{feature.replace('-', '_')}.log"
    if feature == "fire-detect":
        _fire_response_path(arm_id).unlink(missing_ok=True)
        arm_fire_prompts[arm_id] = None
    elif feature == "face-recognition":
        arm_face_prompts[arm_id] = None
        arm_face_last_detection[arm_id] = None
    start_offset = log_path.stat().st_size if log_path.exists() else 0
    log_file = log_path.open("ab", buffering=0)
    environment = os.environ.copy()
    environment["ROS_DOMAIN_ID"] = "69"
    environment["MPLCONFIGDIR"] = "/tmp/wasab-matplotlib"
    try:
        process = subprocess.Popen(
            command,
            cwd=ARM_FEATURE_ROOT,
            env=environment,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except Exception:
        log_file.close()
        raise
    arm_feature_processes[(arm_id, feature)] = (process, log_file)
    threading.Thread(
        target=_arm_feature_log_worker,
        args=(arm_id, feature, log_path, start_offset, process),
        daemon=True,
        name=f"{arm_id}-{feature}-log",
    ).start()
    return process


@asynccontextmanager
async def lifespan(_: FastAPI):
    if settings.euler_order != "zyx":
        raise RuntimeError("This project currently supports only EULER_ORDER=zyx")
    if not settings.model_path.exists():
        raise FileNotFoundError(f"YOLO model not found: {settings.model_path}")

    # 모델과 calibration은 서버 시작 시 한 번만 로드합니다.
    wasab_service_state.model = YOLO(str(settings.model_path))
    wasab_service_state.calibration = load_wasab_calibration(settings)
    wasab_service_state.arm_calibrations = {
        "left": wasab_service_state.calibration,
    }
    if settings.right_intrinsic_file and settings.right_handeye_result_json:
        right_settings = replace(
            settings,
            intrinsic_file=settings.right_intrinsic_file,
            handeye_result_json=settings.right_handeye_result_json,
        )
        wasab_service_state.arm_calibrations["right"] = load_wasab_calibration(
            right_settings
        )
    wasab_service_state.inference_lock = Lock()
    wasab_service_state.latest_frame_lock = Lock()
    wasab_service_state.latest_frame_jpeg = None
    wasab_service_state.latest_frame_meta = None
    wasab_service_state.arm_frames = {
        "left": {"jpeg": None, "meta": None},
        "right": {"jpeg": None, "meta": None},
    }
    wasab_service_state.arm_detection_overlays = {
        arm_id: {"until": 0.0, "detections": [], "summary": None}
        for arm_id in ("left", "right")
    }
    wasab_service_state.arm_workspace_overlays = {
        "left": None,
        "right": None,
    }
    wasab_service_state.detection_overlay_until = 0.0
    wasab_service_state.detection_overlay_detections = []
    wasab_service_state.detection_overlay_summary = None
    wasab_service_state.command_lock = Lock()
    wasab_service_state.command_condition = Condition(wasab_service_state.command_lock)
    wasab_service_state.command_queue = []
    wasab_service_state.command_seq = 0
    wasab_service_state.operation_logs = []
    wasab_service_state.gesture_enabled = False
    wasab_service_state.target_z_offsets_lock = Lock()
    wasab_service_state.tcp_offset_lock = Lock()
    wasab_service_state.udp_stream_stop = None
    wasab_service_state.udp_stream_thread = None
    if settings.udp_stream_enabled:
        wasab_service_state.udp_stream_stop = threading.Event()
        wasab_service_state.udp_stream_thread = threading.Thread(
            target=run_udp_streamer_receiver,
            args=(wasab_service_state.udp_stream_stop,),
            daemon=True,
        )
        wasab_service_state.udp_stream_thread.start()

    print("[STARTUP] YOLO model:", settings.model_path)
    print("[STARTUP] device:", settings.device)
    print("[STARTUP] intrinsic:", settings.intrinsic_file)
    print("[STARTUP] hand-eye:", settings.handeye_result_json)
    print("[STARTUP] hand-eye method:", wasab_service_state.calibration.selected_method)
    if "right" in wasab_service_state.arm_calibrations:
        print(
            "[STARTUP] right intrinsic:",
            settings.right_intrinsic_file,
        )
        print(
            "[STARTUP] right hand-eye:",
            settings.right_handeye_result_json,
        )
        print(
            "[STARTUP] right hand-eye method:",
            wasab_service_state.arm_calibrations["right"].selected_method,
        )
    if settings.udp_stream_enabled:
        print("[STARTUP] UDP Streamer receiver:", f"{settings.udp_stream_host}:{settings.udp_stream_port}")
    try:
        yield
    finally:
        with arm_feature_lock:
            for arm_id, feature in list(arm_feature_processes):
                _stop_arm_feature(arm_id, feature)
        if wasab_service_state.udp_stream_stop is not None:
            wasab_service_state.udp_stream_stop.set()
        if wasab_service_state.udp_stream_thread is not None:
            wasab_service_state.udp_stream_thread.join(timeout=1.0)


app = FastAPI(title="WaSaBWebService + WaSaBOPService", version="2.1.0", lifespan=lifespan)


# ============================================================
# 3. 기본 API
# ============================================================

@app.get("/health")
def wasab_health() -> dict[str, Any]:
    return {
        "status": "ok",
        "runtime": "laptop-local",
        "model_path": str(settings.model_path),
        "device": settings.device,
        "default_conf": settings.default_conf,
        "pick_flange_orientation_deg": (
            list(settings.pick_flange_orientation_deg)
            if settings.pick_flange_orientation_deg is not None
            else None
        ),
        "tcp_offset_flange_to_tcp_mm": list(settings.tcp_offset_flange_to_tcp_mm),
        "gripper_orientation_mode": "image_axis_dynamic",
        "gripper_auto_rotate_long_bbox_enabled": settings.gripper_auto_rotate_long_bbox_enabled,
        "gripper_auto_rotate_aspect_ratio": settings.gripper_auto_rotate_aspect_ratio,
        "gripper_auto_rotate_rz_offset_deg": settings.gripper_auto_rotate_rz_offset_deg,
        "euler_order": settings.euler_order,
        "calibration_method": wasab_service_state.calibration.selected_method,
    }


@app.get("/arm-features/{arm_id}/status")
def arm_features_status(arm_id: str) -> dict[str, Any]:
    if arm_id not in {"left", "right"}:
        raise HTTPException(status_code=404, detail=f"unknown arm: {arm_id}")
    with arm_feature_lock:
        status = {}
        for feature in ARM_FEATURE_NAMES:
            running = _arm_feature_is_running(arm_id, feature)
            status[feature] = {
                "running": running,
                "pid": arm_feature_processes[(arm_id, feature)][0].pid if running else None,
            }
        return status


@app.post("/arm-features/{arm_id}/{feature}/toggle")
def toggle_arm_feature(arm_id: str, feature: str) -> dict[str, Any]:
    if arm_id not in {"left", "right"}:
        raise HTTPException(status_code=404, detail=f"unknown arm: {arm_id}")
    if feature not in ARM_FEATURE_NAMES:
        raise HTTPException(status_code=404, detail=f"unknown feature: {feature}")
    with arm_feature_lock:
        if _arm_feature_is_running(arm_id, feature):
            _stop_arm_feature(arm_id, feature)
            enqueue_wasab_arm_command("vision-sweep-off", arm_id)
            return {"arm_id": arm_id, "feature": feature, "running": False, "pid": None}
        stopped_features = []
        for other_feature in ARM_FEATURE_NAMES - {feature}:
            if _arm_feature_is_running(arm_id, other_feature):
                _stop_arm_feature(arm_id, other_feature)
                stopped_features.append(other_feature)
        process = _start_arm_feature(arm_id, feature)
        sweep_command = (
            "vision-sweep-fire-on"
            if feature == "fire-detect"
            else (
                "vision-sweep-tracking-on"
                if feature == "tracking"
                else "vision-sweep-face-on"
            )
        )
        enqueue_wasab_arm_command(sweep_command, arm_id)
        return {
            "arm_id": arm_id,
            "feature": feature,
            "running": True,
            "pid": process.pid,
            "stopped_features": stopped_features,
        }


@app.get("/arm-features/{arm_id}/fire-prompt")
def get_fire_prompt(arm_id: str) -> dict[str, Any]:
    if arm_id not in {"left", "right"}:
        raise HTTPException(status_code=404, detail=f"unknown arm: {arm_id}")
    with arm_feature_lock:
        return {"prompt": arm_fire_prompts[arm_id]}


@app.get("/arm-features/{arm_id}/face-prompt")
def get_face_prompt(arm_id: str) -> dict[str, Any]:
    if arm_id not in {"left", "right"}:
        raise HTTPException(status_code=404, detail=f"unknown arm: {arm_id}")
    with arm_feature_lock:
        return {"prompt": arm_face_prompts[arm_id]}


@app.post("/arm-features/{arm_id}/face-prompt/ack")
def acknowledge_face_prompt(arm_id: str) -> dict[str, Any]:
    if arm_id not in {"left", "right"}:
        raise HTTPException(status_code=404, detail=f"unknown arm: {arm_id}")
    with arm_feature_lock:
        arm_face_prompts[arm_id] = None
    return {"status": "acknowledged", "arm_id": arm_id}


@app.post("/arm-features/{arm_id}/fire-response")
def submit_fire_response(
    arm_id: str,
    request: FireResponseRequest,
) -> dict[str, Any]:
    if arm_id not in {"left", "right"}:
        raise HTTPException(status_code=404, detail=f"unknown arm: {arm_id}")
    response = request.response.strip().lower()
    if response not in {"yes", "no"}:
        raise HTTPException(status_code=400, detail="response must be yes or no")
    post_action = False
    with arm_feature_lock:
        if not _arm_feature_is_running(arm_id, "fire-detect"):
            raise HTTPException(status_code=409, detail="fire detection is not running")
        prompt = arm_fire_prompts[arm_id]
        if prompt is None:
            raise HTTPException(status_code=409, detail="fire prompt is no longer active")
        post_action = prompt.get("kind") == "post-action"
        if not post_action:
            _fire_response_path(arm_id).write_text(response, encoding="utf-8")
        elif response == "no":
            _stop_arm_feature(arm_id, "fire-detect")
        arm_fire_prompts[arm_id] = None
    if post_action:
        enqueue_wasab_arm_command("fire-suppress-open", arm_id)
        if response == "yes":
            enqueue_wasab_arm_command("vision-sweep-fire-on", arm_id)
        else:
            enqueue_wasab_arm_command("vision-sweep-off", arm_id)
            enqueue_wasab_arm_command("home", arm_id)
    elif response == "yes":
        enqueue_wasab_arm_command("fire-suppress-close", arm_id)
    return {
        "status": "accepted",
        "arm_id": arm_id,
        "response": response,
        "prompt_kind": "post-action" if post_action else "suppress",
    }


def _target_z_offsets_payload() -> dict[str, Any]:
    with wasab_service_state.target_z_offsets_lock:
        return {
            "status": "ok",
            "object_plane_z_base_mm": settings.object_plane_z_base_mm,
            "default_target_z_offset_mm": settings.default_target_z_offset_mm,
            "class_target_z_offsets_mm": dict(settings.target_z_offsets_mm),
            "effective_target_z_mm": {
                label: settings.object_plane_z_base_mm + offset
                for label, offset in settings.target_z_offsets_mm.items()
            },
        }


def _tcp_offset_payload() -> dict[str, Any]:
    with wasab_service_state.tcp_offset_lock:
        return {
            "status": "ok",
            "tcp_offset_flange_to_tcp_mm": [
                round(float(value), 3)
                for value in settings.tcp_offset_flange_to_tcp_mm
            ],
        }


def _replace_config_option(
    lines: list[str],
    section: str,
    option: str,
    value: str,
) -> list[str]:
    current_section: str | None = None
    inserted = False
    result: list[str] = []
    section_found = False

    for line in lines:
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            if current_section == section and not inserted:
                result.append(f"{option} = {value}\n")
                inserted = True
            current_section = stripped[1:-1].strip()
            if current_section == section:
                section_found = True

        if current_section == section and stripped.startswith(f"{option}"):
            prefix = stripped.split("=", 1)[0].strip()
            if prefix == option:
                result.append(f"{option} = {value}\n")
                inserted = True
                continue

        result.append(line)

    if section_found and current_section == section and not inserted:
        result.append(f"{option} = {value}\n")
    if not section_found:
        if result and result[-1].strip():
            result.append("\n")
        result.append(f"[{section}]\n")
        result.append(f"{option} = {value}\n")
    return result


def _remove_config_option(lines: list[str], section: str, option: str) -> list[str]:
    current_section: str | None = None
    result: list[str] = []
    for line in lines:
        stripped = line.strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            current_section = stripped[1:-1].strip()
        if current_section == section and stripped.startswith(f"{option}"):
            prefix = stripped.split("=", 1)[0].strip()
            if prefix == option:
                continue
        result.append(line)
    return result


def _replace_target_z_offsets_section(lines: list[str]) -> list[str]:
    section_start: int | None = None
    section_end = len(lines)
    for index, line in enumerate(lines):
        stripped = line.strip()
        if stripped == "[target_z_offsets_mm]":
            section_start = index
            continue
        if section_start is not None and index > section_start and stripped.startswith("[") and stripped.endswith("]"):
            section_end = index
            break

    replacement = ["[target_z_offsets_mm]\n"]
    for label, offset in sorted(settings.target_z_offsets_mm.items()):
        replacement.append(f"{label} = {offset:.3f}\n")

    if section_start is None:
        result = list(lines)
        if result and result[-1].strip():
            result.append("\n")
        result.extend(replacement)
        return result

    return lines[:section_start] + replacement + lines[section_end:]


def _persist_target_z_offsets_to_config() -> None:
    lines = DEFAULT_CONFIG_PATH.read_text(encoding="utf-8").splitlines(keepends=True)
    lines = _replace_config_option(
        lines,
        "calibration",
        "default_target_z_offset_mm",
        f"{settings.default_target_z_offset_mm:.3f}",
    )
    lines = _replace_target_z_offsets_section(lines)
    DEFAULT_CONFIG_PATH.write_text("".join(lines), encoding="utf-8")


def _persist_tcp_offset_to_config() -> None:
    lines = DEFAULT_CONFIG_PATH.read_text(encoding="utf-8").splitlines(keepends=True)
    value = ", ".join(f"{float(item):.3f}" for item in settings.tcp_offset_flange_to_tcp_mm)
    lines = _replace_config_option(
        lines,
        "calibration",
        "tcp_offset_flange_to_tcp_mm",
        value,
    )
    DEFAULT_CONFIG_PATH.write_text("".join(lines), encoding="utf-8")


def _persist_handover_zone_to_config() -> None:
    lines = DEFAULT_CONFIG_PATH.read_text(encoding="utf-8").splitlines(keepends=True)
    names = ("x_min_norm", "x_max_norm", "y_min_norm", "y_max_norm")
    for name, value in zip(names, settings.handover_zone_norm):
        lines = _replace_config_option(lines, "handover_zone", name, f"{value:.6f}")
    DEFAULT_CONFIG_PATH.write_text("".join(lines), encoding="utf-8")


@app.get("/settings/handover-zone")
def get_handover_zone() -> dict[str, Any]:
    x_min, x_max, y_min, y_max = settings.handover_zone_norm
    return {
        "x_min_norm": x_min,
        "x_max_norm": x_max,
        "y_min_norm": y_min,
        "y_max_norm": y_max,
    }


@app.patch("/settings/handover-zone")
def update_handover_zone(update: HandoverZoneUpdate) -> dict[str, Any]:
    zone = (
        float(update.x_min_norm),
        float(update.x_max_norm),
        float(update.y_min_norm),
        float(update.y_max_norm),
    )
    if not all(np.isfinite(value) for value in zone):
        raise HTTPException(status_code=400, detail="handover zone must be finite")
    x_min, x_max, y_min, y_max = zone
    if not (0.0 <= x_min < x_max <= 1.0 and 0.0 <= y_min < y_max <= 1.0):
        raise HTTPException(
            status_code=400,
            detail="handover zone must satisfy 0 <= min < max <= 1",
        )
    if x_max - x_min < 0.1 or y_max - y_min < 0.1:
        raise HTTPException(status_code=400, detail="handover zone is too small")
    object.__setattr__(settings, "handover_zone_norm", zone)
    _persist_handover_zone_to_config()
    return get_handover_zone()


@app.get("/settings/target-z-offsets")
def get_target_z_offsets() -> dict[str, Any]:
    return _target_z_offsets_payload()


@app.patch("/settings/target-z-offsets")
def update_target_z_offsets(update: TargetZOffsetsUpdate) -> dict[str, Any]:
    with wasab_service_state.target_z_offsets_lock:
        if update.default_target_z_offset_mm is not None:
            value = float(update.default_target_z_offset_mm)
            if not np.isfinite(value):
                raise HTTPException(status_code=400, detail="default_target_z_offset_mm must be finite")
            object.__setattr__(settings, "default_target_z_offset_mm", value)

        if update.class_target_z_offsets_mm is not None:
            for raw_label, raw_value in update.class_target_z_offsets_mm.items():
                label = raw_label.strip()
                if not label:
                    raise HTTPException(status_code=400, detail="target label cannot be empty")
                value = float(raw_value)
                if not np.isfinite(value):
                    raise HTTPException(status_code=400, detail=f"{label} offset must be finite")
                settings.target_z_offsets_mm[label] = value

        _persist_target_z_offsets_to_config()

    return _target_z_offsets_payload()


@app.get("/settings/tcp-offset")
def get_tcp_offset() -> dict[str, Any]:
    return _tcp_offset_payload()


@app.patch("/settings/tcp-offset")
def update_tcp_offset(update: TcpOffsetUpdate) -> dict[str, Any]:
    values = update.tcp_offset_flange_to_tcp_mm
    if len(values) != 3:
        raise HTTPException(status_code=400, detail="tcp_offset_flange_to_tcp_mm must contain three values")
    try:
        offset = tuple(float(value) for value in values)
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail="tcp_offset_flange_to_tcp_mm must be numeric") from exc
    if not all(np.isfinite(value) for value in offset):
        raise HTTPException(status_code=400, detail="tcp_offset_flange_to_tcp_mm values must be finite")

    with wasab_service_state.tcp_offset_lock:
        object.__setattr__(settings, "tcp_offset_flange_to_tcp_mm", offset)
        _persist_tcp_offset_to_config()

    return _tcp_offset_payload()


@app.get("/camera-view", response_class=HTMLResponse)
def admin_gui() -> str:
    """브라우저에서 로봇팔 카메라와 원격 동작 버튼을 확인합니다."""
    return """<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>WaSaB AdminGUI</title>
  <style>
    :root { color-scheme: dark; }
    body {
      margin: 0;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, \"Segoe UI\", sans-serif;
      background: #15191e;
      color: #e7ebef;
    }
    header {
      display: grid;
      grid-template-columns: minmax(250px, 1fr) auto minmax(620px, 1.8fr);
      gap: 12px;
      align-items: stretch;
      padding: 10px 12px;
      border-bottom: 1px solid #3b424a;
      background: #252a30;
      box-shadow: 0 2px 8px rgba(0, 0, 0, .28);
    }
    h1 {
      margin: 0;
      font-size: 17px;
      font-weight: 700;
      letter-spacing: .2px;
      color: #f3f6f8;
    }
    .left-controls {
      display: flex;
      flex-direction: column;
      align-items: stretch;
      gap: 7px;
    }
    .camera-controls {
      display: grid;
      grid-template-columns: repeat(3, minmax(68px, 1fr));
      gap: 6px;
      align-items: stretch;
    }
    #status {
      font-size: 14px;
      color: #bbb;
      white-space: nowrap;
      grid-column: 1 / -1;
      text-align: right;
      min-height: 16px;
    }
    .right-controls {
      display: grid;
      grid-template-columns: minmax(260px, .8fr) minmax(360px, 1.2fr);
      gap: 8px;
      align-items: stretch;
    }
    .utility-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(82px, 1fr));
      gap: 6px;
      width: 100%;
    }
    .operation-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(82px, 1fr));
      gap: 6px;
      width: 100%;
    }
    .center-safety {
      display: grid;
      grid-template-columns: repeat(3, minmax(82px, 1fr));
      gap: 8px;
      align-items: center;
      justify-content: center;
    }
    .tool-group {
      position: relative;
      border: 1px solid #414952;
      border-radius: 5px;
      background: #20252b;
      padding: 20px 8px 8px;
      min-width: 0;
      box-sizing: border-box;
    }
    .group-title {
      position: absolute;
      top: 4px;
      left: 9px;
      color: #9faab5;
      font-size: 10px;
      font-weight: 700;
      letter-spacing: .8px;
      text-transform: uppercase;
      pointer-events: none;
    }
    .utility-grid button, .operation-grid button {
      width: 100%;
    }
    button {
      border: 1px solid #4a535d;
      border-radius: 4px;
      background: linear-gradient(#343b43, #2a3037);
      color: #eef2f5;
      min-width: 42px;
      height: 31px;
      padding: 0 10px;
      font: inherit;
      font-size: 12px;
      font-weight: 600;
      cursor: pointer;
    }
    button:hover { background: #414a54; border-color: #65717d; }
    button:active { transform: translateY(1px); }
    button.danger {
      border-color: #7a2d2d;
      background: linear-gradient(#502a2a, #3b2020);
    }
    button.stop {
      border-color: #b91c1c;
      background: linear-gradient(#a32929, #7f1d1d);
      font-weight: 750;
    }
    button.gesture-on, button.feature-on { border-color: #15803d; background: #14532d; }
    button.gesture-off { border-color: #6b7280; background: #374151; }
    button[data-arm-mode].active {
      border-color: #4b83c5;
      background: linear-gradient(#3675b9, #285d96);
      color: white;
      font-weight: 700;
    }
    button.servo-released { border-color: #a16207; background: #713f12; }
    button:disabled { cursor: not-allowed; opacity: 0.45; }
    main {
      min-height: calc(100vh - 50px);
      display: grid;
      grid-template-columns: minmax(180px, 260px) minmax(0, 1fr) minmax(260px, 340px);
      gap: 16px;
      align-items: start;
      padding: 16px;
      box-sizing: border-box;
    }
    .camera-panel {
      display: flex;
      justify-content: center;
      min-width: 0;
      gap: 10px;
    }
    .camera-panel.dual { display: grid; grid-template-columns: repeat(2, minmax(0, 1fr)); }
    .camera-feed { min-width: 0; position: relative; display: flex; justify-content: center; }
    .camera-feed[hidden] { display: none; }
    .camera-label {
      display: none;
    }
    aside.log-panel {
      grid-column: 3;
      grid-row: 1;
      min-width: 0;
      box-sizing: border-box;
      display: flex;
      flex-direction: column;
    }
    .log-heading { display: flex; align-items: center; justify-content: space-between; gap: 8px; }
    .log-heading button { height: 28px; min-width: 0; padding: 0 9px; font-size: 12px; }
    #operationLogs {
      height: auto;
      min-height: 0;
      flex: 1;
      overflow: auto;
      margin: 0;
      padding: 8px;
      border: 1px solid #333;
      border-radius: 4px;
      background: #0b0b0b;
      color: #cbd5e1;
      font: 12px/1.5 ui-monospace, SFMono-Regular, Menlo, Consolas, monospace;
      white-space: pre-wrap;
      overflow-wrap: anywhere;
    }
    .log-error { color: #fca5a5; }
    .log-warning { color: #fde68a; }
    dialog {
      border: 1px solid #b91c1c;
      border-radius: 8px;
      background: #20252b;
      color: #f8fafc;
      min-width: 300px;
      padding: 22px;
    }
    dialog::backdrop { background: rgba(0, 0, 0, .68); }
    .fire-dialog-actions {
      display: flex;
      justify-content: flex-end;
      gap: 10px;
      margin-top: 20px;
    }
    img {
      max-width: 100%;
      max-height: calc(100vh - 92px);
      background: #050505;
      border: 1px solid #333;
      object-fit: contain;
    }
    aside {
      border: 1px solid #333;
      background: #181818;
      padding: 12px;
      border-radius: 6px;
    }
    aside.offset-panel {
      grid-column: 1;
      grid-row: 1;
    }
    aside.offset-panel details > summary {
      cursor: pointer;
      font-size: 15px;
      font-weight: 650;
      user-select: none;
    }
    aside.offset-panel details[open] > summary { margin-bottom: 12px; }
    .camera-panel {
      grid-column: 2;
      grid-row: 1;
    }
    aside h2 {
      margin: 0 0 10px;
      font-size: 15px;
      font-weight: 650;
      letter-spacing: 0;
    }
    .offset-row {
      display: grid;
      grid-template-columns: 1fr 86px;
      gap: 8px;
      align-items: center;
      margin: 6px 0;
      font-size: 13px;
    }
    .offset-row input {
      width: 100%;
      box-sizing: border-box;
      border: 1px solid #444;
      border-radius: 4px;
      background: #111;
      color: #eee;
      height: 30px;
      padding: 0 8px;
      font: inherit;
    }
    #offsetStatus {
      margin-top: 8px;
      min-height: 18px;
      color: #bbb;
      font-size: 12px;
    }
    .panel-spacer { height: 18px; }
    #tcpStatus {
      margin-top: 8px;
      min-height: 18px;
      color: #bbb;
      font-size: 12px;
    }
    @media (max-width: 900px) {
      header { grid-template-columns: 1fr; }
      .center-safety { justify-content: flex-start; }
      .right-controls { grid-template-columns: 1fr; }
      .utility-grid, .operation-grid { width: 100%; }
      main { grid-template-columns: 1fr; }
      aside.offset-panel, .camera-panel, aside.log-panel { grid-column: auto; grid-row: auto; }
      img { max-height: 60vh; }
      aside.log-panel { height: 180px; }
    }
  </style>
</head>
<body>
  <header>
    <div class="left-controls">
      <h1>WaSaB AdminGUI</h1>
      <div class="camera-controls tool-group" aria-label="Camera controls">
        <span class="group-title">Camera</span>
        <button id="liveButton" title="show live camera stream">Live</button>
        <button id="detectButton" title="run YOLO on latest frame and show scores">Detect</button>
        <button id="captureButton" title="save current camera capture on server">Capture</button>
      </div>
    </div>
    <div class="center-safety tool-group" aria-label="Emergency controls">
      <span class="group-title">Safety</span>
      <button id="dualStopButton" class="stop" title="stop both arms">Dual STOP</button>
      <button class="stop" data-command="stop" title="stop current motion immediately">STOP</button>
      <button class="danger" data-command="exit" title="stop and quit">Exit</button>
    </div>
    <div class="right-controls">
      <div class="utility-grid tool-group" aria-label="Arm and setup controls">
        <span class="group-title">Arm &amp; Setup</span>
        <button class="active" data-arm-mode="left">Left Arm</button>
        <button data-arm-mode="right">Right Arm</button>
        <button data-arm-mode="dual">Dual Arm</button>
        <button id="servoToggleButton" data-arm-modes="left right" title="toggle servo focus/release">Servo: FOCUSED</button>
        <button id="gestureToggleButton" data-arm-modes="right" class="gesture-on" title="toggle palm gesture recognition">Gesture: ON</button>
        <button class="danger" data-command="calibration" data-arm-modes="left right" title="stop the selected robot client and run auto_marker.py">Calibration</button>
      </div>
      <div class="operation-grid tool-group" aria-label="Robot operation controls">
        <span class="group-title">Operations</span>
        <button data-command="pick-place" data-arm-modes="left right" title="pick up a cola object, place it, then return home">Pick &amp; Place</button>
        <button data-command="restock" data-arm-modes="left" title="Left Arm: check one fixed view and replenish the saved pickup position">Restock</button>
        <button data-command="pick" data-arm-modes="left right" title="detect and pick up a cola object with YOLO">Pickup</button>
        <button data-command="place" data-arm-modes="left right" title="place the held object at the configured position">Place</button>
        <button data-command="home" data-arm-modes="left right dual" title="force home">Home</button>
        <button data-command="recycle" data-arm-modes="left" title="Left Arm: sort trash into the red bin and water into the blue bin">Recycle</button>
        <button data-command="help" data-arm-modes="left" title="Left Arm: detect and pick up AprilTag ID 0, then run Place">Help</button>
        <button id="fireDetectButton" data-arm-modes="left right" title="toggle fire detection for the selected arm">Fire Detect: OFF</button>
        <button id="faceRecognitionButton" data-arm-modes="left right" title="toggle face recognition for the selected arm">Face Recognition: OFF</button>
        <button id="trackingButton" data-arm-modes="left right" title="toggle original known-face tracking for the selected arm">Tracking: OFF</button>
        <button data-command="pose" data-arm-modes="left right" title="print current pose">Pose</button>
        <button data-command="gripper" data-arm-modes="left right" title="toggle gripper">Gripper</button>
        <button id="dualArmButton" data-arm-modes="dual" title="run Left preparation followed by Right palm handover" hidden>Gift Giving</button>
      </div>
      <div id="status">waiting for frame...</div>
    </div>
  </header>
  <main>
    <aside class="offset-panel">
      <details open>
        <summary>Offset settings</summary>
        <h2>Target Z offsets</h2>
        <div id="offsetRows"></div>
        <button id="saveOffsetsButton" title="apply target z offsets immediately">Save offsets</button>
        <div id="offsetStatus">loading offsets...</div>
        <div class="panel-spacer"></div>
        <h2>TCP offset</h2>
        <label class="offset-row"><span>X mm</span><input id="tcpOffsetX" type="number" step="1"></label>
        <label class="offset-row"><span>Y mm</span><input id="tcpOffsetY" type="number" step="1"></label>
        <label class="offset-row"><span>Z mm</span><input id="tcpOffsetZ" type="number" step="1"></label>
        <button id="saveTcpOffsetButton" title="apply TCP offset immediately">Save TCP</button>
        <div id="tcpStatus">loading TCP offset...</div>
      </details>
    </aside>
    <div class="camera-panel">
      <div id="leftCameraFeed" class="camera-feed"><span class="camera-label">LEFT</span><img id="frame" alt="latest left robot camera frame"></div>
      <div id="rightCameraFeed" class="camera-feed" hidden><span class="camera-label">RIGHT</span><img id="rightFrame" alt="latest right robot camera frame"></div>
    </div>
    <aside class="log-panel">
      <div class="log-heading"><h2>Operation logs</h2><button id="clearLogsButton">Clear</button></div>
      <div id="operationLogs" role="log" aria-live="polite">loading logs...</div>
    </aside>
  </main>
  <dialog id="firePromptDialog">
    <strong id="firePromptTitle">화재 감지</strong>
    <p id="firePromptMessage">진압을 시작할까요?</p>
    <div class="fire-dialog-actions">
      <button id="fireResponseNo">No</button>
      <button id="fireResponseYes" class="danger">Yes</button>
    </div>
  </dialog>
  <dialog id="facePromptDialog">
    <strong id="facePromptTitle">얼굴인식 결과</strong>
    <p id="facePromptMessage"></p>
    <div class="fire-dialog-actions">
      <button id="facePromptOk">확인</button>
    </div>
  </dialog>
  <script>
    const img = document.getElementById("frame");
    const rightImg = document.getElementById("rightFrame");
    const cameraPanel = document.querySelector(".camera-panel");
    const leftCameraFeed = document.getElementById("leftCameraFeed");
    const rightCameraFeed = document.getElementById("rightCameraFeed");
    const statusEl = document.getElementById("status");
    const offsetRowsEl = document.getElementById("offsetRows");
    const offsetStatusEl = document.getElementById("offsetStatus");
    const tcpStatusEl = document.getElementById("tcpStatus");
    const tcpOffsetInputs = [
      document.getElementById("tcpOffsetX"),
      document.getElementById("tcpOffsetY"),
      document.getElementById("tcpOffsetZ"),
    ];
    const gestureToggleButton = document.getElementById("gestureToggleButton");
    const fireDetectButton = document.getElementById("fireDetectButton");
    const faceRecognitionButton = document.getElementById("faceRecognitionButton");
    const trackingButton = document.getElementById("trackingButton");
    const operationLogsEl = document.getElementById("operationLogs");
    const logPanel = document.querySelector("aside.log-panel");
    const firePromptDialog = document.getElementById("firePromptDialog");
    const firePromptTitle = document.getElementById("firePromptTitle");
    const firePromptMessage = document.getElementById("firePromptMessage");
    const fireResponseYes = document.getElementById("fireResponseYes");
    const fireResponseNo = document.getElementById("fireResponseNo");
    const facePromptDialog = document.getElementById("facePromptDialog");
    const facePromptTitle = document.getElementById("facePromptTitle");
    const facePromptMessage = document.getElementById("facePromptMessage");
    const facePromptOk = document.getElementById("facePromptOk");
    const dualArmButton = document.getElementById("dualArmButton");
    const servoToggleButton = document.getElementById("servoToggleButton");
    const armModeButtons = [...document.querySelectorAll("button[data-arm-mode]")];
    const armScopedButtons = [...document.querySelectorAll("button[data-arm-modes]")];
    const singleArmCommandButtons = [...document.querySelectorAll("button[data-command]")]
      .filter((button) => !["stop", "exit"].includes(button.dataset.command));
    let armMode = "left";
    const servoFocused = { left: true, right: true };
    let lastLogId = 0;
    let gestureEnabled = false;
    let activeFirePrompt = null;
    let activeFacePrompt = null;
    function syncLogPanelHeight() {
      if (window.matchMedia("(max-width: 900px)").matches) {
        logPanel.style.height = "180px";
        return;
      }
      const visibleImages = [img, rightImg].filter(
        (item) => !item.closest(".camera-feed").hidden
      );
      const height = Math.max(
        ...visibleImages.map((item) => item.getBoundingClientRect().height),
        180
      );
      logPanel.style.height = `${Math.round(height)}px`;
    }
    new ResizeObserver(syncLogPanelHeight).observe(cameraPanel);
    img.addEventListener("load", syncLogPanelHeight);
    rightImg.addEventListener("load", syncLogPanelHeight);
    window.addEventListener("resize", syncLogPanelHeight);

    async function refreshFirePrompt() {
      if (firePromptDialog.open || armMode === "dual") return;
      try {
        const response = await fetch(
          `/arm-features/${armMode}/fire-prompt?t=${Date.now()}`,
          { cache: "no-store" }
        );
        if (!response.ok) return;
        const data = await response.json();
        if (!data.prompt || data.prompt.id === activeFirePrompt?.id) return;
        activeFirePrompt = data.prompt;
        firePromptTitle.textContent = `${data.prompt.arm_id.toUpperCase()} ARM · ${data.prompt.title || "화재 감지"}`;
        firePromptMessage.textContent = data.prompt.message;
        fireResponseYes.textContent = data.prompt.yes_label || "Yes";
        fireResponseNo.textContent = data.prompt.no_label || "No";
        fireResponseYes.classList.toggle("danger", data.prompt.kind !== "post-action");
        firePromptDialog.showModal();
      } catch { /* main status handles connectivity */ }
    }

    async function sendFireResponse(responseValue) {
      if (!activeFirePrompt) return;
      const prompt = activeFirePrompt;
      firePromptDialog.close();
      try {
        const response = await fetch(
          `/arm-features/${prompt.arm_id}/fire-response`,
          {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify({ response: responseValue }),
          }
        );
        const data = await response.json();
        statusEl.textContent = response.ok
          ? `fire response: ${responseValue.toUpperCase()}`
          : (data.detail || "fire response failed");
      } catch {
        statusEl.textContent = "fire response request failed";
      }
    }
    fireResponseYes.addEventListener(
      "click", () => sendFireResponse("yes")
    );
    fireResponseNo.addEventListener(
      "click", () => sendFireResponse("no")
    );

    async function refreshFacePrompt() {
      if (facePromptDialog.open || firePromptDialog.open || armMode === "dual") return;
      try {
        const response = await fetch(
          `/arm-features/${armMode}/face-prompt?t=${Date.now()}`,
          { cache: "no-store" }
        );
        if (!response.ok) return;
        const data = await response.json();
        if (!data.prompt || data.prompt.id === activeFacePrompt?.id) return;
        activeFacePrompt = data.prompt;
        facePromptTitle.textContent = `${data.prompt.arm_id.toUpperCase()} ARM · ${data.prompt.title}`;
        facePromptMessage.textContent = data.prompt.message;
        facePromptOk.classList.toggle("danger", !data.prompt.registered);
        facePromptDialog.showModal();
      } catch { /* main status handles connectivity */ }
    }

    facePromptOk.addEventListener("click", async () => {
      if (!activeFacePrompt) return;
      const prompt = activeFacePrompt;
      facePromptDialog.close();
      try {
        await fetch(`/arm-features/${prompt.arm_id}/face-prompt/ack`, {
          method: "POST",
        });
      } catch { /* the next poll retries while the prompt remains active */ }
    });
    const armFeatureButtons = {
      "fire-detect": fireDetectButton,
      "face-recognition": faceRecognitionButton,
      "tracking": trackingButton,
    };
    const armFeatureLabels = {
      "fire-detect": "Fire Detect",
      "face-recognition": "Face Recognition",
      "tracking": "Tracking",
    };
    function renderArmFeature(feature, running) {
      const button = armFeatureButtons[feature];
      button.textContent = `${armFeatureLabels[feature]}: ${running ? "ON" : "OFF"}`;
      button.classList.toggle("feature-on", running);
      button.setAttribute("aria-pressed", String(running));
    }
    async function refreshArmFeatures() {
      if (armMode === "dual") return;
      try {
        const requestedArm = armMode;
        const response = await fetch(`/arm-features/${requestedArm}/status?t=${Date.now()}`, { cache: "no-store" });
        if (!response.ok) return;
        const data = await response.json();
        if (armMode !== requestedArm) return;
        Object.keys(armFeatureButtons).forEach((feature) => {
          renderArmFeature(feature, Boolean(data[feature]?.running));
        });
      } catch { /* main status handles server connectivity */ }
    }
    async function toggleArmFeature(feature) {
      if (armMode === "dual") return;
      const requestedArm = armMode;
      const button = armFeatureButtons[feature];
      button.disabled = true;
      try {
        const response = await fetch(`/arm-features/${requestedArm}/${feature}/toggle`, {
          method: "POST",
          cache: "no-store",
        });
        const data = await response.json();
        if (!response.ok) {
          statusEl.textContent = data.detail || `${armFeatureLabels[feature]} toggle failed`;
          return;
        }
        if (armMode === requestedArm) renderArmFeature(feature, Boolean(data.running));
        statusEl.textContent = `${requestedArm} ${armFeatureLabels[feature]} ${data.running ? "started" : "stopped"}`;
      } catch {
        statusEl.textContent = `${armFeatureLabels[feature]} request failed`;
      } finally {
        button.disabled = false;
      }
    }
    Object.entries(armFeatureButtons).forEach(([feature, button]) => {
      button.addEventListener("click", () => toggleArmFeature(feature));
    });
    function renderGestureToggle() {
      gestureToggleButton.textContent = `Gesture: ${gestureEnabled ? "ON" : "OFF"}`;
      gestureToggleButton.classList.toggle("gesture-on", gestureEnabled);
      gestureToggleButton.classList.toggle("gesture-off", !gestureEnabled);
      gestureToggleButton.setAttribute("aria-pressed", String(gestureEnabled));
    }

    async function sendCommandToArm(command, armId) {
      statusEl.textContent = `sending ${command} to ${armId}...`;
      try {
        const response = await fetch(`/robot-command/${command}?arm_id=${armId}`, {
          method: "POST",
          cache: "no-store",
        });
        const data = await response.json();
        if (!response.ok) {
          statusEl.textContent = data.detail || `command ${command} failed`;
          return false;
        }
        statusEl.textContent = `queued ${data.command} #${data.id} → ${armId}`;
        return true;
      } catch {
        statusEl.textContent = "command send failed";
        return false;
      }
    }

    async function sendCommand(command) {
      if (armMode === "dual") {
        if (command === "stop") {
          await dualArmAction("stop");
          return true;
        }
        if (command === "home") {
          const results = await Promise.all([
            sendCommandToArm("home", "left"),
            sendCommandToArm("home", "right"),
          ]);
          statusEl.textContent = results.every(Boolean)
            ? "queued HOME → left + right"
            : "dual HOME command failed";
          return results.every(Boolean);
        }
        if (command !== "exit") {
          statusEl.textContent = "Dual mode: use Gift Giving, Home, or Dual STOP";
          return false;
        }
        const results = await Promise.all([
          sendCommandToArm(command, "left"),
          sendCommandToArm(command, "right"),
        ]);
        return results.every(Boolean);
      }
      return sendCommandToArm(command, armMode);
    }

    function renderArmMode() {
      armModeButtons.forEach((button) => {
        const active = button.dataset.armMode === armMode;
        button.classList.toggle("active", active);
        button.setAttribute("aria-pressed", String(active));
      });
      const dual = armMode === "dual";
      armScopedButtons.forEach((button) => {
        const modes = button.dataset.armModes.split(/\\s+/).filter(Boolean);
        button.hidden = !modes.includes(armMode);
      });
      cameraPanel.classList.toggle("dual", dual);
      leftCameraFeed.hidden = armMode === "right";
      rightCameraFeed.hidden = armMode === "left";
      singleArmCommandButtons.forEach((button) => {
        button.disabled = dual && button.dataset.command !== "home";
      });
      renderServoToggle();
      statusEl.textContent = `${armMode} control mode`;
      showLiveStream();
      refreshArmFeatures();
      requestAnimationFrame(syncLogPanelHeight);
    }

    armModeButtons.forEach((button) => {
      button.addEventListener("click", () => {
        armMode = button.dataset.armMode;
        renderArmMode();
      });
    });


    function renderServoToggle() {
      const focused = armMode === "dual"
        ? servoFocused.left && servoFocused.right
        : servoFocused[armMode];
      servoToggleButton.textContent = `Servo: ${focused ? "FOCUSED" : "RELEASED"}`;
      servoToggleButton.classList.toggle("servo-released", !focused);
      servoToggleButton.setAttribute("aria-pressed", String(focused));
    }

    servoToggleButton.addEventListener("click", async () => {
      const arms = armMode === "dual" ? ["left", "right"] : [armMode];
      const shouldFocus = !arms.every((armId) => servoFocused[armId]);
      const command = shouldFocus ? "servo-focus" : "servo-release";
      const results = await Promise.all(arms.map((armId) => sendCommandToArm(command, armId)));
      if (!results.every(Boolean)) return;
      arms.forEach((armId) => { servoFocused[armId] = shouldFocus; });
      renderServoToggle();
    });

    gestureToggleButton.addEventListener("click", async () => {
      const nextEnabled = !gestureEnabled;
      const accepted = await sendCommand(nextEnabled ? "gesture-on" : "gesture-off");
      if (!accepted) return;
      gestureEnabled = nextEnabled;
      renderGestureToggle();
    });

    async function loadGestureState() {
      try {
        const response = await fetch("/gesture-state", { cache: "no-store" });
        if (!response.ok) return;
        const data = await response.json();
        gestureEnabled = Boolean(data.enabled);
      } finally {
        renderGestureToggle();
      }
    }
    loadGestureState();

    document.querySelectorAll("button[data-command]").forEach((button) => {
      button.addEventListener("click", () => sendCommand(button.dataset.command));
    });

    const keyCommands = {
      g: "pick",
      p: "pose",
      q: "gripper",
      s: "servo-release",
      k: "servo-focus",
      f: "place",
      a: "recycle",
      t: "throw",
      w: "home",
      c: "calibration",
      " ": "stop",
      escape: "stop",
      x: "exit",
    };

    document.addEventListener("keydown", (event) => {
      if (event.repeat || event.altKey || event.ctrlKey || event.metaKey) return;
      const tagName = document.activeElement?.tagName?.toLowerCase();
      if (tagName === "input" || tagName === "textarea" || tagName === "select") return;
      const command = keyCommands[event.key.toLowerCase()];
      if (!command) return;
      event.preventDefault();
      sendCommand(command);
    });

    function renderOffsets(data) {
      const offsets = data.class_target_z_offsets_mm || {};
      const labels = Object.keys(offsets).filter(
        (label) => label.trim().toLowerCase() === "coca-cola"
      );
      offsetRowsEl.innerHTML = "";

      labels.forEach((label) => {
        const row = document.createElement("label");
        row.className = "offset-row";
        row.innerHTML = `<span></span><input data-offset-label="" type="number" step="1">`;
        row.querySelector("span").textContent = label;
        const input = row.querySelector("input");
        input.dataset.offsetLabel = label;
        input.value = offsets[label];
        offsetRowsEl.appendChild(row);
      });
    }

    async function loadOffsets() {
      try {
        const response = await fetch(`/settings/target-z-offsets?t=${Date.now()}`, { cache: "no-store" });
        const data = await response.json();
        if (!response.ok) {
          offsetStatusEl.textContent = data.detail || "offset load failed";
          return;
        }
        renderOffsets(data);
        offsetStatusEl.textContent = `plane Z ${data.object_plane_z_base_mm} mm`;
      } catch {
        offsetStatusEl.textContent = "offset server not reachable";
      }
    }

    async function saveOffsets() {
      const payload = {
        class_target_z_offsets_mm: {},
      };
      offsetRowsEl.querySelectorAll("input[data-offset-label]").forEach((input) => {
        payload.class_target_z_offsets_mm[input.dataset.offsetLabel] = Number(input.value);
      });

      offsetStatusEl.textContent = "saving offsets...";
      try {
        const response = await fetch("/settings/target-z-offsets", {
          method: "PATCH",
          headers: { "Content-Type": "application/json" },
          cache: "no-store",
          body: JSON.stringify(payload),
        });
        const data = await response.json();
        if (!response.ok) {
          offsetStatusEl.textContent = data.detail || "offset save failed";
          return;
        }
        renderOffsets(data);
        offsetStatusEl.textContent = "offsets applied";
      } catch {
        offsetStatusEl.textContent = "offset save failed";
      }
    }

    function renderTcpOffset(data) {
      const values = data.tcp_offset_flange_to_tcp_mm || [0, 0, 0];
      tcpOffsetInputs.forEach((input, index) => {
        input.value = values[index] ?? 0;
      });
    }

    async function loadTcpOffset() {
      try {
        const response = await fetch(`/settings/tcp-offset?t=${Date.now()}`, { cache: "no-store" });
        const data = await response.json();
        if (!response.ok) {
          tcpStatusEl.textContent = data.detail || "TCP offset load failed";
          return;
        }
        renderTcpOffset(data);
        tcpStatusEl.textContent = "TCP offset ready";
      } catch {
        tcpStatusEl.textContent = "TCP offset server not reachable";
      }
    }

    async function saveTcpOffset() {
      const payload = {
        tcp_offset_flange_to_tcp_mm: tcpOffsetInputs.map((input) => Number(input.value)),
      };

      tcpStatusEl.textContent = "saving TCP offset...";
      try {
        const response = await fetch("/settings/tcp-offset", {
          method: "PATCH",
          headers: { "Content-Type": "application/json" },
          cache: "no-store",
          body: JSON.stringify(payload),
        });
        const data = await response.json();
        if (!response.ok) {
          tcpStatusEl.textContent = data.detail || "TCP offset save failed";
          return;
        }
        renderTcpOffset(data);
        tcpStatusEl.textContent = "TCP offset applied";
      } catch {
        tcpStatusEl.textContent = "TCP offset save failed";
      }
    }

    function showLiveStream() {
      const cacheBust = Date.now();
      if (armMode === "left" || armMode === "dual") {
        img.src = `/camera-frame/stream.mjpg?arm_id=left&t=${cacheBust}`;
      }
      if (armMode === "right" || armMode === "dual") {
        rightImg.src = `/camera-frame/stream.mjpg?arm_id=right&t=${cacheBust}`;
      }
      statusEl.textContent = `${armMode} live stream`;
    }

    async function showWaSaBObjectDetectionPreview() {
      if (armMode === "dual") {
        statusEl.textContent = "Select Left or Right for detection";
        return;
      }
      statusEl.textContent = "running YOLO...";
      try {
        const response = await fetch(`/camera-frame/detect?arm_id=${armMode}&t=${Date.now()}`, {
          method: "POST",
          cache: "no-store",
        });
        const data = await response.json();
        if (!response.ok) {
          statusEl.textContent = data.detail || "detect failed";
          return;
        }
        const markerText = (data.markers || [])
          .map((marker) => `${marker.role}:${marker.id}`)
          .join(", ");
        statusEl.textContent = `YOLO ${data.detection_count} objects | tags ${markerText || "none"} | ${data.inference_ms.toFixed(1)}ms`;
      } catch {
        statusEl.textContent = "detect request failed";
      }
    }

    async function captureFrame() {
      if (armMode === "dual") {
        statusEl.textContent = "Select Left or Right for capture";
        return;
      }
      statusEl.textContent = "saving capture...";
      try {
        const response = await fetch(`/camera-frame/capture?arm_id=${armMode}&t=${Date.now()}`, {
          method: "POST",
          cache: "no-store",
        });
        const data = await response.json();
        if (!response.ok) {
          statusEl.textContent = data.detail || "capture failed";
          return;
        }
        statusEl.textContent = `capture saved: ${data.filename}`;
      } catch {
        statusEl.textContent = "capture request failed";
      }
    }

    document.getElementById("liveButton").addEventListener("click", showLiveStream);
    document.getElementById("detectButton").addEventListener("click", showWaSaBObjectDetectionPreview);
    document.getElementById("captureButton").addEventListener("click", captureFrame);
    document.getElementById("saveOffsetsButton").addEventListener("click", saveOffsets);
    document.getElementById("saveTcpOffsetButton").addEventListener("click", saveTcpOffset);
    document.getElementById("clearLogsButton").addEventListener("click", () => {
      operationLogsEl.replaceChildren();
    });

    async function dualArmAction(action) {
      statusEl.textContent = `dual-arm ${action}...`;
      try {
        const response = await fetch(`/dual-arm/${action}`, { method: "POST", cache: "no-store" });
        const data = await response.json();
        if (!response.ok) {
          statusEl.textContent = data.detail || `dual-arm ${action} failed`;
          return;
        }
        statusEl.textContent = `dual-arm: ${data.phase || data.status}`;
      } catch {
        statusEl.textContent = `dual-arm ${action} request failed`;
      }
    }

    async function refreshDualArmStatus() {
      try {
        const response = await fetch(`/dual-arm/status?t=${Date.now()}`, { cache: "no-store" });
        if (!response.ok) return;
        const data = await response.json();
        dualArmButton.disabled = !data.can_start;
        dualArmButton.title = data.can_start
          ? "start Gift Giving"
          : (data.running
            ? `Gift Giving in progress: ${data.phase}`
            : `Arm setup required: ${(data.right?.missing_fields || []).join(", ")}`);
        dualArmButton.textContent = data.running
          ? `Gift Giving: ${data.phase}`
          : "Gift Giving";
      } catch { /* status bar handles server connectivity */ }
    }

    dualArmButton.addEventListener("click", () => dualArmAction("gift-giving"));
    document.getElementById("dualStopButton").addEventListener("click", () => dualArmAction("stop"));

    function appendLogs(logs) {
      if (!logs.length) return;
      if (operationLogsEl.textContent === "loading logs...") operationLogsEl.replaceChildren();
      logs.forEach((entry) => {
        const line = document.createElement("div");
        line.className = `log-${entry.level || "info"}`;
        line.textContent = `[${entry.timestamp_iso}] [${entry.source}] ${entry.message}`;
        operationLogsEl.appendChild(line);
        lastLogId = Math.max(lastLogId, Number(entry.id) || 0);
      });
      while (operationLogsEl.children.length > 300) operationLogsEl.firstChild.remove();
      operationLogsEl.scrollTop = operationLogsEl.scrollHeight;
    }

    async function refreshOperationLogs() {
      try {
        const response = await fetch(`/robot-logs?after_id=${lastLogId}&t=${Date.now()}`, { cache: "no-store" });
        if (!response.ok) return;
        const data = await response.json();
        appendLogs(data.logs || []);
      } catch { /* camera status already reports connectivity */ }
    }

    showLiveStream();
    loadOffsets();
    loadTcpOffset();

    async function refreshStatus() {
      const cacheBust = Date.now();
      try {
        if (armMode === "dual") {
          const responses = await Promise.all(["left", "right"].map((armId) =>
            fetch(`/camera-frame/status?arm_id=${armId}&t=${cacheBust}`, { cache: "no-store" })
          ));
          const summaries = await Promise.all(responses.map(async (response, index) => {
            const armId = index === 0 ? "L" : "R";
            if (!response.ok) return `${armId}: waiting`;
            const data = await response.json();
            return `${armId}: ${data.width}x${data.height} ${data.age_sec.toFixed(1)}s`;
          }));
          statusEl.textContent = summaries.join(" | ");
          return;
        }
        const response = await fetch(`/camera-frame/status?arm_id=${armMode}&t=${cacheBust}`, { cache: "no-store" });
        if (!response.ok) {
          statusEl.textContent = "waiting for arm camera stream...";
          return;
        }
        const data = await response.json();
        statusEl.textContent = `${armMode} ${data.width}x${data.height} | ${data.source} | ${data.age_sec.toFixed(1)}s ago`;
      } catch {
        statusEl.textContent = "server not reachable";
      }
    }

    refreshStatus();
    setInterval(refreshStatus, 1000);
    refreshOperationLogs();
    setInterval(refreshOperationLogs, 700);
    refreshDualArmStatus();
    setInterval(refreshDualArmStatus, 2000);
    refreshArmFeatures();
    setInterval(refreshArmFeatures, 2000);
    refreshFirePrompt();
    setInterval(refreshFirePrompt, 500);
    refreshFacePrompt();
    setInterval(refreshFacePrompt, 500);
    renderArmMode();
  </script>
</body>
</html>"""


@app.post("/robot-command/{command}")
def enqueue_wasab_arm_command(
    command: str,
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> dict[str, Any]:
    """브라우저 버튼에서 Pi 클라이언트가 실행할 이름 기반 명령을 큐에 넣습니다."""
    requested = command.lower().strip()
    normalized = COMMAND_ALIASES.get(requested)
    if normalized is None:
        allowed = ", ".join(sorted(set(COMMAND_ALIASES.values())))
        raise HTTPException(status_code=400, detail=f"command must be one of: {allowed}")

    # STOP is a mode stop as well as a one-shot motor stop.  Leaving a vision
    # worker alive lets it enqueue new vision-track commands immediately after
    # the arm has stopped, which appears to the operator as a broken STOP.
    if normalized == "stop":
        with arm_feature_lock:
            for feature in ARM_FEATURE_NAMES:
                if _arm_feature_is_running(arm_id, feature):
                    _stop_arm_feature(arm_id, feature)
            arm_fire_prompts[arm_id] = None

    now = time.time()
    with wasab_service_state.command_lock:
        wasab_service_state.command_seq += 1
        item = {
            "id": wasab_service_state.command_seq,
            "command": normalized,
            "timestamp": now,
            "timestamp_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
            "source": "camera-view",
            "arm_id": arm_id,
        }
        if normalized == "stop":
            wasab_service_state.command_queue.clear()
        if normalized == "gesture-on":
            wasab_service_state.gesture_enabled = True
        elif normalized == "gesture-off":
            wasab_service_state.gesture_enabled = False
        wasab_service_state.command_queue.append(item)
        pending = len(wasab_service_state.command_queue)
        _append_operation_log_locked(
            "info", f"queued {normalized} #{item['id']} arm={arm_id} (pending={pending})", "camera-view"
        )
        wasab_service_state.command_condition.notify_all()
    print(
        "[ROBOT COMMAND] queued",
        normalized,
        f"id={item['id']}",
        f"pending={pending}",
        f"source={item['source']}",
    )
    return {"status": "queued", "pending": pending, **item}


@app.post("/palm-hitbox-target")
def enqueue_palm_hitbox_target(
    count: int = Query(ge=1, le=1000),
) -> dict[str, Any]:
    """Update the persistent Right Arm Palm Check sample target."""
    now = time.time()
    command = f"palm-hitbox-target:{count}"
    with wasab_service_state.command_lock:
        wasab_service_state.command_seq += 1
        item = {
            "id": wasab_service_state.command_seq,
            "command": command,
            "timestamp": now,
            "timestamp_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
            "source": "camera-view",
            "arm_id": "right",
        }
        wasab_service_state.command_queue.append(item)
        pending = len(wasab_service_state.command_queue)
        _append_operation_log_locked(
            "info",
            f"queued {command} #{item['id']} arm=right (pending={pending})",
            "camera-view",
        )
        wasab_service_state.command_condition.notify_all()
    return {"status": "queued", "pending": pending, "target_samples": count, **item}


def _append_operation_log_locked(level: str, message: str, source: str) -> dict[str, Any]:
    now = time.time()
    last_id = wasab_service_state.operation_logs[-1]["id"] if wasab_service_state.operation_logs else 0
    entry = {
        "id": last_id + 1,
        "level": level.lower().strip() or "info",
        "message": message.strip(),
        "source": source.strip() or "unknown",
        "timestamp": now,
        "timestamp_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
    }
    wasab_service_state.operation_logs.append(entry)
    del wasab_service_state.operation_logs[:-500]
    return entry


@app.get("/robot-logs")
def get_robot_logs(after_id: int = Query(default=0, ge=0)) -> dict[str, Any]:
    with wasab_service_state.command_lock:
        logs = [dict(item) for item in wasab_service_state.operation_logs if item["id"] > after_id]
    return {"status": "ok", "logs": logs}


@app.post("/robot-logs")
def append_robot_log(event: RobotLogEvent) -> dict[str, Any]:
    message = event.message.strip()
    if not message:
        raise HTTPException(status_code=400, detail="message cannot be empty")
    with wasab_service_state.command_lock:
        entry = _append_operation_log_locked(event.level, message[:2000], event.source)
    return {"status": "ok", "log": entry}


@app.get("/gesture-state")
def get_gesture_state() -> dict[str, Any]:
    with wasab_service_state.command_lock:
        enabled = wasab_service_state.gesture_enabled
    return {"status": "ok", "enabled": enabled}


@app.get("/dual-arm/status")
def get_dual_arm_status() -> dict[str, Any]:
    return {"status": "ok", **dual_arm_status()}


def _set_gift_giving_phase(phase: str) -> None:
    with dual_arm_runtime.lock:
        dual_arm_runtime.phase = phase
    with wasab_service_state.command_lock:
        _append_operation_log_locked("info", f"Gift Giving: {phase}", "dual-arm")


def _wait_for_arm_result(
    *,
    arm_id: str,
    success_message: str,
    after_log_id: int,
    timeout_sec: float,
) -> None:
    deadline = time.monotonic() + timeout_sec
    source = f"robot-client:{arm_id}"
    while time.monotonic() < deadline:
        with dual_arm_runtime.lock:
            if not dual_arm_runtime.running:
                raise RuntimeError("Gift Giving was stopped")
        with wasab_service_state.command_lock:
            logs = [
                dict(item)
                for item in wasab_service_state.operation_logs
                if item["id"] > after_log_id and item["source"] == source
            ]
        for item in logs:
            message = item["message"]
            if success_message in message:
                return
            normalized = message.lower()
            # Robot clients report an unsuccessful attempt before performing
            # their built-in recovery/retry.  This is progress, not the final
            # result of the arm command, so keep waiting for either the
            # completion message or a later terminal failure.
            retry_in_progress = (
                "retrying" in normalized
                or "retry in progress" in normalized
            )
            if retry_in_progress:
                continue
            reported_failure = any(
                marker in normalized
                for marker in (
                    "plan rejected",
                    "not detected",
                    "failed",
                    "timeout",
                    "aborted",
                )
            )
            if item["level"] == "error" or reported_failure:
                raise RuntimeError(f"{arm_id} arm failed: {message}")
        time.sleep(0.2)
    raise TimeoutError(
        f"{arm_id} arm did not report '{success_message}' within {timeout_sec:.0f}s"
    )


def _latest_operation_log_id() -> int:
    with wasab_service_state.command_lock:
        return (
            wasab_service_state.operation_logs[-1]["id"]
            if wasab_service_state.operation_logs
            else 0
        )


def _run_gift_giving_sequence() -> None:
    try:
        _set_gift_giving_phase("right_waiting_for_palm")
        palm_start_log_id = _latest_operation_log_id()
        # Gesture ON first moves Right Arm to its measured camera pose. Palm
        # Check immediately switches recognition to detection-only, so the
        # recognized hand cannot start Right pickup before Left prepares cola.
        enqueue_wasab_arm_command("gesture-on", arm_id="right")
        enqueue_wasab_arm_command("palm-check", arm_id="right")
        _wait_for_arm_result(
            arm_id="right",
            success_message="ONE PALM RECOGNIZED",
            after_log_id=palm_start_log_id,
            timeout_sec=180.0,
        )

        _set_gift_giving_phase("left_preparing_cola")
        left_start_log_id = _latest_operation_log_id()
        enqueue_wasab_arm_command("pick-place", arm_id="left")
        _wait_for_arm_result(
            arm_id="left",
            success_message="Fixed black-table place complete",
            after_log_id=left_start_log_id,
            timeout_sec=180.0,
        )

        _set_gift_giving_phase("right_delivering_cola")
        right_start_log_id = _latest_operation_log_id()
        enqueue_wasab_arm_command("pick-place", arm_id="right")
        _wait_for_arm_result(
            arm_id="right",
            success_message="Right pickup complete; Place starting",
            after_log_id=right_start_log_id,
            timeout_sec=180.0,
        )

        # Restock starts as soon as Right pickup succeeds and Right begins its
        # Place motion. Both arms then work concurrently.
        _set_gift_giving_phase("right_placing_and_left_restocking")
        restock_start_log_id = _latest_operation_log_id()
        enqueue_wasab_arm_command("restock", arm_id="left")
        _wait_for_arm_result(
            arm_id="right",
            success_message="Palm place complete; HOME ready",
            after_log_id=right_start_log_id,
            timeout_sec=300.0,
        )

        _wait_for_arm_result(
            arm_id="left",
            success_message="Restock complete",
            after_log_id=restock_start_log_id,
            timeout_sec=180.0,
        )

        with dual_arm_runtime.lock:
            dual_arm_runtime.running = False
            dual_arm_runtime.phase = "complete"
            dual_arm_runtime.last_error = None
        with wasab_service_state.command_lock:
            _append_operation_log_locked(
                "info",
                "Gift Giving complete: Coca-Cola delivered to the recognized palm",
                "dual-arm",
            )
    except Exception as exc:
        message = f"{type(exc).__name__}: {exc}"
        with dual_arm_runtime.lock:
            dual_arm_runtime.running = False
            dual_arm_runtime.phase = "failed"
            dual_arm_runtime.last_error = message
        with wasab_service_state.command_lock:
            _append_operation_log_locked("error", f"Gift Giving failed: {message}", "dual-arm")


def _start_gift_giving() -> dict[str, Any]:
    status = dual_arm_status()
    if not status["can_start"]:
        missing = status["right"].get("missing_fields", [])
        detail = "Gift Giving is not ready"
        if missing:
            detail += ": configure right_arm " + ", ".join(missing)
        raise HTTPException(status_code=409, detail=detail)
    with dual_arm_runtime.lock:
        dual_arm_runtime.running = True
        dual_arm_runtime.phase = "starting"
        dual_arm_runtime.last_error = None
        worker = threading.Thread(target=_run_gift_giving_sequence, daemon=True)
        dual_arm_runtime.worker = worker
        worker.start()
    return {"status": "started", **dual_arm_status()}


@app.post("/dual-arm/gift-giving")
def start_gift_giving() -> dict[str, Any]:
    return _start_gift_giving()


@app.post("/dual-arm/start")
def start_dual_arm_mode() -> dict[str, Any]:
    return _start_gift_giving()


@app.post("/dual-arm/stop")
def stop_dual_arm_mode() -> dict[str, Any]:
    with dual_arm_runtime.lock:
        dual_arm_runtime.running = False
        dual_arm_runtime.phase = "stopped"
    enqueue_wasab_arm_command("stop", arm_id="left")
    enqueue_wasab_arm_command("stop", arm_id="right")
    return {"status": "stopped", **dual_arm_status()}


@app.get("/robot-command/stream")
def stream_wasab_arm_commands(
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> StreamingResponse:
    """Stream queued browser commands to the Pi without polling."""
    def generate():
        while True:
            payload: dict[str, Any] | None = None
            with wasab_service_state.command_condition:
                matching_index = next(
                    (index for index, queued in enumerate(wasab_service_state.command_queue)
                     if queued.get("arm_id", "left") == arm_id),
                    None,
                )
                if matching_index is None:
                    wasab_service_state.command_condition.wait(timeout=15.0)
                    matching_index = next(
                        (index for index, queued in enumerate(wasab_service_state.command_queue)
                         if queued.get("arm_id", "left") == arm_id),
                        None,
                    )
                if matching_index is not None:
                    item = wasab_service_state.command_queue.pop(matching_index)
                    pending = len(wasab_service_state.command_queue)
                    _append_operation_log_locked(
                        "info", f"delivered {item.get('command')} #{item.get('id')} (pending={pending})", "server"
                    )
                    payload = {"status": "ok", "pending": pending, **item}
            # Never suspend the streaming generator while holding
            # command_condition. Robot clients post completion events through
            # /robot-logs using the same lock.
            if payload is None:
                yield json.dumps({"status": "heartbeat"}).encode() + b"\n"
                continue
            print(
                "[ROBOT COMMAND] delivered",
                payload.get("command"),
                f"id={payload.get('id')}",
                f"pending={pending}",
            )
            yield json.dumps(payload).encode() + b"\n"

    return StreamingResponse(
        generate(),
        media_type="application/x-ndjson",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


# ============================================================
# 4. 공통 유틸
# ============================================================

def _dump(model: BaseModel) -> dict[str, Any]:
    """Pydantic v1/v2 양쪽에서 동작하도록 직렬화."""
    if hasattr(model, "model_dump"):
        return model.model_dump()  # type: ignore[attr-defined]
    return model.dict()


UDP_STREAM_MAGIC = b"WASABU1"
UDP_STREAM_HEADER = struct.Struct("!7sIHHH")
ARM_ID_BY_STREAM_IP = {
    "192.168.2.10": "left",
    "192.168.2.12": "right",
}


def _normalize_arm_id(arm_id: str) -> str:
    normalized = arm_id.strip().lower()
    if normalized not in {"left", "right"}:
        raise HTTPException(status_code=400, detail="arm_id must be left or right")
    return normalized


def _arm_frame_snapshot(arm_id: str) -> tuple[bytes | None, dict[str, Any] | None]:
    arm_id = _normalize_arm_id(arm_id)
    with wasab_service_state.latest_frame_lock:
        entry = wasab_service_state.arm_frames[arm_id]
        jpeg = entry["jpeg"]
        meta = dict(entry["meta"]) if entry["meta"] else None
    return jpeg, meta


def run_udp_streamer_receiver(stop_event: threading.Event) -> None:
    """Receive WaSaBArm Streamer JPEG frames over UDP and publish them to AdminGUI."""
    max_payload = max(256, settings.udp_stream_max_datagram_bytes - UDP_STREAM_HEADER.size)
    fragments: dict[tuple[str, int], dict[str, Any]] = {}
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.2)
    try:
        sock.bind((settings.udp_stream_host, settings.udp_stream_port))
    except OSError as exc:
        print(f"[UDP STREAM] bind failed: {exc}")
        sock.close()
        return

    print(
        "[UDP STREAM] receiver ready:",
        f"{settings.udp_stream_host}:{settings.udp_stream_port}",
        f"payload<={max_payload}",
    )
    while not stop_event.is_set():
        try:
            packet, addr = sock.recvfrom(settings.udp_stream_max_datagram_bytes + 512)
        except socket.timeout:
            now = time.monotonic()
            stale = [
                key for key, entry in fragments.items()
                if now - float(entry["updated_at"]) > settings.udp_stream_frame_timeout_sec
            ]
            for key in stale:
                fragments.pop(key, None)
            continue
        except OSError as exc:
            if not stop_event.is_set():
                print(f"[UDP STREAM] receive error: {exc}")
            break

        if len(packet) < UDP_STREAM_HEADER.size:
            continue
        magic, frame_id, chunk_index, chunk_count, payload_len = UDP_STREAM_HEADER.unpack_from(packet)
        if magic != UDP_STREAM_MAGIC or chunk_count <= 0 or chunk_index >= chunk_count:
            continue
        payload = packet[UDP_STREAM_HEADER.size:]
        if len(payload) != payload_len:
            continue

        key = (addr[0], frame_id)
        entry = fragments.get(key)
        if entry is None or int(entry["chunk_count"]) != chunk_count:
            entry = {
                "chunk_count": chunk_count,
                "chunks": {},
                "updated_at": time.monotonic(),
                "addr": addr,
            }
            fragments[key] = entry
        entry["chunks"][chunk_index] = payload
        entry["updated_at"] = time.monotonic()

        if len(entry["chunks"]) != chunk_count:
            continue

        raw_jpeg = b"".join(entry["chunks"][index] for index in range(chunk_count))
        fragments.pop(key, None)
        try:
            frame = decode_image(raw_jpeg)
            _validate_frame_size(frame)
            arm_id = ARM_ID_BY_STREAM_IP.get(addr[0])
            if arm_id is None:
                print(f"[UDP STREAM] unknown sender {addr[0]}; defaulting to left")
                arm_id = "left"
            store_uploaded_streamer_jpeg(
                raw_jpeg, frame, f"udp-stream:{addr[0]}", arm_id=arm_id
            )
        except Exception as exc:
            print(f"[UDP STREAM] dropped frame {frame_id} from {addr[0]}: {exc}")

    sock.close()


def decode_image(file_bytes: bytes) -> np.ndarray:
    np_arr = np.frombuffer(file_bytes, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("이미지 디코딩 실패")
    return image


def _detect_apriltag_on_laptop(
    frame: np.ndarray,
    target_id: int,
) -> dict[str, Any] | None:
    if not hasattr(cv2, "aruco"):
        raise RuntimeError("Laptop OpenCV aruco module is unavailable")
    dictionary = cv2.aruco.getPredefinedDictionary(
        cv2.aruco.DICT_APRILTAG_36h11
    )
    parameters = (
        cv2.aruco.DetectorParameters()
        if hasattr(cv2.aruco, "DetectorParameters")
        else cv2.aruco.DetectorParameters_create()
    )
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray,
            dictionary,
            parameters=parameters,
        )
    if ids is None:
        return None
    ids_flat = ids.flatten().astype(int).tolist()
    for index, marker_id in enumerate(ids_flat):
        if marker_id != target_id:
            continue
        points = corners[index].reshape(-1, 2).astype(float)
        center = points.mean(axis=0)
        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        return {
            "id": marker_id,
            "role": "place",
            "ids": ids_flat,
            "center": [float(center[0]), float(center[1])],
            "corners": [
                [float(point[0]), float(point[1])]
                for point in points
            ],
            "bbox": [
                float(minimum[0]),
                float(minimum[1]),
                float(maximum[0]),
                float(maximum[1]),
            ],
            "corner_count": int(len(points)),
        }
    return None


def _latest_frame_meta(frame: np.ndarray, source: str, arm_id: str = "left") -> dict[str, Any]:
    h, w = frame.shape[:2]
    now = time.time()
    return {
        "timestamp": now,
        "timestamp_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
        "width": w,
        "height": h,
        "source": source,
        "arm_id": arm_id,
    }


def store_latest_stream_frame(frame: np.ndarray, source: str, arm_id: str = "left") -> None:
    arm_id = _normalize_arm_id(arm_id)
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    if not ok:
        raise ValueError("최신 카메라 프레임 JPEG 인코딩 실패")
    with wasab_service_state.latest_frame_lock:
        jpeg = encoded.tobytes()
        meta = _latest_frame_meta(frame, source, arm_id)
        wasab_service_state.arm_frames[arm_id] = {"jpeg": jpeg, "meta": meta}
        if arm_id == "left":
            wasab_service_state.latest_frame_jpeg = jpeg
            wasab_service_state.latest_frame_meta = meta


def _draw_detection_summary(frame: np.ndarray, summary: str | None) -> None:
    if not summary:
        return
    cv2.putText(
        frame,
        summary,
        (18, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.72,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )


def _draw_workspace_overlay(frame: np.ndarray, arm_id: str) -> np.ndarray:
    workspace = wasab_service_state.arm_workspace_overlays.get(arm_id)
    if not workspace or time.time() - float(workspace["timestamp"]) > 2.0:
        return frame

    flange_coords = workspace["flange_coords"]
    safe_x = workspace["safe_x_mm"]
    safe_y = workspace["safe_y_mm"]
    safe_z = workspace["safe_z_mm"]
    plane_z = float(workspace["object_plane_z_base_mm"])
    target_z = float(workspace["target_z_offset_mm"])
    target_offset = np.asarray(workspace["target_base_offset_mm"], dtype=np.float64)
    orientation = workspace["flange_orientation_deg"]

    R_base_flange = euler_to_rotation_matrix(*orientation, settings.euler_order)
    tcp_offset_base = (
        R_base_flange
        @ np.asarray(settings.tcp_offset_flange_to_tcp_mm, dtype=np.float64)
    )
    target_flange_z = (
        plane_z + target_z + float(target_offset[2]) - float(tcp_offset_base[2])
    )
    z_reachable = safe_z[0] <= target_flange_z <= safe_z[1]

    output = frame.copy()
    if z_reachable:
        base_points = np.asarray(
            [
                [
                    x + tcp_offset_base[0] - target_offset[0],
                    y + tcp_offset_base[1] - target_offset[1],
                    plane_z,
                ]
                for x, y in (
                    (safe_x[0], safe_y[0]),
                    (safe_x[1], safe_y[0]),
                    (safe_x[1], safe_y[1]),
                    (safe_x[0], safe_y[1]),
                )
            ],
            dtype=np.float64,
        )
        T_base_camera = (
            wasab_arm_pose_to_T_base_flange(flange_coords, settings.euler_order)
            @ wasab_service_state.calibration.T_flange_camera
        )
        T_camera_base = np.linalg.inv(T_base_camera)
        camera_points = (
            T_camera_base[:3, :3] @ base_points.T
            + T_camera_base[:3, 3:4]
        ).T
        if np.all(camera_points[:, 2] > 1e-6):
            rotation_vector, _ = cv2.Rodrigues(T_camera_base[:3, :3])
            pixels, _ = cv2.projectPoints(
                base_points,
                rotation_vector,
                T_camera_base[:3, 3],
                wasab_service_state.calibration.K,
                wasab_service_state.calibration.dist,
            )
            polygon = np.rint(pixels.reshape(-1, 2)).astype(np.int32)
            tint = output.copy()
            cv2.fillConvexPoly(tint, polygon, (0, 150, 0))
            output = cv2.addWeighted(tint, 0.22, output, 0.78, 0.0)
            cv2.polylines(output, [polygon], True, (0, 255, 0), 3, cv2.LINE_AA)
            cv2.putText(
                output,
                "SAFETY-ALLOWED AREA (IK NOT CHECKED)",
                tuple(polygon[0]),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
    else:
        tint = np.full_like(output, (0, 0, 180))
        output = cv2.addWeighted(tint, 0.16, output, 0.84, 0.0)
        delta = (
            safe_z[0] - target_flange_z
            if target_flange_z < safe_z[0]
            else target_flange_z - safe_z[1]
        )
        direction = "below min" if target_flange_z < safe_z[0] else "above max"
        cv2.putText(
            output,
            f"NO SAFETY-ALLOWED AREA: flange Z {target_flange_z:.1f} "
            f"({delta:.1f}mm {direction})",
            (16, 64),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 0, 255),
            2,
            cv2.LINE_AA,
        )

    cv2.putText(
        output,
        f"XYZ limits X {safe_x[0]:.1f}..{safe_x[1]:.1f}  "
        f"Y {safe_y[0]:.1f}..{safe_y[1]:.1f}  "
        f"Z {safe_z[0]:.1f}..{safe_z[1]:.1f}",
        (16, frame.shape[0] - 16),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return output


def store_uploaded_streamer_jpeg(
    raw_jpeg: bytes, frame: np.ndarray, source: str, arm_id: str = "left"
) -> None:
    arm_id = _normalize_arm_id(arm_id)
    # Preview frames are already JPEG-encoded on the Pi. Reusing those bytes avoids
    # a decode/re-encode cycle unless a short-lived detection overlay is active.
    with wasab_service_state.latest_frame_lock:
        frame = _draw_workspace_overlay(frame, arm_id)
        overlay = wasab_service_state.arm_detection_overlays[arm_id]
        workspace_active = wasab_service_state.arm_workspace_overlays.get(arm_id) is not None
        if time.time() < float(overlay["until"]):
            annotated = draw_wasab_detections(frame, overlay["detections"])
            _draw_detection_summary(annotated, overlay["summary"])
            ok, encoded = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ok:
                stored_jpeg = encoded.tobytes()
            else:
                stored_jpeg = raw_jpeg
        elif workspace_active:
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75]
            )
            stored_jpeg = encoded.tobytes() if ok else raw_jpeg
        else:
            overlay["detections"] = []
            overlay["summary"] = None
            stored_jpeg = raw_jpeg
        meta = _latest_frame_meta(frame, source, arm_id)
        wasab_service_state.arm_frames[arm_id] = {"jpeg": stored_jpeg, "meta": meta}
        if arm_id == "left":
            wasab_service_state.latest_frame_jpeg = stored_jpeg
            wasab_service_state.latest_frame_meta = meta


@app.post("/camera-frame")
async def receive_streamer_frame(
    image: UploadFile = File(...),
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> dict[str, Any]:
    """라즈베리파이/Jetcobot 클라이언트가 실시간 보기용 최신 프레임을 업로드합니다."""
    raw = await image.read()
    frame = _read_and_decode_upload(image, raw)
    store_uploaded_streamer_jpeg(raw, frame, "stream", arm_id=arm_id)
    _, meta = _arm_frame_snapshot(arm_id)
    return {"status": "ok", **meta}


@app.post("/palm-hitbox-capture")
async def receive_palm_hitbox_capture(
    image: UploadFile = File(...),
    arm_id: str = Query(default="right", pattern="^(left|right)$"),
) -> dict[str, Any]:
    """Save the separate colored hitbox capture produced after 3-second recognition."""
    raw = await image.read()
    frame = _read_and_decode_upload(image, raw)
    capture_dir = Path(__file__).resolve().parents[1] / "capture"
    capture_dir.mkdir(parents=True, exist_ok=True)
    path = capture_dir / f"palm_hitbox_capture.{arm_id}.png"
    if not cv2.imwrite(str(path), frame):
        raise HTTPException(status_code=500, detail="failed to save palm hitbox capture")
    return {"status": "ok", "arm_id": arm_id, "path": str(path)}


@app.post("/camera-frame/workspace")
def update_camera_workspace(
    request: WorkspaceOverlayUpdate,
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> dict[str, Any]:
    fields = {
        "flange_coords": (request.flange_coords, 6),
        "safe_x_mm": (request.safe_x_mm, 2),
        "safe_y_mm": (request.safe_y_mm, 2),
        "safe_z_mm": (request.safe_z_mm, 2),
        "target_base_offset_mm": (request.target_base_offset_mm, 3),
        "flange_orientation_deg": (request.flange_orientation_deg, 3),
    }
    parsed: dict[str, list[float]] = {}
    for name, (values, expected) in fields.items():
        if len(values) != expected:
            raise HTTPException(
                status_code=400,
                detail=f"{name} must contain {expected} values",
            )
        numbers = [float(value) for value in values]
        if not np.isfinite(np.asarray(numbers, dtype=np.float64)).all():
            raise HTTPException(status_code=400, detail=f"{name} must be finite")
        parsed[name] = numbers
    for name in ("safe_x_mm", "safe_y_mm", "safe_z_mm"):
        if parsed[name][0] >= parsed[name][1]:
            raise HTTPException(status_code=400, detail=f"{name} must be increasing")

    wasab_service_state.arm_workspace_overlays[arm_id] = {
        **parsed,
        "object_plane_z_base_mm": float(request.object_plane_z_base_mm),
        "target_z_offset_mm": float(request.target_z_offset_mm),
        "timestamp": time.time(),
    }
    return {"status": "ok", "arm_id": arm_id}


@app.post("/camera-frame/capture")
def capture_streamer_frame(
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> dict[str, Any]:
    """Save the latest camera-view JPEG on the laptop/server side."""
    jpeg, meta = _arm_frame_snapshot(arm_id)
    if jpeg is None or meta is None:
        raise HTTPException(status_code=404, detail="아직 업로드된 카메라 프레임이 없습니다.")

    capture_dir = Path(__file__).resolve().parents[1] / "capture"
    capture_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"jetcobot_{arm_id}_capture_{timestamp}.jpg"
    path = capture_dir / filename
    path.write_bytes(jpeg)
    return {
        "status": "ok",
        "filename": filename,
        "path": str(path),
        **meta,
    }


@app.get("/camera-frame/latest.jpg")
def latest_streamer_frame(
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> Response:
    jpeg, _ = _arm_frame_snapshot(arm_id)
    if jpeg is None:
        raise HTTPException(status_code=404, detail="아직 업로드된 카메라 프레임이 없습니다.")
    return Response(
        content=jpeg,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


def detect_apriltag_roles(frame: np.ndarray) -> list[dict[str, Any]]:
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    markers: list[dict[str, Any]] = []
    if ids is None:
        return markers
    for marker_corners, marker_id_raw in zip(corners, ids.flatten().astype(int).tolist()):
        if marker_id_raw in APRILTAG_PICKUP_IDS:
            role = "pickup"
        elif marker_id_raw in APRILTAG_PLACE_IDS:
            role = "place"
        else:
            continue
        points = marker_corners.reshape(-1, 2).astype(float)
        center = points.mean(axis=0)
        markers.append(
            {
                "id": int(marker_id_raw),
                "role": role,
                "center": [float(center[0]), float(center[1])],
                "corners": [[float(x), float(y)] for x, y in points],
            }
        )
    return markers


def draw_apriltag_roles(frame: np.ndarray, markers: list[dict[str, Any]]) -> None:
    for marker in markers:
        points = np.asarray(marker["corners"], dtype=np.int32).reshape(-1, 1, 2)
        color = (255, 170, 0) if marker["role"] == "pickup" else (255, 0, 255)
        cv2.polylines(frame, [points], True, color, 3, cv2.LINE_AA)
        center_x, center_y = (int(round(value)) for value in marker["center"])
        cv2.putText(
            frame,
            f"TAG {marker['role']} id={marker['id']}",
            (center_x - 70, max(24, center_y - 18)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.62,
            color,
            2,
            cv2.LINE_AA,
        )


@app.post("/camera-frame/detect")
def run_streamer_frame_detection(
    conf: float = Query(settings.default_conf, ge=0.0, le=1.0),
    imgsz: int = Query(settings.default_imgsz, ge=32),
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> dict[str, Any]:
    """Run YOLO on the latest uploaded frame and publish the annotated result to MJPEG."""
    jpeg, meta = _arm_frame_snapshot(arm_id)
    if jpeg is None or meta is None:
        raise HTTPException(status_code=404, detail="아직 업로드된 카메라 프레임이 없습니다.")

    try:
        frame = decode_image(jpeg)
    except ValueError as exc:
        raise HTTPException(status_code=500, detail=f"최신 카메라 프레임 디코딩 실패: {exc}") from exc

    try:
        detections, inference_ms = _run_inference(
            frame=frame,
            conf=conf,
            imgsz=imgsz,
            target_label=None,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"YOLO inference failed: {exc}") from exc

    markers = detect_apriltag_roles(frame)
    annotated = draw_wasab_detections(frame, detections)
    draw_apriltag_roles(annotated, markers)
    summary = (
        f"detections={len(detections)} tags={len(markers)} "
        f"inference_ms={inference_ms:.1f}"
    )
    _draw_detection_summary(annotated, summary)
    with wasab_service_state.latest_frame_lock:
        overlay = wasab_service_state.arm_detection_overlays[arm_id]
        overlay["until"] = time.time() + 3.0
        overlay["detections"] = detections
        overlay["summary"] = summary
    store_latest_stream_frame(annotated, "detect-result", arm_id=arm_id)
    return {
        "status": "ok",
        "detection_count": len(detections),
        "inference_ms": inference_ms,
        "detections": [_dump(det) for det in detections],
        "markers": markers,
    }


@app.get("/camera-frame/stream.mjpg")
def stream_camera_mjpeg(
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> StreamingResponse:
    def generate():
        last_timestamp = None
        while True:
            jpeg, meta = _arm_frame_snapshot(arm_id)
            if jpeg is None or meta is None:
                time.sleep(0.02)
                continue

            timestamp = meta.get("timestamp")
            if timestamp == last_timestamp:
                time.sleep(0.01)
                continue
            last_timestamp = timestamp

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                + f"Content-Length: {len(jpeg)}\r\n\r\n".encode()
                + jpeg
                + b"\r\n"
            )

    return StreamingResponse(
        generate(),
        media_type="multipart/x-mixed-replace; boundary=frame",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@app.get("/camera-frame/status")
def streamer_frame_status(
    arm_id: str = Query(default="left", pattern="^(left|right)$"),
) -> dict[str, Any]:
    _, meta = _arm_frame_snapshot(arm_id)
    if meta is None:
        raise HTTPException(status_code=404, detail="아직 업로드된 카메라 프레임이 없습니다.")
    meta["age_sec"] = max(0.0, time.time() - float(meta["timestamp"]))
    return {"status": "ok", **meta}


def draw_wasab_detections(frame: np.ndarray, detections: list[WaSaBObjectDetection]) -> np.ndarray:
    annotated = frame.copy()
    if settings.pick_roi_enabled:
        x_min, x_max = settings.pick_roi_x_px
        y_min, y_max = settings.pick_roi_y_px
        cv2.rectangle(annotated, (x_min, y_min), (x_max, y_max), (0, 200, 255), 2)
        cv2.putText(
            annotated,
            "PICK ROI",
            (x_min + 6, max(y_min - 8, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 200, 255),
            2,
            cv2.LINE_AA,
        )
    for det in detections:
        x1, y1, x2, y2 = [int(round(v)) for v in det.bbox]
        u, v = [int(round(value)) for value in det.center]
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            annotated,
            f"{det.label} {det.confidence:.2f}",
            (x1, max(y1 - 10, 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )
        cv2.circle(annotated, (u, v), 4, (0, 255, 0), -1)
    return annotated


def save_ai_detection_result(
    *,
    frame: np.ndarray,
    annotated: np.ndarray,
    detections: list[WaSaBObjectDetection],
    image_width: int,
    image_height: int,
    inference_ms: float,
    conf: float,
    imgsz: int,
    target_label: Optional[str],
    extra_result: dict[str, Any] | None = None,
) -> tuple[Optional[str], Optional[str], Optional[str], Optional[str]]:
    """기존 서버의 이미지/JSON 로그 기능을 유지합니다."""
    if not settings.save_results:
        return None, None, None, None

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    save_dir = settings.save_root_dir / timestamp
    save_dir.mkdir(parents=True, exist_ok=True)

    raw_path = save_dir / "raw.jpg"
    annotated_path = save_dir / "annotated.jpg"
    result_path = save_dir / "result.json"

    cv2.imwrite(str(raw_path), frame)
    cv2.imwrite(str(annotated_path), annotated)

    result_data: dict[str, Any] = {
        "timestamp": timestamp,
        "model_path": str(settings.model_path),
        "device": settings.device,
        "request": {"conf": conf, "imgsz": imgsz, "target_label": target_label},
        "image": {
            "width": image_width,
            "height": image_height,
            "raw_image_path": str(raw_path),
            "annotated_image_path": str(annotated_path),
        },
        "inference_ms": inference_ms,
        "detections": [_dump(det) for det in detections],
    }
    if extra_result is not None:
        result_data["grasp_plan_result"] = extra_result

    with result_path.open("w", encoding="utf-8") as f:
        json.dump(result_data, f, indent=2, ensure_ascii=False)

    return str(save_dir), str(raw_path), str(annotated_path), str(result_path)


def _validate_frame_size(frame: np.ndarray) -> None:
    h, w = frame.shape[:2]
    if settings.expected_image_width and w != settings.expected_image_width:
        raise HTTPException(
            status_code=400,
            detail=(
                f"Image width {w} differs from EXPECTED_IMAGE_WIDTH "
                f"{settings.expected_image_width}; camera calibration may be invalid"
            ),
        )
    if settings.expected_image_height and h != settings.expected_image_height:
        raise HTTPException(
            status_code=400,
            detail=(
                f"Image height {h} differs from EXPECTED_IMAGE_HEIGHT "
                f"{settings.expected_image_height}; camera calibration may be invalid"
            ),
        )


def _bbox_iou(first: list[float], second: list[float]) -> float:
    x1 = max(first[0], second[0])
    y1 = max(first[1], second[1])
    x2 = min(first[2], second[2])
    y2 = min(first[3], second[3])

    intersection_width = max(0.0, x2 - x1)
    intersection_height = max(0.0, y2 - y1)
    intersection_area = intersection_width * intersection_height
    if intersection_area <= 0.0:
        return 0.0

    first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
    second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
    union_area = first_area + second_area - intersection_area
    return intersection_area / union_area if union_area > 0.0 else 0.0


def _suppress_overlapping_detections(
    detections: list[WaSaBObjectDetection],
    iou_threshold: float = OVERLAP_SUPPRESSION_IOU_THRESHOLD,
) -> list[WaSaBObjectDetection]:
    kept: list[WaSaBObjectDetection] = []
    for detection in detections:
        if any(_bbox_iou(detection.bbox, existing.bbox) >= iou_threshold for existing in kept):
            continue
        kept.append(detection)
    return kept


def _normalize_axis_angle_deg(angle: float) -> float:
    return float(angle) % 180.0


def _estimate_grip_axis_from_crop(
    frame: np.ndarray,
    bbox: list[float],
) -> tuple[float | None, str | None, list[float] | None, str | None, list[list[float]] | None, str | None]:
    """Estimate the object's long axis in image space.

    The detector returns an axis-aligned box, so use image content inside the
    crop when possible and fall back to the bbox major axis if segmentation is
    not reliable enough.
    """
    h, w = frame.shape[:2]
    x1, y1, x2, y2 = bbox
    left = max(0, int(np.floor(x1)))
    top = max(0, int(np.floor(y1)))
    right = min(w, int(np.ceil(x2)))
    bottom = min(h, int(np.ceil(y2)))
    if right - left < 8 or bottom - top < 8:
        return None, None, None, None, None, None

    crop = frame[top:bottom, left:right]
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    contour_candidates: list[np.ndarray] = []
    for mask in (binary, cv2.bitwise_not(binary)):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contour_candidates.extend(contours)

    crop_area = float(crop.shape[0] * crop.shape[1])
    usable_contours = [
        contour
        for contour in contour_candidates
        if 0.04 * crop_area <= cv2.contourArea(contour) <= 0.92 * crop_area
    ]
    if usable_contours:
        contour = max(usable_contours, key=cv2.contourArea)
        moments = cv2.moments(contour)
        grip_center: list[float] | None = None
        center_source: str | None = None
        if abs(moments["m00"]) > 1e-9:
            grip_center = [
                float(left + moments["m10"] / moments["m00"]),
                float(top + moments["m01"] / moments["m00"]),
            ]
            center_source = "contour_moments"
        points = contour.reshape(-1, 2).astype(np.float64)
        if len(points) >= 5:
            mean = points.mean(axis=0)
            centered = points - mean
            covariance = centered.T @ centered / max(1, len(points) - 1)
            eigenvalues, eigenvectors = np.linalg.eigh(covariance)
            major = eigenvectors[:, int(np.argmax(eigenvalues))]
            angle = np.degrees(np.arctan2(float(major[1]), float(major[0])))
            projections = centered @ major
            projection_min = float(projections.min())
            projection_max = float(projections.max())
            projection_span = max(1.0, projection_max - projection_min)
            end_band = max(3.0, projection_span * 0.08)

            min_end_points = points[projections <= projection_min + end_band]
            max_end_points = points[projections >= projection_max - end_band]
            endpoint_min = (
                min_end_points.mean(axis=0)
                if len(min_end_points) >= 2
                else mean + major * projection_min
            )
            endpoint_max = (
                max_end_points.mean(axis=0)
                if len(max_end_points) >= 2
                else mean + major * projection_max
            )
            endpoints = [
                [float(left + endpoint_min[0]), float(top + endpoint_min[1])],
                [float(left + endpoint_max[0]), float(top + endpoint_max[1])],
            ]
            return (
                _normalize_axis_angle_deg(angle),
                "contour_pca",
                grip_center,
                center_source,
                endpoints,
                "contour_axis_projection",
            )
        if grip_center is not None:
            return None, None, grip_center, center_source, None, None

    width = max(0.0, x2 - x1)
    height = max(0.0, y2 - y1)
    if width <= 0.0 or height <= 0.0:
        return None, None, None, None, None, None
    if width >= height:
        endpoints = [[float(x1), float((y1 + y2) / 2.0)], [float(x2), float((y1 + y2) / 2.0)]]
        angle = 0.0
    else:
        endpoints = [[float((x1 + x2) / 2.0), float(y1)], [float((x1 + x2) / 2.0), float(y2)]]
        angle = 90.0
    return angle, "bbox_major_axis", None, None, endpoints, "bbox_major_axis"


def _axis_reachability(name: str, value: float, limits: tuple[float, float]) -> dict[str, Any] | None:
    if limits[0] <= value <= limits[1]:
        return None
    return {
        "axis": name,
        "value": round(float(value), 3),
        "min": round(float(limits[0]), 3),
        "max": round(float(limits[1]), 3),
    }


def evaluate_plan_reachability(flange_command: list[float]) -> dict[str, Any]:
    violations: list[dict[str, Any]] = []
    violations.extend(
        item
        for item in [
            _axis_reachability("x", flange_command[0], settings.plan_safe_x_mm),
            _axis_reachability("y", flange_command[1], settings.plan_safe_y_mm),
            _axis_reachability("z", flange_command[2], settings.plan_safe_z_mm),
        ]
        if item is not None
    )

    for index, angle in enumerate(flange_command[3:], start=3):
        if abs(angle) > settings.plan_safe_euler_abs_deg:
            violations.append(
                {
                    "axis": ("rx", "ry", "rz")[index - 3],
                    "value": round(float(angle), 3),
                    "min": round(-float(settings.plan_safe_euler_abs_deg), 3),
                    "max": round(float(settings.plan_safe_euler_abs_deg), 3),
                }
            )

    return {
        "reachable": not violations,
        "violations": violations,
        "limits": {
            "x_mm": list(settings.plan_safe_x_mm),
            "y_mm": list(settings.plan_safe_y_mm),
            "z_mm": list(settings.plan_safe_z_mm),
            "euler_abs_deg": settings.plan_safe_euler_abs_deg,
        },
    }


def _pick_bbox_edge_reject_reason(
    detection: WaSaBObjectDetection,
    image_width: int,
    image_height: int,
    margin_px: float = PICK_BBOX_EDGE_MARGIN_PX,
) -> str | None:
    x1, y1, x2, y2 = detection.bbox
    touched_edges: list[str] = []
    if x1 <= margin_px:
        touched_edges.append("left")
    if y1 <= margin_px:
        touched_edges.append("top")
    if x2 >= image_width - margin_px:
        touched_edges.append("right")
    if y2 >= image_height - margin_px:
        touched_edges.append("bottom")
    if not touched_edges:
        return None
    edges = ",".join(touched_edges)
    return f"partial_object_near_image_edge:{edges}; margin_px={margin_px:.0f}"


def _pick_roi_reject_reason(detection: WaSaBObjectDetection) -> str | None:
    if not settings.pick_roi_enabled:
        return None
    x_min, x_max = settings.pick_roi_x_px
    y_min, y_max = settings.pick_roi_y_px
    x1, y1, x2, y2 = detection.bbox
    center_x = (x1 + x2) / 2.0
    center_y = (y1 + y2) / 2.0
    if x_min <= center_x <= x_max and y_min <= center_y <= y_max:
        return None
    return (
        f"outside_pick_roi:center=({center_x:.1f},{center_y:.1f}); "
        f"roi=({x_min},{y_min})-({x_max},{y_max})"
    )


def _filter_pickable_detections(
    detections: list[WaSaBObjectDetection],
    image_width: int,
    image_height: int,
) -> tuple[list[WaSaBObjectDetection], list[dict[str, Any]]]:
    pickable: list[WaSaBObjectDetection] = []
    rejected: list[dict[str, Any]] = []
    for detection in detections:
        reason = _pick_roi_reject_reason(detection)
        if reason is None:
            reason = _pick_bbox_edge_reject_reason(detection, image_width, image_height)
        if reason is None:
            pickable.append(detection)
            continue
        payload = _dump(detection)
        payload["reject_reason"] = reason
        rejected.append(payload)
    return pickable, rejected


def _bbox_is_near_square(width: float, height: float) -> bool:
    if width <= 1e-9 or height <= 1e-9:
        return True
    ratio = max(width / height, height / width)
    return ratio < settings.gripper_auto_rotate_aspect_ratio


def _run_inference(
    *,
    frame: np.ndarray,
    conf: float,
    imgsz: int,
    target_label: str | None,
) -> tuple[list[WaSaBObjectDetection], float]:
    start = time.perf_counter()
    # 단일 GPU 모델은 요청 간 동시 접근을 막아 메모리/결과 충돌을 피합니다.
    with wasab_service_state.inference_lock:
        results = wasab_service_state.model.predict(
            source=frame,
            conf=conf,
            imgsz=imgsz,
            device=settings.device,
            agnostic_nms=True,
            verbose=False,
        )
    inference_ms = (time.perf_counter() - start) * 1000.0

    detections: list[WaSaBObjectDetection] = []
    if not results or results[0].boxes is None:
        return detections, inference_ms

    result = results[0]
    names = result.names
    for box in result.boxes:
        x1, y1, x2, y2 = box.xyxy[0].detach().cpu().numpy().astype(float)
        score = float(box.conf[0].detach().cpu().item())
        class_id = int(box.cls[0].detach().cpu().item())
        label = str(names.get(class_id, class_id))
        if target_label is not None and label != target_label:
            continue

        width = x2 - x1
        height = y2 - y1
        bbox_center = [(x1 + x2) / 2.0, (y1 + y2) / 2.0]
        (
            grip_axis_image_deg,
            grip_axis_source,
            grip_center,
            grip_center_source,
            grip_axis_endpoints,
            grip_axis_endpoints_source,
        ) = _estimate_grip_axis_from_crop(
            frame,
            [x1, y1, x2, y2],
        )
        if _bbox_is_near_square(width, height):
            center = bbox_center
            grip_axis_image_deg = None
            grip_axis_source = None
            grip_center_source = "bbox_center_near_square"
            grip_axis_endpoints = None
            grip_axis_endpoints_source = None
        else:
            center = grip_center if grip_center is not None else bbox_center
        detections.append(
            WaSaBObjectDetection(
                label=label,
                class_id=class_id,
                confidence=score,
                bbox=[x1, y1, x2, y2],
                center=center,
                width=width,
                height=height,
                grip_axis_image_deg=grip_axis_image_deg,
                grip_axis_source=grip_axis_source,
                grip_center_source=grip_center_source or "bbox_center",
                grip_axis_endpoints=grip_axis_endpoints,
                grip_axis_endpoints_source=grip_axis_endpoints_source,
            )
        )

    detections.sort(key=lambda item: item.confidence, reverse=True)
    detections = _suppress_overlapping_detections(detections)
    return detections, inference_ms


def _read_and_decode_upload(image: UploadFile, raw: bytes) -> np.ndarray:
    if not raw:
        raise HTTPException(status_code=400, detail="빈 이미지 업로드입니다.")
    if len(raw) > settings.max_upload_bytes:
        raise HTTPException(status_code=413, detail="업로드 이미지가 MAX_UPLOAD_BYTES를 초과했습니다.")
    try:
        frame = decode_image(raw)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    _validate_frame_size(frame)
    return frame


def _parse_robot_state(raw: str) -> tuple[list[float], str | None, str]:
    """Pi가 보내는 `robot_state` form field를 검증합니다.

    형식: {"request_id":"...", "flange_coords":[x,y,z,rx,ry,rz]}
    """
    try:
        robot_state = json.loads(raw)
    except json.JSONDecodeError as exc:
        raise HTTPException(status_code=400, detail="robot_state는 JSON 문자열이어야 합니다.") from exc

    coords = robot_state.get("flange_coords") if isinstance(robot_state, dict) else None
    if not isinstance(coords, list) or len(coords) != 6:
        raise HTTPException(status_code=400, detail="robot_state.flange_coords는 6개 숫자여야 합니다.")
    try:
        parsed = [float(value) for value in coords]
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail="flange_coords에 숫자가 아닌 값이 있습니다.") from exc
    if not np.isfinite(np.asarray(parsed, dtype=np.float64)).all():
        raise HTTPException(status_code=400, detail="flange_coords에 유한하지 않은 값이 있습니다.")

    request_id = robot_state.get("request_id")
    arm_id = str(robot_state.get("arm_id", "left")).strip().lower()
    if arm_id not in {"left", "right"}:
        raise HTTPException(status_code=400, detail="robot_state.arm_id must be left or right")
    return parsed, str(request_id) if request_id is not None else None, arm_id


# ============================================================
# 5. 기존 YOLO detect API (호환 유지)
# ============================================================

@app.post("/v1/apriltag-detect")
async def apriltag_detect_v1(
    image: UploadFile = File(...),
    target_id: int = Form(...),
    arm_id: str = Form("left"),
) -> dict[str, Any]:
    raw = await image.read()
    frame = _read_and_decode_upload(image, raw)
    normalized_arm_id = arm_id.strip().lower()
    if normalized_arm_id not in {"left", "right"}:
        raise HTTPException(
            status_code=400,
            detail="arm_id must be left or right",
        )
    try:
        detection = _detect_apriltag_on_laptop(frame, int(target_id))
    except Exception as exc:
        raise HTTPException(
            status_code=500,
            detail=f"AprilTag detection failed: {exc}",
        ) from exc
    store_latest_stream_frame(
        frame,
        "apriltag-detect",
        arm_id=normalized_arm_id,
    )
    return {
        "status": "ok" if detection is not None else "not_found",
        "target_id": int(target_id),
        "arm_id": normalized_arm_id,
        "image_width": int(frame.shape[1]),
        "image_height": int(frame.shape[0]),
        "detection": detection,
    }


@app.post("/detect", response_model=WaSaBInferResponse)
async def detect(
    image: UploadFile = File(...),
    conf: float = Query(settings.default_conf, ge=0.0, le=1.0),
    imgsz: int = Query(settings.default_imgsz, ge=32),
    target_label: Optional[str] = Query(None),
) -> WaSaBInferResponse:
    raw = await image.read()
    frame = _read_and_decode_upload(image, raw)
    store_latest_stream_frame(frame, "detect")
    h, w = frame.shape[:2]
    try:
        detections, inference_ms = _run_inference(
            frame=frame,
            conf=conf,
            imgsz=imgsz,
            target_label=target_label,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"YOLO inference failed: {exc}") from exc

    annotated = draw_wasab_detections(frame, detections)
    save_dir, raw_path, annotated_path, json_path = save_ai_detection_result(
        frame=frame,
        annotated=annotated,
        detections=detections,
        image_width=w,
        image_height=h,
        inference_ms=inference_ms,
        conf=conf,
        imgsz=imgsz,
        target_label=target_label,
    )
    print("[DETECT] raw:", raw_path)
    print("[DETECT] annotated:", annotated_path)
    print("[DETECT] json:", json_path)

    return WaSaBInferResponse(
        status="ok",
        image_width=w,
        image_height=h,
        inference_ms=inference_ms,
        detections=detections,
        saved_dir=save_dir,
        raw_image_path=raw_path,
        annotated_image_path=annotated_path,
        result_json_path=json_path,
    )


# ============================================================
# 6. 서버측 YOLO + 3D 파지계획 API
# ============================================================

def _create_grasp_plan_from_frame(
    *,
    frame: np.ndarray,
    current_flange_coords: list[float],
    request_id: str | None,
    frame_source: str,
    target_label: str | None = None,
    arm_id: str = "left",
) -> dict[str, Any]:
    h, w = frame.shape[:2]

    try:
        raw_detections, inference_ms = _run_inference(
            frame=frame,
            conf=settings.default_conf,
            imgsz=settings.default_imgsz,
            target_label=target_label,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"YOLO inference failed: {exc}") from exc

    detections, rejected_detections = _filter_pickable_detections(raw_detections, w, h)

    annotated = draw_wasab_detections(frame, detections)
    store_latest_stream_frame(annotated, "grasp-plan-result", arm_id=arm_id)
    extra_result: dict[str, Any] = {
        "request_id": request_id,
        "current_flange_coords": current_flange_coords,
        "rejected_detections": rejected_detections,
        "pick_edge_margin_px": PICK_BBOX_EDGE_MARGIN_PX,
        "pick_roi": {
            "enabled": settings.pick_roi_enabled,
            "x_px": list(settings.pick_roi_x_px),
            "y_px": list(settings.pick_roi_y_px),
        },
    }

    if not detections:
        message = "Target object was not detected."
        if rejected_detections:
            reject_reasons = [str(item.get("reject_reason", "")) for item in rejected_detections]
            if any(reason.startswith("outside_pick_roi:") for reason in reject_reasons):
                # Keep this distinct from the no-detection/edge messages. The Pi
                # client must not move its search pose to bring an out-of-ROI
                # object back into view.
                message = "Target detections were outside the pick ROI; pick was skipped."
            else:
                message = "Only partial target detections near the image edge were found; pick was skipped."
        extra_result.update({"status": "not_found", "message": message})
        save_dir, raw_path, annotated_path, json_path = save_ai_detection_result(
            frame=frame,
            annotated=annotated,
            detections=detections,
            image_width=w,
            image_height=h,
            inference_ms=inference_ms,
            conf=settings.default_conf,
            imgsz=settings.default_imgsz,
            target_label=target_label,
            extra_result=extra_result,
        )
        return {
            "status": "not_found",
            "request_id": request_id,
            "message": message,
            "frame_source": frame_source,
            "image_width": w,
            "image_height": h,
            "inference_ms": inference_ms,
            "detections": [_dump(det) for det in detections],
            "rejected_detections": rejected_detections,
            "pick_edge_margin_px": PICK_BBOX_EDGE_MARGIN_PX,
            "saved_dir": save_dir,
            "raw_image_path": raw_path,
            "annotated_image_path": annotated_path,
            "result_json_path": json_path,
        }

    # detections는 confidence 내림차순. 가장 높은 대상 하나를 서버가 선택합니다.
    selected = detections[0]
    try:
        with wasab_service_state.target_z_offsets_lock:
            selected_target_z_offset_mm = target_z_offset_for_label(selected.label, settings)
            selected_target_base_offset_mm = pick_target_base_offset_for_label(selected.label, settings)
            class_target_z_offsets_mm = dict(settings.target_z_offsets_mm)
            class_target_base_offsets_mm = {
                label: list(offset)
                for label, offset in settings.pick_target_base_offsets_mm.items()
            }
            selected_detection = _dump(selected)
            selected_detection["target_base_offset_mm"] = list(selected_target_base_offset_mm)
            result = compute_wasab_operation_plan(
                detection=selected_detection,
                current_flange_coords=current_flange_coords,
                calibration=wasab_service_state.arm_calibrations.get(
                    arm_id,
                    wasab_service_state.calibration,
                ),
                settings=settings,
            )
    except WaSaBOperationPlanError as exc:
        raise HTTPException(status_code=422, detail=f"Grasp-plan geometry error: {exc}") from exc

    plan_reachability = evaluate_plan_reachability(result["plan"]["flange_command"])
    extra_result.update({"status": "ok", **result, "plan_reachability": plan_reachability})
    save_dir, raw_path, annotated_path, json_path = save_ai_detection_result(
        frame=frame,
        annotated=annotated,
        detections=detections,
        image_width=w,
        image_height=h,
        inference_ms=inference_ms,
        conf=settings.default_conf,
        imgsz=settings.default_imgsz,
        target_label=target_label,
        extra_result=extra_result,
    )

    return {
        "status": "ok",
        "request_id": request_id,
        "frame_source": frame_source,
        "image_width": w,
        "image_height": h,
        "inference_ms": inference_ms,
        "detections": [_dump(det) for det in detections],
        "rejected_detections": rejected_detections,
        "pick_edge_margin_px": PICK_BBOX_EDGE_MARGIN_PX,
        "server_policy": {
            "target_label": target_label,
            "confidence_threshold": settings.default_conf,
            "object_plane_z_base_mm": settings.object_plane_z_base_mm,
            "default_target_z_offset_mm": settings.default_target_z_offset_mm,
            "selected_target_z_offset_mm": selected_target_z_offset_mm,
            "selected_target_base_offset_mm": list(selected_target_base_offset_mm),
            "pick_flange_orientation_deg": (
                list(settings.pick_flange_orientation_deg)
                if settings.pick_flange_orientation_deg is not None
                else None
            ),
            "gripper_auto_rotate_long_bbox_enabled": settings.gripper_auto_rotate_long_bbox_enabled,
            "gripper_orientation_mode": "image_axis_dynamic",
            "gripper_auto_rotate_aspect_ratio": settings.gripper_auto_rotate_aspect_ratio,
            "gripper_auto_rotate_rz_offset_deg": settings.gripper_auto_rotate_rz_offset_deg,
            "class_target_z_offsets_mm": class_target_z_offsets_mm,
            "class_target_base_offsets_mm": class_target_base_offsets_mm,
        },
        **result,
        "plan_reachability": plan_reachability,
        "saved_dir": save_dir,
        "raw_image_path": raw_path,
        "annotated_image_path": annotated_path,
        "result_json_path": json_path,
    }


async def _create_grasp_plan(
    *,
    image: UploadFile,
    robot_state: str,
    target_label: str | None = None,
) -> dict[str, Any]:
    """노트북의 고정 정책으로 파지 계획을 생성합니다.

    라즈베리파이는 frame과 촬영 시점의 현재 Flange pose만 보내며,
    YOLO 및 모든 좌표변환은 이 함수 내부에서 완료합니다.
    """
    current_flange_coords, request_id, arm_id = _parse_robot_state(robot_state)
    raw = await image.read()
    frame = _read_and_decode_upload(image, raw)
    return _create_grasp_plan_from_frame(
        frame=frame,
        current_flange_coords=current_flange_coords,
        request_id=request_id,
        frame_source="uploaded_grasp_plan_frame",
        target_label=target_label,
        arm_id=arm_id,
    )


@app.post("/grasp-plan")
async def grasp_plan(
    image: UploadFile = File(...),
    robot_state: str = Form(...),
    target_label: str | None = Form(None),
) -> dict[str, Any]:
    return await _create_grasp_plan(
        image=image,
        robot_state=robot_state,
        target_label=target_label,
    )


def _parse_latest_frame_grasp_plan_request(request: LatestFrameGraspPlanRequest) -> tuple[list[float], str | None]:
    coords = request.flange_coords
    if not isinstance(coords, list) or len(coords) != 6:
        raise HTTPException(status_code=400, detail="flange_coords must contain six values")
    try:
        parsed = [float(value) for value in coords]
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail="flange_coords must be numeric") from exc
    if not np.isfinite(np.asarray(parsed, dtype=np.float64)).all():
        raise HTTPException(status_code=400, detail="flange_coords contains non-finite values")
    return parsed, request.request_id


@app.post("/v1/latest-frame-grasp-plan")
def latest_frame_grasp_plan_v1(request: LatestFrameGraspPlanRequest) -> dict[str, Any]:
    current_flange_coords, request_id = _parse_latest_frame_grasp_plan_request(request)
    with wasab_service_state.latest_frame_lock:
        jpeg = wasab_service_state.latest_frame_jpeg
        meta = dict(wasab_service_state.latest_frame_meta) if wasab_service_state.latest_frame_meta else None
    if jpeg is None:
        raise HTTPException(status_code=404, detail="No latest camera preview frame is available.")
    try:
        frame = decode_image(jpeg)
    except ValueError as exc:
        raise HTTPException(status_code=500, detail=f"Latest camera preview decode failed: {exc}") from exc
    _validate_frame_size(frame)
    result = _create_grasp_plan_from_frame(
        frame=frame,
        current_flange_coords=current_flange_coords,
        request_id=request_id,
        frame_source="latest_camera_preview",
    )
    if meta is not None:
        result["latest_frame_meta"] = meta
    return result


@app.post("/latest-frame-grasp-plan")
def latest_frame_grasp_plan(request: LatestFrameGraspPlanRequest) -> dict[str, Any]:
    return latest_frame_grasp_plan_v1(request)


@app.post("/v1/marker-place-plan")
def marker_place_plan_v1(request: MarkerPlacePlanRequest) -> dict[str, Any]:
    if not isinstance(request.flange_coords, list) or len(request.flange_coords) != 6:
        raise HTTPException(status_code=400, detail="flange_coords must contain six values")
    try:
        current_flange_coords = [float(value) for value in request.flange_coords]
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail="flange_coords must be numeric") from exc
    if not np.isfinite(np.asarray(current_flange_coords, dtype=np.float64)).all():
        raise HTTPException(status_code=400, detail="flange_coords contains non-finite values")

    marker = request.marker_detection
    bbox = marker.get("bbox") if isinstance(marker, dict) else None
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise HTTPException(status_code=400, detail="marker_detection.bbox must be [x1,y1,x2,y2]")

    picked_target_label = (request.picked_target_label or "").strip()
    marker_role = str(marker.get("role", "")).strip().lower()
    black_table_place = marker_role == "black_table"
    place_offset_label = "box" if black_table_place else (picked_target_label or "place")
    raw_marker_id = marker.get("id") if isinstance(marker, dict) else None
    marker_id: int | None = None
    if raw_marker_id is not None:
        try:
            marker_id = int(raw_marker_id)
        except (TypeError, ValueError) as exc:
            raise HTTPException(status_code=400, detail="marker_detection.id must be an integer") from exc
    marker_place_base_offset = (
        list(settings.marker_place_tcp_offset_base_mm)
        if request.target_base_offset_mm is None
        else request.target_base_offset_mm
    )
    if len(marker_place_base_offset) != 3:
        raise HTTPException(
            status_code=400,
            detail="target_base_offset_mm must contain three values",
        )
    try:
        marker_place_base_offset = [
            float(value) for value in marker_place_base_offset
        ]
    except (TypeError, ValueError) as exc:
        raise HTTPException(
            status_code=400,
            detail="target_base_offset_mm must be numeric",
        ) from exc

    detection = {
        "label": place_offset_label,
        "class_id": -1,
        "confidence": 1.0,
        "bbox": [float(value) for value in bbox],
        "target_z_offset_add_mm": settings.place_extra_z_offset_mm,
        "target_base_offset_mm": (
            [0.0, 0.0, 0.0]
            if black_table_place
            else marker_place_base_offset
        ),
    }
    if marker_role == "recycle_bin":
        center = marker.get("center")
        if not isinstance(center, list) or len(center) != 2:
            raise HTTPException(
                status_code=400,
                detail="recycle_bin marker_detection.center must be [u,v]",
            )
        orientation = marker.get("flange_orientation_deg")
        if not isinstance(orientation, list) or len(orientation) != 3:
            raise HTTPException(
                status_code=400,
                detail=(
                    "recycle_bin marker_detection.flange_orientation_deg "
                    "must contain three values"
                ),
            )
        try:
            detection["center"] = [float(value) for value in center]
            detection["object_plane_z_base_mm"] = float(
                marker["object_plane_z_base_mm"]
            )
            detection["flange_orientation_deg"] = [
                float(value) for value in orientation
            ]
        except (KeyError, TypeError, ValueError) as exc:
            raise HTTPException(
                status_code=400,
                detail="invalid recycle_bin center, plane Z, or orientation",
            ) from exc
    if black_table_place:
        try:
            detection["object_plane_z_base_mm"] = float(
                marker.get("object_plane_z_base_mm", settings.object_plane_z_base_mm)
            )
        except (TypeError, ValueError) as exc:
            raise HTTPException(
                status_code=400,
                detail="black_table object_plane_z_base_mm must be numeric",
            ) from exc
        if marker.get("grip_axis_image_deg") is not None:
            detection["grip_axis_image_deg"] = marker["grip_axis_image_deg"]
    try:
        with wasab_service_state.target_z_offsets_lock:
            place_target_z_offset_mm = target_z_offset_for_label(place_offset_label, settings)
            effective_place_extra_z_offset_mm = settings.place_extra_z_offset_mm
            class_target_z_offsets_mm = dict(settings.target_z_offsets_mm)
            result = compute_wasab_operation_plan(
                detection=detection,
                current_flange_coords=current_flange_coords,
                calibration=wasab_service_state.calibration,
                settings=settings,
            )
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"Invalid marker bbox: {exc}") from exc
    except WaSaBOperationPlanError as exc:
        raise HTTPException(status_code=422, detail=f"Marker place geometry error: {exc}") from exc

    return {
        "status": "ok",
        "request_id": request.request_id,
        "marker_detection": marker,
        "current_flange_coords": current_flange_coords,
        "server_policy": {
            "object_plane_z_base_mm": settings.object_plane_z_base_mm,
            "picked_target_label": picked_target_label or None,
            "place_offset_label": place_offset_label,
            "marker_id": marker_id,
            "marker_role": marker_role or None,
            "black_table_place": black_table_place,
            "marker_place_tcp_offset_base_mm": marker_place_base_offset,
            "place_base_target_z_offset_mm": place_target_z_offset_mm,
            "place_extra_z_offset_mm": effective_place_extra_z_offset_mm,
            "place_target_z_offset_mm": place_target_z_offset_mm + settings.place_extra_z_offset_mm,
            "pick_flange_orientation_deg": (
                list(settings.pick_flange_orientation_deg)
                if settings.pick_flange_orientation_deg is not None
                else None
            ),
            "class_target_z_offsets_mm": class_target_z_offsets_mm,
        },
        **result,
    }


@app.post("/marker-place-plan")
def marker_place_plan(request: MarkerPlacePlanRequest) -> dict[str, Any]:
    return marker_place_plan_v1(request)


@app.post("/v1/marker-pickup-plan")
def marker_pickup_plan_v1(request: MarkerPickupPlanRequest) -> dict[str, Any]:
    if len(request.flange_coords) != 6:
        raise HTTPException(status_code=400, detail="flange_coords must contain six values")
    marker = request.marker_detection
    bbox = marker.get("bbox") if isinstance(marker, dict) else None
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise HTTPException(status_code=400, detail="marker_detection.bbox must be [x1,y1,x2,y2]")
    if len(request.target_base_offset_mm) != 3 or len(request.flange_orientation_deg) != 3:
        raise HTTPException(status_code=400, detail="pickup offsets/orientation must contain three values")

    label = "marker_pickup"
    base_z_offset = target_z_offset_for_label(label, settings)
    detection = {
        "label": label,
        "class_id": -1,
        "confidence": 1.0,
        "bbox": [float(value) for value in bbox],
        "object_plane_z_base_mm": float(request.marker_plane_z_base_mm),
        "target_z_offset_add_mm": float(request.target_z_offset_mm) - base_z_offset,
        "target_base_offset_mm": [float(value) for value in request.target_base_offset_mm],
        "flange_orientation_deg": [float(value) for value in request.flange_orientation_deg],
    }
    try:
        result = compute_wasab_operation_plan(
            detection=detection,
            current_flange_coords=[float(value) for value in request.flange_coords],
            calibration=wasab_service_state.calibration,
            settings=settings,
        )
    except (TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=f"Invalid marker pickup request: {exc}") from exc
    except WaSaBOperationPlanError as exc:
        raise HTTPException(status_code=422, detail=f"Marker pickup geometry error: {exc}") from exc
    return {
        "status": "ok",
        "request_id": request.request_id,
        "marker_detection": marker,
        "current_flange_coords": request.flange_coords,
        "server_policy": {
            "marker_pickup_target_z_offset_mm": request.target_z_offset_mm,
            "marker_pickup_plane_z_base_mm": request.marker_plane_z_base_mm,
            "marker_pickup_target_base_offset_mm": request.target_base_offset_mm,
            "marker_pickup_flange_orientation_deg": request.flange_orientation_deg,
        },
        **result,
    }


# 이전 분리형 클라이언트와 호환되는 alias입니다.
@app.post("/v1/grasp-plan")
async def grasp_plan_v1(
    image: UploadFile = File(...),
    robot_state: str = Form(...),
    target_label: str | None = Form(None),
) -> dict[str, Any]:
    return await _create_grasp_plan(
        image=image,
        robot_state=robot_state,
        target_label=target_label,
    )
