#!/usr/bin/env python3
# encoding: utf-8
"""노트북 로컬 YOLO 검출 + 3D 파지계획 FastAPI 서비스.

라즈베리파이는 SSH 터널 없이 노트북의 LAN IP로 프레임과 현재 Flange pose를
전송한다. ``/detect``, ``/grasp-plan``, ``/v1/grasp-plan`` 요청 형식과
응답 형식은 기존 원격 딥러닝 서버와 호환된다.
"""
from __future__ import annotations

import json
import socket
import struct
import threading
import time
from contextlib import asynccontextmanager
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

from app.components.wasab_op_service.geometry import WaSaBCalibration, WaSaBOperationPlanError, compute_wasab_operation_plan, load_wasab_calibration
from app.settings import settings
from app.components.wasab_web_service.commands import COMMAND_ALIASES


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
    detection_overlay_until: float
    detection_overlay_detections: list[WaSaBObjectDetection]
    detection_overlay_summary: str | None
    command_lock: Lock
    command_condition: Condition
    command_queue: list[dict[str, Any]]
    command_seq: int
    udp_stream_stop: threading.Event | None
    udp_stream_thread: threading.Thread | None


wasab_service_state = WaSaBServiceState()


@asynccontextmanager
async def lifespan(_: FastAPI):
    if settings.euler_order != "zyx":
        raise RuntimeError("This project currently supports only EULER_ORDER=zyx")
    if not settings.model_path.exists():
        raise FileNotFoundError(f"YOLO model not found: {settings.model_path}")

    # 모델과 calibration은 서버 시작 시 한 번만 로드합니다.
    wasab_service_state.model = YOLO(str(settings.model_path))
    wasab_service_state.calibration = load_wasab_calibration(settings)
    wasab_service_state.inference_lock = Lock()
    wasab_service_state.latest_frame_lock = Lock()
    wasab_service_state.latest_frame_jpeg = None
    wasab_service_state.latest_frame_meta = None
    wasab_service_state.detection_overlay_until = 0.0
    wasab_service_state.detection_overlay_detections = []
    wasab_service_state.detection_overlay_summary = None
    wasab_service_state.command_lock = Lock()
    wasab_service_state.command_condition = Condition(wasab_service_state.command_lock)
    wasab_service_state.command_queue = []
    wasab_service_state.command_seq = 0
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
    if settings.udp_stream_enabled:
        print("[STARTUP] UDP Streamer receiver:", f"{settings.udp_stream_host}:{settings.udp_stream_port}")
    try:
        yield
    finally:
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
        "default_target_label": settings.default_target_label,
        "euler_order": settings.euler_order,
        "calibration_method": wasab_service_state.calibration.selected_method,
    }


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
      background: #111;
      color: #eee;
    }
    header {
      display: flex;
      gap: 16px;
      align-items: center;
      justify-content: space-between;
      padding: 12px 16px;
      border-bottom: 1px solid #333;
      background: #181818;
    }
    h1 {
      margin: 0;
      font-size: 18px;
      font-weight: 650;
      letter-spacing: 0;
    }
    #status {
      font-size: 14px;
      color: #bbb;
      white-space: nowrap;
    }
    .actions {
      display: flex;
      gap: 8px;
      align-items: center;
      flex-wrap: wrap;
      justify-content: flex-end;
    }
    button {
      border: 1px solid #444;
      border-radius: 6px;
      background: #242424;
      color: #f2f2f2;
      min-width: 42px;
      height: 34px;
      padding: 0 12px;
      font: inherit;
      font-size: 14px;
      cursor: pointer;
    }
    button:hover { background: #303030; }
    button:active { transform: translateY(1px); }
    button.danger {
      border-color: #7a2d2d;
      background: #3a1d1d;
    }
    button.stop {
      border-color: #b91c1c;
      background: #7f1d1d;
      font-weight: 750;
    }
    main {
      min-height: calc(100vh - 50px);
      display: grid;
      place-items: center;
      padding: 16px;
      box-sizing: border-box;
    }
    img {
      max-width: 100%;
      max-height: calc(100vh - 92px);
      background: #050505;
      border: 1px solid #333;
      object-fit: contain;
    }
  </style>
</head>
<body>
  <header>
    <h1>WaSaB AdminGUI</h1>
    <div class="actions">
      <button id="liveButton" title="show live camera stream">Live</button>
      <button id="detectButton" title="run YOLO on latest frame and show scores">Detect</button>
      <button id="captureButton" title="save current camera capture on server">Capture</button>
      <button data-command="pick" title="plan and pick">Pick</button>
      <button data-command="place" title="return home, then place at the last picked position">Place</button>
      <button data-command="pose" title="print current pose">Pose</button>
      <button data-command="gripper" title="toggle gripper">Gripper</button>
      <button data-command="random" title="safe random pose">Random</button>
      <button data-command="servo-release" title="release all servos">Servo Release</button>
      <button data-command="servo-focus" title="focus all servos">Servo Focus</button>
      <button class="stop" data-command="stop" title="stop current motion immediately">STOP</button>
      <button data-command="find-marker" title="scan until an April marker is visible">Find Marker</button>
      <button data-command="home" title="force home">Home</button>
      <button class="danger" data-command="exit" title="stop and quit">Exit</button>
      <div id="status">waiting for frame...</div>
    </div>
  </header>
  <main>
    <img id="frame" alt="latest robot camera frame">
  </main>
  <script>
    const img = document.getElementById("frame");
    const statusEl = document.getElementById("status");

    async function sendCommand(command) {
      statusEl.textContent = `sending ${command}...`;
      try {
        const response = await fetch(`/robot-command/${command}`, {
          method: "POST",
          cache: "no-store",
        });
        const data = await response.json();
        if (!response.ok) {
          statusEl.textContent = data.detail || `command ${command} failed`;
          return;
        }
        statusEl.textContent = `queued ${data.command} #${data.id}`;
      } catch {
        statusEl.textContent = "command send failed";
      }
    }

    document.querySelectorAll("button[data-command]").forEach((button) => {
      button.addEventListener("click", () => sendCommand(button.dataset.command));
    });

    const keyCommands = {
      g: "pick",
      p: "pose",
      q: "gripper",
      r: "random",
      s: "servo-release",
      k: "servo-focus",
      f: "place",
      a: "find-marker",
      t: "throw",
      w: "home",
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

    function showLiveStream() {
      img.src = `/camera-frame/stream.mjpg?t=${Date.now()}`;
      statusEl.textContent = "live stream";
    }

    async function showWaSaBObjectDetectionPreview() {
      statusEl.textContent = "running YOLO...";
      try {
        const response = await fetch(`/camera-frame/detect?t=${Date.now()}`, {
          method: "POST",
          cache: "no-store",
        });
        const data = await response.json();
        if (!response.ok) {
          statusEl.textContent = data.detail || "detect failed";
          return;
        }
        statusEl.textContent = `YOLO ${data.detection_count} objects | ${data.inference_ms.toFixed(1)}ms`;
      } catch {
        statusEl.textContent = "detect request failed";
      }
    }

    async function captureFrame() {
      statusEl.textContent = "saving capture...";
      try {
        const response = await fetch(`/camera-frame/capture?t=${Date.now()}`, {
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

    showLiveStream();

    async function refreshStatus() {
      const cacheBust = Date.now();
      try {
        const response = await fetch(`/camera-frame/status?t=${cacheBust}`, { cache: "no-store" });
        if (!response.ok) {
          statusEl.textContent = "waiting for arm camera stream...";
          return;
        }
        const data = await response.json();
        statusEl.textContent = `${data.width}x${data.height} | ${data.source} | ${data.age_sec.toFixed(1)}s ago`;
      } catch {
        statusEl.textContent = "server not reachable";
      }
    }

    refreshStatus();
    setInterval(refreshStatus, 1000);
  </script>
</body>
</html>"""


@app.post("/robot-command/{command}")
def enqueue_wasab_arm_command(command: str) -> dict[str, Any]:
    """브라우저 버튼에서 Pi 클라이언트가 실행할 이름 기반 명령을 큐에 넣습니다."""
    requested = command.lower().strip()
    normalized = COMMAND_ALIASES.get(requested)
    if normalized is None:
        allowed = ", ".join(sorted(set(COMMAND_ALIASES.values())))
        raise HTTPException(status_code=400, detail=f"command must be one of: {allowed}")

    now = time.time()
    with wasab_service_state.command_lock:
        wasab_service_state.command_seq += 1
        item = {
            "id": wasab_service_state.command_seq,
            "command": normalized,
            "timestamp": now,
            "timestamp_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
            "source": "camera-view",
        }
        if normalized == "stop":
            wasab_service_state.command_queue.clear()
        wasab_service_state.command_queue.append(item)
        pending = len(wasab_service_state.command_queue)
        wasab_service_state.command_condition.notify_all()
    print(
        "[ROBOT COMMAND] queued",
        normalized,
        f"id={item['id']}",
        f"pending={pending}",
        f"source={item['source']}",
    )
    return {"status": "queued", "pending": pending, **item}


@app.get("/robot-command/stream")
def stream_wasab_arm_commands() -> StreamingResponse:
    """Stream queued browser commands to the Pi without polling."""
    def generate():
        while True:
            with wasab_service_state.command_condition:
                while not wasab_service_state.command_queue:
                    wasab_service_state.command_condition.wait(timeout=15.0)
                    if not wasab_service_state.command_queue:
                        yield json.dumps({"status": "heartbeat"}).encode() + b"\n"
                item = wasab_service_state.command_queue.pop(0)
                pending = len(wasab_service_state.command_queue)
            payload = {"status": "ok", "pending": pending, **item}
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
            store_uploaded_streamer_jpeg(raw_jpeg, frame, "udp-stream")
        except Exception as exc:
            print(f"[UDP STREAM] dropped frame {frame_id} from {addr[0]}: {exc}")

    sock.close()


def decode_image(file_bytes: bytes) -> np.ndarray:
    np_arr = np.frombuffer(file_bytes, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("이미지 디코딩 실패")
    return image


def _latest_frame_meta(frame: np.ndarray, source: str) -> dict[str, Any]:
    h, w = frame.shape[:2]
    now = time.time()
    return {
        "timestamp": now,
        "timestamp_iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
        "width": w,
        "height": h,
        "source": source,
    }


def store_latest_stream_frame(frame: np.ndarray, source: str) -> None:
    ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
    if not ok:
        raise ValueError("최신 카메라 프레임 JPEG 인코딩 실패")
    with wasab_service_state.latest_frame_lock:
        wasab_service_state.latest_frame_jpeg = encoded.tobytes()
        wasab_service_state.latest_frame_meta = _latest_frame_meta(frame, source)


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


def store_uploaded_streamer_jpeg(raw_jpeg: bytes, frame: np.ndarray, source: str) -> None:
    # Preview frames are already JPEG-encoded on the Pi. Reusing those bytes avoids
    # a decode/re-encode cycle unless a short-lived detection overlay is active.
    with wasab_service_state.latest_frame_lock:
        if time.time() < wasab_service_state.detection_overlay_until:
            annotated = draw_wasab_detections(frame, wasab_service_state.detection_overlay_detections)
            _draw_detection_summary(annotated, wasab_service_state.detection_overlay_summary)
            ok, encoded = cv2.imencode(".jpg", annotated, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
            if ok:
                wasab_service_state.latest_frame_jpeg = encoded.tobytes()
            else:
                wasab_service_state.latest_frame_jpeg = raw_jpeg
        else:
            wasab_service_state.detection_overlay_detections = []
            wasab_service_state.detection_overlay_summary = None
            wasab_service_state.latest_frame_jpeg = raw_jpeg
        wasab_service_state.latest_frame_meta = _latest_frame_meta(frame, source)


@app.post("/camera-frame")
async def receive_streamer_frame(image: UploadFile = File(...)) -> dict[str, Any]:
    """라즈베리파이/Jetcobot 클라이언트가 실시간 보기용 최신 프레임을 업로드합니다."""
    raw = await image.read()
    frame = _read_and_decode_upload(image, raw)
    store_uploaded_streamer_jpeg(raw, frame, "stream")
    with wasab_service_state.latest_frame_lock:
        meta = dict(wasab_service_state.latest_frame_meta or {})
    return {"status": "ok", **meta}


@app.post("/camera-frame/capture")
def capture_streamer_frame() -> dict[str, Any]:
    """Save the latest camera-view JPEG on the laptop/server side."""
    with wasab_service_state.latest_frame_lock:
        jpeg = wasab_service_state.latest_frame_jpeg
        meta = dict(wasab_service_state.latest_frame_meta) if wasab_service_state.latest_frame_meta else None
    if jpeg is None or meta is None:
        raise HTTPException(status_code=404, detail="아직 업로드된 카메라 프레임이 없습니다.")

    capture_dir = Path(__file__).resolve().parents[1] / "capture"
    capture_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    filename = f"jetcobot_capture_{timestamp}.jpg"
    path = capture_dir / filename
    path.write_bytes(jpeg)
    return {
        "status": "ok",
        "filename": filename,
        "path": str(path),
        **meta,
    }


@app.get("/camera-frame/latest.jpg")
def latest_streamer_frame() -> Response:
    with wasab_service_state.latest_frame_lock:
        jpeg = wasab_service_state.latest_frame_jpeg
    if jpeg is None:
        raise HTTPException(status_code=404, detail="아직 업로드된 카메라 프레임이 없습니다.")
    return Response(
        content=jpeg,
        media_type="image/jpeg",
        headers={"Cache-Control": "no-store, max-age=0"},
    )


@app.post("/camera-frame/detect")
def run_streamer_frame_detection(
    conf: float = Query(settings.default_conf, ge=0.0, le=1.0),
    imgsz: int = Query(settings.default_imgsz, ge=32),
) -> dict[str, Any]:
    """Run YOLO on the latest uploaded frame and publish the annotated result to MJPEG."""
    with wasab_service_state.latest_frame_lock:
        jpeg = wasab_service_state.latest_frame_jpeg
        meta = dict(wasab_service_state.latest_frame_meta) if wasab_service_state.latest_frame_meta else None
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

    annotated = draw_wasab_detections(frame, detections)
    summary = f"detections={len(detections)} inference_ms={inference_ms:.1f}"
    _draw_detection_summary(annotated, summary)
    with wasab_service_state.latest_frame_lock:
        wasab_service_state.detection_overlay_until = time.time() + 3.0
        wasab_service_state.detection_overlay_detections = detections
        wasab_service_state.detection_overlay_summary = summary
    store_latest_stream_frame(annotated, "detect-result")
    return {
        "status": "ok",
        "detection_count": len(detections),
        "inference_ms": inference_ms,
        "detections": [_dump(det) for det in detections],
    }


@app.get("/camera-frame/stream.mjpg")
def stream_camera_mjpeg() -> StreamingResponse:
    def generate():
        last_timestamp = None
        while True:
            with wasab_service_state.latest_frame_lock:
                jpeg = wasab_service_state.latest_frame_jpeg
                meta = dict(wasab_service_state.latest_frame_meta) if wasab_service_state.latest_frame_meta else None
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
def streamer_frame_status() -> dict[str, Any]:
    with wasab_service_state.latest_frame_lock:
        meta = dict(wasab_service_state.latest_frame_meta) if wasab_service_state.latest_frame_meta else None
    if meta is None:
        raise HTTPException(status_code=404, detail="아직 업로드된 카메라 프레임이 없습니다.")
    meta["age_sec"] = max(0.0, time.time() - float(meta["timestamp"]))
    return {"status": "ok", **meta}


def draw_wasab_detections(frame: np.ndarray, detections: list[WaSaBObjectDetection]) -> np.ndarray:
    annotated = frame.copy()
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


def _effective_target_label(requested: str | None) -> str | None:
    # API query가 주어지면 그 값을 사용하되, 없으면 서버 정책의 기본 클래스를 사용합니다.
    return requested if requested is not None else settings.default_target_label


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

        u = (x1 + x2) / 2.0
        v = (y1 + y2) / 2.0
        detections.append(
            WaSaBObjectDetection(
                label=label,
                class_id=class_id,
                confidence=score,
                bbox=[x1, y1, x2, y2],
                center=[u, v],
                width=x2 - x1,
                height=y2 - y1,
            )
        )

    detections.sort(key=lambda item: item.confidence, reverse=True)
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


def _parse_robot_state(raw: str) -> tuple[list[float], str | None]:
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
    return parsed, str(request_id) if request_id is not None else None


# ============================================================
# 5. 기존 YOLO detect API (호환 유지)
# ============================================================

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
    effective_target_label = _effective_target_label(target_label)

    try:
        detections, inference_ms = _run_inference(
            frame=frame,
            conf=conf,
            imgsz=imgsz,
            target_label=effective_target_label,
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
        target_label=effective_target_label,
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

async def _create_grasp_plan(
    *,
    image: UploadFile,
    robot_state: str,
) -> dict[str, Any]:
    """노트북의 고정 정책으로 파지 계획을 생성합니다.

    라즈베리파이는 frame과 촬영 시점의 현재 Flange pose만 보내며,
    YOLO 및 모든 좌표변환은 이 함수 내부에서 완료합니다.
    """
    current_flange_coords, request_id = _parse_robot_state(robot_state)
    raw = await image.read()
    frame = _read_and_decode_upload(image, raw)
    h, w = frame.shape[:2]

    try:
        detections, inference_ms = _run_inference(
            frame=frame,
            conf=settings.default_conf,
            imgsz=settings.default_imgsz,
            target_label=settings.default_target_label,
        )
    except Exception as exc:
        raise HTTPException(status_code=500, detail=f"YOLO inference failed: {exc}") from exc

    annotated = draw_wasab_detections(frame, detections)
    store_latest_stream_frame(annotated, "grasp-plan-result")
    extra_result: dict[str, Any] = {
        "request_id": request_id,
        "current_flange_coords": current_flange_coords,
    }

    if not detections:
        extra_result.update({"status": "not_found", "message": "Target object was not detected."})
        save_dir, raw_path, annotated_path, json_path = save_ai_detection_result(
            frame=frame,
            annotated=annotated,
            detections=detections,
            image_width=w,
            image_height=h,
            inference_ms=inference_ms,
            conf=settings.default_conf,
            imgsz=settings.default_imgsz,
            target_label=settings.default_target_label,
            extra_result=extra_result,
        )
        return {
            "status": "not_found",
            "request_id": request_id,
            "message": "No target detection satisfied the server target-label/confidence policy.",
            "image_width": w,
            "image_height": h,
            "inference_ms": inference_ms,
            "detections": [_dump(det) for det in detections],
            "saved_dir": save_dir,
            "raw_image_path": raw_path,
            "annotated_image_path": annotated_path,
            "result_json_path": json_path,
        }

    # detections는 confidence 내림차순. 가장 높은 대상 하나를 서버가 선택합니다.
    selected = detections[0]
    selected_target_z_offset_mm = settings.target_z_offsets_mm.get(
        selected.label,
        settings.toothbrush_target_z_offset_mm,
    )
    try:
        result = compute_wasab_operation_plan(
            detection=_dump(selected),
            current_flange_coords=current_flange_coords,
            calibration=wasab_service_state.calibration,
            settings=settings,
        )
    except WaSaBOperationPlanError as exc:
        raise HTTPException(status_code=422, detail=f"Grasp-plan geometry error: {exc}") from exc

    extra_result.update({"status": "ok", **result})
    save_dir, raw_path, annotated_path, json_path = save_ai_detection_result(
        frame=frame,
        annotated=annotated,
        detections=detections,
        image_width=w,
        image_height=h,
        inference_ms=inference_ms,
        conf=settings.default_conf,
        imgsz=settings.default_imgsz,
        target_label=settings.default_target_label,
        extra_result=extra_result,
    )

    return {
        "status": "ok",
        "request_id": request_id,
        "image_width": w,
        "image_height": h,
        "inference_ms": inference_ms,
        "detections": [_dump(det) for det in detections],
        "server_policy": {
            "target_label": settings.default_target_label,
            "confidence_threshold": settings.default_conf,
            "object_plane_z_base_mm": settings.object_plane_z_base_mm,
            "default_target_z_offset_mm": settings.toothbrush_target_z_offset_mm,
            "selected_target_z_offset_mm": selected_target_z_offset_mm,
            "class_target_z_offsets_mm": settings.target_z_offsets_mm,
        },
        **result,
        "saved_dir": save_dir,
        "raw_image_path": raw_path,
        "annotated_image_path": annotated_path,
        "result_json_path": json_path,
    }


@app.post("/grasp-plan")
async def grasp_plan(
    image: UploadFile = File(...),
    robot_state: str = Form(...),
) -> dict[str, Any]:
    return await _create_grasp_plan(image=image, robot_state=robot_state)


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

    detection = {
        "label": "place",
        "class_id": -1,
        "confidence": 1.0,
        "bbox": [float(value) for value in bbox],
    }
    try:
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
            "place_target_z_offset_mm": settings.target_z_offsets_mm.get(
                "place",
                settings.toothbrush_target_z_offset_mm,
            ),
            "class_target_z_offsets_mm": settings.target_z_offsets_mm,
        },
        **result,
    }


@app.post("/marker-place-plan")
def marker_place_plan(request: MarkerPlacePlanRequest) -> dict[str, Any]:
    return marker_place_plan_v1(request)


# 이전 분리형 클라이언트와 호환되는 alias입니다.
@app.post("/v1/grasp-plan")
async def grasp_plan_v1(
    image: UploadFile = File(...),
    robot_state: str = Form(...),
) -> dict[str, Any]:
    return await _create_grasp_plan(image=image, robot_state=robot_state)
