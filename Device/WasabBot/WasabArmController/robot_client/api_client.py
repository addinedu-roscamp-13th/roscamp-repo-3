"""Raspberry Pi -> laptop-local YOLO/grasp-plan HTTP client.

The Pi sends:
  * a current JPEG frame at the calibration resolution
  * current Base-frame Flange pose [x, y, z, rx, ry, rz]

The laptop returns a detection and a Base-frame ``flange_command``.
The Pi still applies its own final workspace/safety validation before motion.
"""
from __future__ import annotations

import ipaddress
import json
import math
import socket
import struct
import time
import uuid
from itertools import count
from threading import Event
from typing import Any, Iterator
from urllib.parse import urlparse, urlunparse

import cv2
import requests

from . import config


_SESSION = requests.Session()
_UDP_STREAM_MAGIC = b"WASABU1"
_UDP_STREAM_HEADER = struct.Struct("!7sIHHH")
_UDP_FRAME_IDS = count(1)
_UDP_SOCKET = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class WaSaBServiceError(RuntimeError):
    """Raised when the laptop service is unreachable or returns an invalid plan."""


def _parsed_wasab_service_url():
    parsed = urlparse(config.GRASP_SERVER_URL)
    if parsed.scheme not in {"http", "https"} or not parsed.hostname:
        raise WaSaBServiceError(
            "Invalid grasp_server_url. Set the laptop LAN address, for example: "
            "http://192.168.0.20:8000/v1/grasp-plan"
        )

    host = parsed.hostname.lower()
    if host in {"laptop_lan_ip", "your_laptop_ip"}:
        raise WaSaBServiceError(
            "grasp_server_url still contains a placeholder. Replace LAPTOP_LAN_IP "
            "with the laptop IPv4 address, for example 192.168.0.20."
        )

    if host in {"localhost", "127.0.0.1", "::1"} and not config.ALLOW_LOOPBACK_SERVER:
        raise WaSaBServiceError(
            "grasp_server_url points to loopback. On the Raspberry Pi, 127.0.0.1 "
            "means the Pi itself, not the laptop. Use the laptop LAN IPv4 address."
        )

    endpoint = parsed.path.rstrip("/")
    if endpoint not in {"/grasp-plan", "/v1/grasp-plan"}:
        raise WaSaBServiceError(
            "Laptop service endpoint must be /v1/grasp-plan (recommended) or "
            f"/grasp-plan, not {parsed.path!r}."
        )
    return parsed


def _wasab_health_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/health", "", "", ""))


def _detect_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse(
        (parsed.scheme, parsed.netloc, "/detect", "", "", "")
    )


def _streamer_frame_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((
        parsed.scheme, parsed.netloc, "/camera-frame", "", f"arm_id={config.ARM_ID}", ""
    ))


def _workspace_overlay_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((
        parsed.scheme,
        parsed.netloc,
        "/camera-frame/workspace",
        "",
        f"arm_id={config.ARM_ID}",
        "",
    ))


def _wasab_arm_command_stream_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((
        parsed.scheme, parsed.netloc, "/robot-command/stream", "", f"arm_id={config.ARM_ID}", ""
    ))


def _wasab_robot_logs_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/robot-logs", "", "", ""))


def post_robot_log(message: str, *, level: str = "info", source: str = "robot-client") -> None:
    """Best-effort operation log upload for the AdminGUI."""
    try:
        response = _SESSION.post(
            _wasab_robot_logs_url(),
            json={
                "message": str(message),
                "level": level,
                "source": f"{source}:{config.ARM_ID}",
            },
            timeout=(config.CONNECT_TIMEOUT_SEC, 2.0),
        )
        response.raise_for_status()
    except requests.RequestException as exc:
        print("[ROBOT LOG] upload failed:", exc)


def _marker_place_plan_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/v1/marker-place-plan", "", "", ""))


def _apriltag_detect_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse(
        (
            parsed.scheme,
            parsed.netloc,
            "/v1/apriltag-detect",
            "",
            "",
            "",
        )
    )


def _marker_pickup_plan_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/v1/marker-pickup-plan", "", "", ""))


def _latest_frame_grasp_plan_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/v1/latest-frame-grasp-plan", "", "", ""))


def _is_private_or_local_host(host: str) -> bool:
    if host.lower() in {"localhost", "127.0.0.1", "::1"}:
        return True
    try:
        return ipaddress.ip_address(host).is_private
    except ValueError:
        return False


def _wasab_connection_hint(exc: BaseException) -> str:
    """Give deployment-focused guidance for a direct Pi -> laptop LAN connection."""
    parsed = _parsed_wasab_service_url()
    host = parsed.hostname or "<unknown>"
    port = parsed.port or (443 if parsed.scheme == "https" else 80)

    try:
        resolved = sorted(
            {
                item[4][0]
                for item in socket.getaddrinfo(
                    host, port, type=socket.SOCK_STREAM
                )
            }
        )
        resolved_text = ", ".join(resolved)
    except OSError:
        resolved_text = "DNS/IP resolve failed"

    if isinstance(
        exc,
        (requests.exceptions.ConnectTimeout, requests.exceptions.ConnectionError),
    ):
        if _is_private_or_local_host(host):
            network_advice = (
                "Confirm that the laptop server is running with `python run_server.py`, "
                "that its config uses host = 0.0.0.0 and port = 8000, and that this "
                "Pi and laptop are on the same LAN. Allow inbound TCP 8000 for Python "
                "on the laptop's private-network firewall profile. From the Pi, test "
                f"`curl http://{host}:{port}/health`."
            )
        else:
            network_advice = (
                "The configured host is not a private LAN address. For this direct "
                "laptop deployment, use the laptop's current 192.168.x.x or 10.x.x.x "
                "address, or use a VPN such as Tailscale/WireGuard when the two "
                "machines are on different networks."
            )
        return (
            f"Connection failed to {host}:{port} (resolved: {resolved_text}). "
            f"{network_advice}"
        )

    return (
        f"Connection check failed for {host}:{port} "
        f"(resolved: {resolved_text}): {exc}"
    )


def check_wasab_service_health() -> dict[str, Any]:
    """Verify the direct network path and confirm this is the laptop-local service."""
    url = _wasab_health_url()
    try:
        response = _SESSION.get(
            url,
            timeout=(config.CONNECT_TIMEOUT_SEC, config.HEALTH_TIMEOUT_SEC),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc

    if not response.ok:
        raise WaSaBServiceError(
            f"Health check returned HTTP {response.status_code}: "
            f"{response.text[:300]}"
        )
    try:
        payload: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError(
            "Health endpoint response is not valid JSON"
        ) from exc

    if payload.get("status") != "ok":
        raise WaSaBServiceError(
            f"Health endpoint reported an error: {payload}"
        )

    expected_runtime = config.EXPECTED_SERVER_RUNTIME
    if expected_runtime and payload.get("runtime") != expected_runtime:
        raise WaSaBServiceError(
            "Connected service is not the expected laptop-local server: "
            f"expected runtime={expected_runtime!r}, "
            f"received runtime={payload.get('runtime')!r}."
        )
    return payload


def _udp_streamer_host() -> str:
    if config.UDP_STREAM_HOST:
        return config.UDP_STREAM_HOST
    parsed = _parsed_wasab_service_url()
    return parsed.hostname or "127.0.0.1"


def send_udp_streamer_frame(frame) -> dict[str, Any]:
    """Send the latest WaSaBArm Streamer frame to WaSaBWebService over UDP."""
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), config.CAMERA_STREAM_JPEG_QUALITY],
    )
    if not ok:
        raise WaSaBServiceError("UDP Streamer JPEG encoding failed")

    jpeg = encoded.tobytes()
    max_payload = config.UDP_STREAM_MAX_DATAGRAM_BYTES - _UDP_STREAM_HEADER.size
    if max_payload <= 0:
        raise WaSaBServiceError("udp_stream max_datagram_bytes is too small")

    chunk_count = max(1, math.ceil(len(jpeg) / max_payload))
    if chunk_count > 65535:
        raise WaSaBServiceError(f"UDP Streamer frame is too large: chunks={chunk_count}")

    frame_id = next(_UDP_FRAME_IDS) & 0xFFFFFFFF
    target = (_udp_streamer_host(), config.UDP_STREAM_PORT)
    for chunk_index in range(chunk_count):
        start = chunk_index * max_payload
        payload = jpeg[start:start + max_payload]
        header = _UDP_STREAM_HEADER.pack(
            _UDP_STREAM_MAGIC,
            frame_id,
            chunk_index,
            chunk_count,
            len(payload),
        )
        _UDP_SOCKET.sendto(header + payload, target)

    return {
        "status": "ok",
        "transport": "udp",
        "host": target[0],
        "port": target[1],
        "bytes": len(jpeg),
        "chunks": chunk_count,
    }


def upload_streamer_frame(frame) -> dict[str, Any]:
    """Upload the latest camera frame for the laptop /camera-view page."""
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), config.CAMERA_STREAM_JPEG_QUALITY],
    )
    if not ok:
        raise WaSaBServiceError("Camera preview JPEG encoding failed")

    files = {"image": ("frame.jpg", encoded.tobytes(), "image/jpeg")}
    url = _streamer_frame_url()
    try:
        preview_timeout = min(config.CONNECT_TIMEOUT_SEC, config.CAMERA_STREAM_TIMEOUT_SEC)
        response = _SESSION.post(
            url,
            files=files,
            timeout=(preview_timeout, config.CAMERA_STREAM_TIMEOUT_SEC),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc

    if not response.ok:
        raise WaSaBServiceError(
            f"Camera preview upload HTTP {response.status_code}: {response.text[:300]}"
        )
    try:
        payload: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError("Camera preview response is not valid JSON") from exc
    return payload


def upload_palm_hitbox_capture(frame) -> dict[str, Any]:
    """Upload the separate 3-second Palm Check hitbox capture to the laptop."""
    ok, encoded = cv2.imencode(".png", frame)
    if not ok:
        raise WaSaBServiceError("Palm hitbox PNG encoding failed")
    parsed = _parsed_wasab_service_url()
    url = urlunparse((
        parsed.scheme,
        parsed.netloc,
        "/palm-hitbox-capture",
        "",
        f"arm_id={config.ARM_ID}",
        "",
    ))
    response = _SESSION.post(
        url,
        files={"image": ("palm_hitbox_capture.png", encoded.tobytes(), "image/png")},
        timeout=(config.CONNECT_TIMEOUT_SEC, config.CAMERA_STREAM_TIMEOUT_SEC),
    )
    if not response.ok:
        raise WaSaBServiceError(
            f"Palm hitbox capture upload HTTP {response.status_code}: "
            f"{response.text[:300]}"
        )
    return response.json()


def update_workspace_overlay(flange_coords: list[float]) -> None:
    """Best-effort update of the camera-view reachable-workspace overlay."""
    payload = {
        "flange_coords": [float(value) for value in flange_coords],
        "safe_x_mm": list(config.SAFE_X_MM),
        "safe_y_mm": list(config.SAFE_Y_MM),
        "safe_z_mm": list(config.SAFE_Z_MM),
        "object_plane_z_base_mm": config.MARKER_PICKUP_PLANE_Z_BASE_MM,
        "target_z_offset_mm": config.MARKER_PICKUP_TARGET_Z_OFFSET_MM,
        "target_base_offset_mm": list(config.MARKER_PICKUP_TARGET_BASE_OFFSET_MM),
        "flange_orientation_deg": list(config.MARKER_PICKUP_FLANGE_COORDS[3:]),
    }
    try:
        _SESSION.post(
            _workspace_overlay_url(),
            json=payload,
            timeout=(config.CONNECT_TIMEOUT_SEC, min(1.0, config.REQUEST_TIMEOUT_SEC)),
        ).raise_for_status()
    except requests.RequestException:
        return


def stream_wasab_arm_commands(stop_event: Event) -> Iterator[str]:
    """Yield browser commands from a persistent server-push stream."""
    url = _wasab_arm_command_stream_url()
    with requests.Session() as session:
        try:
            with session.get(
                url,
                stream=True,
                timeout=(config.CONNECT_TIMEOUT_SEC, None),
            ) as response:
                if not response.ok:
                    raise WaSaBServiceError(
                        f"Remote command stream HTTP {response.status_code}: {response.text[:300]}"
                    )

                for line in response.iter_lines(chunk_size=1, decode_unicode=True):
                    if stop_event.is_set():
                        break
                    if not line:
                        continue
                    try:
                        payload: dict[str, Any] = json.loads(line)
                    except ValueError as exc:
                        raise WaSaBServiceError("Remote command stream sent invalid JSON") from exc

                    if payload.get("status") == "heartbeat":
                        continue
                    if payload.get("status") != "ok":
                        raise WaSaBServiceError(f"Remote command stream returned: {payload}")

                    raw_command = payload.get("command")
                    command = str(raw_command).lower().strip() if raw_command is not None else ""
                    command_aliases = {
                        "gesture_on": "gesture-on",
                        "gesture on": "gesture-on",
                        "gesture_off": "gesture-off",
                        "gesture off": "gesture-off",
                        "emergency_stop": "emergency-stop",
                        "emergency stop": "emergency-stop",
                        "calibrate": "calibration",
                    }
                    command = command_aliases.get(command, command)
                    valid_commands = {
                        "g", "p", "q", "r", "s", "k", "f", "m", "w", "x",
                        "pick", "pick-place", "gift-supply-pick", "restock", "pickup-tuning",
                        "recycle", "help",
                        "pose", "gripper", "servo-release",
                        "servo-focus", "place", "move", "home", "stop", "halt",
                        "emergency-stop", "gesture-on", "gesture-off", "exit",
                        "vision-sweep-on", "vision-sweep-off",
                        "vision-sweep-face-on", "vision-sweep-fire-on",
                        "vision-sweep-tracking-on",
                        "fire-suppress-close", "fire-suppress-open",
                        "palm-check",
                        "calibration",
                    }
                    if (
                        command not in valid_commands
                        and not command.startswith("palm-hitbox-target:")
                        and not command.startswith("vision-track:")
                    ):
                        raise WaSaBServiceError(f"Invalid remote command received: {raw_command!r}")

                    print(
                        "[REMOTE COMMAND]",
                        command,
                        f"id={payload.get('id')}",
                        f"pending={payload.get('pending')}",
                    )
                    yield command
        except requests.RequestException as exc:
            raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc


def request_wasab_marker_place_plan(
    marker_detection: dict[str, Any],
    flange_coords: list[float],
    picked_target_label: str | None = None,
    target_base_offset_mm: list[float] | None = None,
) -> dict[str, Any]:
    """Ask the laptop to convert an AprilTag bbox into a place Flange command."""
    if not isinstance(flange_coords, list) or len(flange_coords) != 6:
        raise WaSaBServiceError("flange_coords must contain six values")
    bbox = marker_detection.get("bbox") if isinstance(marker_detection, dict) else None
    if not isinstance(bbox, list) or len(bbox) != 4:
        raise WaSaBServiceError("marker_detection.bbox must contain four values")

    request_id = str(uuid.uuid4())
    payload = {
        "request_id": request_id,
        "flange_coords": [float(value) for value in flange_coords],
        "marker_detection": marker_detection,
    }
    if picked_target_label:
        payload["picked_target_label"] = str(picked_target_label)
    if target_base_offset_mm is not None:
        if len(target_base_offset_mm) != 3:
            raise WaSaBServiceError(
                "target_base_offset_mm must contain three values"
            )
        payload["target_base_offset_mm"] = [
            float(value) for value in target_base_offset_mm
        ]

    try:
        response = _SESSION.post(
            _marker_place_plan_url(),
            json=payload,
            timeout=(config.CONNECT_TIMEOUT_SEC, config.REQUEST_TIMEOUT_SEC),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc

    if not response.ok:
        raise WaSaBServiceError(
            f"Laptop marker-place HTTP {response.status_code}: {response.text[:800]}"
        )
    try:
        result: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError("Laptop marker-place response is not valid JSON") from exc

    if result.get("request_id") not in (None, request_id):
        raise WaSaBServiceError("Laptop marker-place request_id mismatch")
    return result


def request_wasab_apriltag_detection(
    frame,
    target_id: int,
) -> dict[str, Any]:
    """Detect one AprilTag on the laptop so Pi OpenCV cannot crash control."""
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), config.JPEG_QUALITY],
    )
    if not ok:
        raise WaSaBServiceError("AprilTag frame JPEG encoding failed")
    try:
        response = _SESSION.post(
            _apriltag_detect_url(),
            files={
                "image": (
                    "apriltag-frame.jpg",
                    encoded.tobytes(),
                    "image/jpeg",
                )
            },
            data={
                "target_id": str(int(target_id)),
                "arm_id": config.ARM_ID,
            },
            timeout=(
                config.CONNECT_TIMEOUT_SEC,
                config.REQUEST_TIMEOUT_SEC,
            ),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc
    if not response.ok:
        raise WaSaBServiceError(
            "Laptop AprilTag detection HTTP "
            f"{response.status_code}: {response.text[:800]}"
        )
    try:
        result: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError(
            "Laptop AprilTag detection response is not valid JSON"
        ) from exc
    return result


def request_wasab_marker_pickup_plan(
    marker_detection: dict[str, Any],
    flange_coords: list[float],
    marker_plane_z_base_mm: float,
    target_z_offset_mm: float,
    target_base_offset_mm: list[float],
    flange_orientation_deg: list[float],
) -> dict[str, Any]:
    """Convert an AprilTag center into a pickup Flange command."""
    request_id = str(uuid.uuid4())
    payload = {
        "request_id": request_id,
        "flange_coords": [float(value) for value in flange_coords],
        "marker_detection": marker_detection,
        "marker_plane_z_base_mm": float(marker_plane_z_base_mm),
        "target_z_offset_mm": float(target_z_offset_mm),
        "target_base_offset_mm": [float(value) for value in target_base_offset_mm],
        "flange_orientation_deg": [float(value) for value in flange_orientation_deg],
    }
    try:
        response = _SESSION.post(
            _marker_pickup_plan_url(),
            json=payload,
            timeout=(config.CONNECT_TIMEOUT_SEC, config.REQUEST_TIMEOUT_SEC),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc
    if not response.ok:
        raise WaSaBServiceError(
            f"Laptop marker-pickup HTTP {response.status_code}: {response.text[:800]}"
        )
    try:
        result: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError("Laptop marker-pickup response is not valid JSON") from exc
    if result.get("request_id") not in (None, request_id):
        raise WaSaBServiceError("Laptop marker-pickup request_id mismatch")
    return result


def request_wasab_operation_plan(
    frame,
    flange_coords: list[float],
    target_label: str | None = None,
) -> dict[str, Any]:
    """Send a frame and current Flange pose to the laptop without SSH tunneling."""
    if not isinstance(flange_coords, list) or len(flange_coords) != 6:
        raise WaSaBServiceError("flange_coords must contain six values")

    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), config.JPEG_QUALITY],
    )
    if not ok:
        raise WaSaBServiceError("JPEG encoding failed")

    request_id = str(uuid.uuid4())
    files = {"image": ("frame.jpg", encoded.tobytes(), "image/jpeg")}
    data = {
        "robot_state": json.dumps(
            {
                "request_id": request_id,
                "arm_id": config.ARM_ID,
                "flange_coords": [float(value) for value in flange_coords],
            }
        )
    }
    if target_label:
        data["target_label"] = target_label

    response = None
    for attempt in range(config.PLAN_CONNECT_RETRY_COUNT + 1):
        try:
            response = _SESSION.post(
                config.GRASP_SERVER_URL,
                files=files,
                data=data,
                timeout=(config.CONNECT_TIMEOUT_SEC, config.REQUEST_TIMEOUT_SEC),
            )
            break
        except (
            requests.exceptions.ConnectTimeout,
            requests.exceptions.ConnectionError,
        ) as exc:
            if attempt >= config.PLAN_CONNECT_RETRY_COUNT:
                raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc
            print(
                "[NETWORK] grasp-plan connection failed; retrying:",
                f"{attempt + 1}/{config.PLAN_CONNECT_RETRY_COUNT}",
                f"in {config.PLAN_CONNECT_RETRY_INTERVAL_SEC:.1f}s",
            )
            time.sleep(config.PLAN_CONNECT_RETRY_INTERVAL_SEC)
        except requests.RequestException as exc:
            # A read timeout may mean that the server is already processing the
            # request, so only pre-request connection failures are retried.
            raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc

    if response is None:
        raise WaSaBServiceError("Laptop server did not return a grasp-plan response")

    if not response.ok:
        raise WaSaBServiceError(
            f"Laptop server HTTP {response.status_code}: {response.text[:800]}"
        )
    try:
        payload: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError(
            "Laptop server response is not valid JSON"
        ) from exc

    if payload.get("request_id") not in (None, request_id):
        raise WaSaBServiceError("Laptop server request_id mismatch")
    return payload


def get_wasab_handover_zone() -> tuple[float, float, float, float]:
    """Load the persisted normalized two-palm interaction zone from the laptop."""
    parsed = urlparse(config.GRASP_SERVER_URL)
    url = urlunparse((parsed.scheme, parsed.netloc, "/settings/handover-zone", "", "", ""))
    try:
        response = _SESSION.get(
            url,
            timeout=(config.CONNECT_TIMEOUT_SEC, config.HEALTH_TIMEOUT_SEC),
        )
        response.raise_for_status()
        payload = response.json()
        zone = tuple(
            float(payload[name])
            for name in ("x_min_norm", "x_max_norm", "y_min_norm", "y_max_norm")
        )
    except (requests.RequestException, ValueError, KeyError, TypeError) as exc:
        raise WaSaBServiceError(f"Cannot load handover zone: {exc}") from exc
    x_min, x_max, y_min, y_max = zone
    if not (0.0 <= x_min < x_max <= 1.0 and 0.0 <= y_min < y_max <= 1.0):
        raise WaSaBServiceError(f"Invalid handover zone from laptop: {zone}")
    return zone


def request_wasab_latest_preview_grasp_plan(flange_coords: list[float]) -> dict[str, Any]:
    """Ask the laptop to plan from its latest camera preview frame."""
    if not isinstance(flange_coords, list) or len(flange_coords) != 6:
        raise WaSaBServiceError("flange_coords must contain six values")

    request_id = str(uuid.uuid4())
    payload = {
        "request_id": request_id,
        "flange_coords": [float(value) for value in flange_coords],
    }

    try:
        response = _SESSION.post(
            _latest_frame_grasp_plan_url(),
            json=payload,
            timeout=(config.CONNECT_TIMEOUT_SEC, config.REQUEST_TIMEOUT_SEC),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc

    if not response.ok:
        raise WaSaBServiceError(
            f"Laptop latest-frame grasp-plan HTTP {response.status_code}: {response.text[:800]}"
        )
    try:
        result: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError("Laptop latest-frame grasp-plan response is not valid JSON") from exc

    if result.get("request_id") not in (None, request_id):
        raise WaSaBServiceError("Laptop latest-frame grasp-plan request_id mismatch")
    return result


def request_wasab_object_detection(
    frame,
    target_label: str,
) -> dict[str, Any]:
    """Run YOLO detection only, without grasp-plan edge/safety rejection."""
    ok, encoded = cv2.imencode(
        ".jpg",
        frame,
        [int(cv2.IMWRITE_JPEG_QUALITY), config.JPEG_QUALITY],
    )
    if not ok:
        raise WaSaBServiceError("Detection frame JPEG encoding failed")
    try:
        response = _SESSION.post(
            _detect_url(),
            files={
                "image": (
                    "detect-frame.jpg",
                    encoded.tobytes(),
                    "image/jpeg",
                )
            },
            params={"target_label": str(target_label)},
            timeout=(
                config.CONNECT_TIMEOUT_SEC,
                config.REQUEST_TIMEOUT_SEC,
            ),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc
    if not response.ok:
        raise WaSaBServiceError(
            f"Laptop detect HTTP {response.status_code}: "
            f"{response.text[:800]}"
        )
    try:
        result: dict[str, Any] = response.json()
    except ValueError as exc:
        raise WaSaBServiceError(
            "Laptop detect response is not valid JSON"
        ) from exc
    detections = result.get("detections")
    if not isinstance(detections, list) or not detections:
        return {
            "status": "not_found",
            "message": "Target object was not detected.",
            **result,
        }
    selected = max(
        (item for item in detections if isinstance(item, dict)),
        key=lambda item: float(item.get("confidence", 0.0) or 0.0),
        default=None,
    )
    if selected is None:
        return {
            "status": "not_found",
            "message": "Target object was not detected.",
            **result,
        }
    return {
        **result,
        "status": "ok",
        "detection": {
            **selected,
            "midpoint_uv": selected.get("center"),
        },
    }
