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


def _streamer_frame_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/camera-frame", "", "", ""))


def _wasab_arm_command_stream_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/robot-command/stream", "", "", ""))


def _marker_place_plan_url() -> str:
    parsed = _parsed_wasab_service_url()
    return urlunparse((parsed.scheme, parsed.netloc, "/v1/marker-place-plan", "", "", ""))


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
                        "find marker": "find-marker",
                        "find_marker": "find-marker",
                        "marker_search": "marker-search",
                        "marker search": "marker-search",
                        "emergency_stop": "emergency-stop",
                        "emergency stop": "emergency-stop",
                    }
                    command = command_aliases.get(command, command)
                    valid_commands = {
                        "g", "p", "q", "r", "s", "k", "f", "m", "a", "t", "w", "x",
                        "pick", "pose", "gripper", "random", "servo-release",
                        "servo-focus", "place", "move", "find-marker", "marker",
                        "marker-search", "throw", "home", "stop", "halt",
                        "emergency-stop", "exit",
                    }
                    if command not in valid_commands:
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


def request_wasab_operation_plan(frame, flange_coords: list[float]) -> dict[str, Any]:
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
                "flange_coords": [float(value) for value in flange_coords],
            }
        )
    }

    try:
        response = _SESSION.post(
            config.GRASP_SERVER_URL,
            files=files,
            data=data,
            timeout=(config.CONNECT_TIMEOUT_SEC, config.REQUEST_TIMEOUT_SEC),
        )
    except requests.RequestException as exc:
        raise WaSaBServiceError(_wasab_connection_hint(exc)) from exc

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
