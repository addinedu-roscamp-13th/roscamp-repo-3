"""Raspberry Pi-side configuration for the laptop-local grasp service.

Only ``config/client_config.ini`` is intended to be edited by the operator.
The YOLO weight, camera intrinsic file, and Hand-Eye result stay on the laptop.
"""
from __future__ import annotations

import configparser
from pathlib import Path


RASPBERRY_PI_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = RASPBERRY_PI_ROOT / "config" / "client_config.ini"


def _numbers(value: str, expected_count: int, name: str) -> list[float]:
    try:
        values = [float(item.strip()) for item in value.split(",")]
    except ValueError as exc:
        raise ValueError(
            f"{name} must be {expected_count} comma-separated numbers"
        ) from exc
    if len(values) != expected_count:
        raise ValueError(f"{name} must contain exactly {expected_count} values")
    return values


def _load() -> configparser.ConfigParser:
    parser = configparser.ConfigParser(interpolation=None)
    if not DEFAULT_CONFIG_PATH.exists():
        raise FileNotFoundError(
            f"Raspberry Pi config not found: {DEFAULT_CONFIG_PATH}"
        )
    parser.read(DEFAULT_CONFIG_PATH, encoding="utf-8")
    return parser


_parser = _load()

# Laptop network
GRASP_SERVER_URL = _parser.get("network", "grasp_server_url").strip()
EXPECTED_SERVER_RUNTIME = _parser.get(
    "network", "expected_server_runtime", fallback="laptop-local"
).strip()
ALLOW_LOOPBACK_SERVER = _parser.getboolean(
    "network", "allow_loopback_server", fallback=False
)
REQUEST_TIMEOUT_SEC = _parser.getfloat("network", "request_timeout_sec")
CONNECT_TIMEOUT_SEC = _parser.getfloat(
    "network", "connect_timeout_sec", fallback=3.0
)
HEALTH_TIMEOUT_SEC = _parser.getfloat(
    "network", "health_timeout_sec", fallback=5.0
)
CHECK_SERVER_ON_STARTUP = _parser.getboolean(
    "network", "check_server_on_startup", fallback=True
)
JPEG_QUALITY = _parser.getint("network", "jpeg_quality")
if not 1 <= JPEG_QUALITY <= 100:
    raise ValueError("jpeg_quality must be in the range 1..100")

# Optional camera preview stream to the laptop server web page.
if _parser.has_section("camera_stream"):
    CAMERA_STREAM_ENABLED = _parser.getboolean("camera_stream", "enabled", fallback=True)
    CAMERA_STREAM_FPS = _parser.getfloat("camera_stream", "fps", fallback=2.0)
    CAMERA_STREAM_TIMEOUT_SEC = _parser.getfloat("camera_stream", "timeout_sec", fallback=0.4)
    CAMERA_STREAM_JPEG_QUALITY = _parser.getint("camera_stream", "jpeg_quality", fallback=70)
else:
    CAMERA_STREAM_ENABLED = True
    CAMERA_STREAM_FPS = 2.0
    CAMERA_STREAM_TIMEOUT_SEC = 0.4
    CAMERA_STREAM_JPEG_QUALITY = 70
if CAMERA_STREAM_FPS <= 0:
    raise ValueError("camera_stream fps must be positive")
if CAMERA_STREAM_TIMEOUT_SEC <= 0:
    raise ValueError("camera_stream timeout_sec must be positive")
if not 1 <= CAMERA_STREAM_JPEG_QUALITY <= 100:
    raise ValueError("camera_stream jpeg_quality must be in the range 1..100")
CAMERA_STREAM_INTERVAL_SEC = 1.0 / CAMERA_STREAM_FPS

# WaSaB architecture Streamer transport. UDP is used for lightweight camera preview;
# HTTP /camera-frame remains available as a fallback.
if _parser.has_section("udp_stream"):
    UDP_STREAM_ENABLED = _parser.getboolean("udp_stream", "enabled", fallback=True)
    UDP_STREAM_HOST = _parser.get("udp_stream", "host", fallback="").strip()
    UDP_STREAM_PORT = _parser.getint("udp_stream", "port", fallback=8001)
    UDP_STREAM_MAX_DATAGRAM_BYTES = _parser.getint("udp_stream", "max_datagram_bytes", fallback=1400)
    UDP_STREAM_FALLBACK_HTTP = _parser.getboolean("udp_stream", "fallback_http", fallback=True)
else:
    UDP_STREAM_ENABLED = True
    UDP_STREAM_HOST = ""
    UDP_STREAM_PORT = 8001
    UDP_STREAM_MAX_DATAGRAM_BYTES = 1400
    UDP_STREAM_FALLBACK_HTTP = True
if not 1 <= UDP_STREAM_PORT <= 65535:
    raise ValueError("udp_stream port must be in the range 1..65535")
if UDP_STREAM_MAX_DATAGRAM_BYTES < 512:
    raise ValueError("udp_stream max_datagram_bytes must be >= 512")

if _parser.has_section("remote_control"):
    REMOTE_COMMAND_ENABLED = _parser.getboolean("remote_control", "enabled", fallback=True)
    REMOTE_COMMAND_TIMEOUT_SEC = _parser.getfloat("remote_control", "timeout_sec", fallback=0.4)
else:
    REMOTE_COMMAND_ENABLED = True
    REMOTE_COMMAND_TIMEOUT_SEC = 0.4
if REMOTE_COMMAND_TIMEOUT_SEC <= 0:
    raise ValueError("remote_control timeout_sec must be positive")

# Camera: must match the laptop calibration image size.
CAMERA_ID_RAW = _parser.get("camera", "camera_id").strip()
CAMERA_ID = int(CAMERA_ID_RAW) if CAMERA_ID_RAW.isdecimal() else CAMERA_ID_RAW
CAMERA_FRAME_WIDTH = _parser.getint("camera", "frame_width")
CAMERA_FRAME_HEIGHT = _parser.getint("camera", "frame_height")
CAMERA_FLUSH_FRAMES = _parser.getint(
    "camera", "flush_frames_before_capture", fallback=0
)
if CAMERA_FRAME_WIDTH <= 0 or CAMERA_FRAME_HEIGHT <= 0:
    raise ValueError("camera frame_width and frame_height must be positive")
if CAMERA_FLUSH_FRAMES < 0:
    raise ValueError("flush_frames_before_capture must be >= 0")

# MyCobot motion
PORT = _parser.get("robot", "mycobot_port").strip()
BAUD = _parser.getint("robot", "mycobot_baud")
MOVE_SPEED = _parser.getint("robot", "move_speed")
MOVE_MODE = _parser.getint("robot", "move_mode")
POSE_POSITION_TOL_MM = _parser.getfloat("robot", "pose_position_tol_mm")
POSE_ANGLE_TOL_DEG = _parser.getfloat("robot", "pose_angle_tol_deg")
MOVE_TIMEOUT_SEC = _parser.getfloat("robot", "move_timeout_sec")
MOVE_POLL_SEC = _parser.getfloat("robot", "move_poll_sec")
HOME_FLANGE_COORDS = _numbers(
    _parser.get("robot", "home_flange_coords"), 6, "home_flange_coords"
)

# Gripper
GRIPPER_OPEN_VALUE = _parser.getint("gripper", "open_value")
GRIPPER_CLOSE_VALUE = _parser.getint("gripper", "close_value")
GRIPPER_SPEED = _parser.getint("gripper", "speed")
GRIPPER_SETTLE_SEC = _parser.getfloat("gripper", "settle_sec")

# Final safety gate: evaluated locally on the Raspberry Pi.
DRY_RUN = _parser.getboolean("safety", "dry_run")
SAFE_X_MM = (
    _parser.getfloat("safety", "safe_x_min_mm"),
    _parser.getfloat("safety", "safe_x_max_mm"),
)
SAFE_Y_MM = (
    _parser.getfloat("safety", "safe_y_min_mm"),
    _parser.getfloat("safety", "safe_y_max_mm"),
)
SAFE_Z_MM = (
    _parser.getfloat("safety", "safe_z_min_mm"),
    _parser.getfloat("safety", "safe_z_max_mm"),
)
SAFE_EULER_ABS_DEG = _parser.getfloat("safety", "safe_euler_abs_deg")

# Conservative random motion around the configured home flange pose.
if _parser.has_section("random_motion"):
    RANDOM_MOTION_ENABLED = _parser.getboolean("random_motion", "enabled", fallback=True)
    RANDOM_X_RADIUS_MM = _parser.getfloat("random_motion", "x_radius_mm", fallback=35.0)
    RANDOM_Y_RADIUS_MM = _parser.getfloat("random_motion", "y_radius_mm", fallback=35.0)
    RANDOM_Z_RADIUS_MM = _parser.getfloat("random_motion", "z_radius_mm", fallback=20.0)
else:
    RANDOM_MOTION_ENABLED = True
    RANDOM_X_RADIUS_MM = 35.0
    RANDOM_Y_RADIUS_MM = 35.0
    RANDOM_Z_RADIUS_MM = 20.0
if min(RANDOM_X_RADIUS_MM, RANDOM_Y_RADIUS_MM, RANDOM_Z_RADIUS_MM) < 0:
    raise ValueError("random_motion radii must be >= 0")

# Manual configured motion. M command moves to this Flange pose after local safety validation.
if _parser.has_section("manual_motion"):
    MANUAL_MOTION_ENABLED = _parser.getboolean("manual_motion", "enabled", fallback=True)
    MANUAL_FLANGE_COORDS = _numbers(
        _parser.get("manual_motion", "target_flange_coords"),
        6,
        "manual_motion.target_flange_coords",
    )
else:
    MANUAL_MOTION_ENABLED = False
    MANUAL_FLANGE_COORDS = HOME_FLANGE_COORDS

# Place motion. F command moves home first, then to this Flange pose and opens the gripper.
if _parser.has_section("place_motion"):
    PLACE_MOTION_ENABLED = _parser.getboolean("place_motion", "enabled", fallback=True)
    PLACE_FLANGE_COORDS = _numbers(
        _parser.get(
            "place_motion",
            "target_flange_coords",
            fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS),
        ),
        6,
        "place_motion.target_flange_coords",
    )
else:
    PLACE_MOTION_ENABLED = True
    PLACE_FLANGE_COORDS = HOME_FLANGE_COORDS

# Conducting motion. Patterns use reference poses and can be scaled gradually.
if _parser.has_section("conducting"):
    CONDUCTING_ENABLED = _parser.getboolean("conducting", "enabled", fallback=True)
    CONDUCTING_CONTROL_MODE = _parser.get("conducting", "control_mode", fallback="joint_rhythm").strip().lower()
    CONDUCTING_DISTANCE_SCALE = _parser.getfloat("conducting", "distance_scale", fallback=0.5)
    CONDUCTING_CYCLES = _parser.getint("conducting", "cycles", fallback=2)
    CONDUCTING_MOVE_SPEED = _parser.getint("conducting", "move_speed", fallback=45)
    CONDUCTING_CONTINUOUS = _parser.getboolean("conducting", "continuous", fallback=True)
    CONDUCTING_INTERPOLATION_STEPS = _parser.getint("conducting", "interpolation_steps", fallback=8)
    CONDUCTING_COMMAND_INTERVAL_SEC = _parser.getfloat("conducting", "command_interval_sec", fallback=0.18)
    CONDUCTING_RATE_HZ = _parser.getfloat("conducting", "rate_hz", fallback=12.0)
    CONDUCTING_BEAT_SEC = _parser.getfloat("conducting", "beat_sec", fallback=0.55)
    CONDUCTING_RETURN_SEC = _parser.getfloat("conducting", "return_sec", fallback=0.8)
    CONDUCTING_SERVO_GAIN = _parser.getfloat("conducting", "servo_gain", fallback=0.45)
    CONDUCTING_YAW_AMPLITUDE_DEG = _parser.getfloat("conducting", "yaw_amplitude_deg", fallback=10.0)
    CONDUCTING_PITCH_AMPLITUDE_DEG = _parser.getfloat("conducting", "pitch_amplitude_deg", fallback=9.0)
    CONDUCTING_WRIST_AMPLITUDE_DEG = _parser.getfloat("conducting", "wrist_amplitude_deg", fallback=4.0)
    CONDUCTING_START_FLANGE_COORDS = _numbers(
        _parser.get("conducting", "start_flange_coords", fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS)),
        6,
        "conducting.start_flange_coords",
    )
    CONDUCTING_DOWN_FLANGE_COORDS = _numbers(
        _parser.get("conducting", "down_flange_coords", fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS)),
        6,
        "conducting.down_flange_coords",
    )
    CONDUCTING_UP_FLANGE_COORDS = _numbers(
        _parser.get("conducting", "up_flange_coords", fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS)),
        6,
        "conducting.up_flange_coords",
    )
    CONDUCTING_LEFT_FLANGE_COORDS = _numbers(
        _parser.get("conducting", "left_flange_coords", fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS)),
        6,
        "conducting.left_flange_coords",
    )
    CONDUCTING_RIGHT_FLANGE_COORDS = _numbers(
        _parser.get("conducting", "right_flange_coords", fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS)),
        6,
        "conducting.right_flange_coords",
    )
else:
    CONDUCTING_ENABLED = True
    CONDUCTING_CONTROL_MODE = "joint_rhythm"
    CONDUCTING_DISTANCE_SCALE = 0.5
    CONDUCTING_CYCLES = 2
    CONDUCTING_MOVE_SPEED = 45
    CONDUCTING_CONTINUOUS = True
    CONDUCTING_INTERPOLATION_STEPS = 8
    CONDUCTING_COMMAND_INTERVAL_SEC = 0.18
    CONDUCTING_RATE_HZ = 12.0
    CONDUCTING_BEAT_SEC = 0.55
    CONDUCTING_RETURN_SEC = 0.8
    CONDUCTING_SERVO_GAIN = 0.45
    CONDUCTING_YAW_AMPLITUDE_DEG = 10.0
    CONDUCTING_PITCH_AMPLITUDE_DEG = 9.0
    CONDUCTING_WRIST_AMPLITUDE_DEG = 4.0
    CONDUCTING_START_FLANGE_COORDS = HOME_FLANGE_COORDS
    CONDUCTING_DOWN_FLANGE_COORDS = HOME_FLANGE_COORDS
    CONDUCTING_UP_FLANGE_COORDS = HOME_FLANGE_COORDS
    CONDUCTING_LEFT_FLANGE_COORDS = HOME_FLANGE_COORDS
    CONDUCTING_RIGHT_FLANGE_COORDS = HOME_FLANGE_COORDS
if CONDUCTING_CONTROL_MODE not in {"joint_rhythm", "flange"}:
    raise ValueError("conducting control_mode must be joint_rhythm or flange")
if CONDUCTING_DISTANCE_SCALE < 0:
    raise ValueError("conducting distance_scale must be >= 0")
if CONDUCTING_CYCLES <= 0:
    raise ValueError("conducting cycles must be positive")
if CONDUCTING_MOVE_SPEED <= 0:
    raise ValueError("conducting move_speed must be positive")
if CONDUCTING_INTERPOLATION_STEPS <= 0:
    raise ValueError("conducting interpolation_steps must be positive")
if CONDUCTING_COMMAND_INTERVAL_SEC <= 0:
    raise ValueError("conducting command_interval_sec must be positive")
if CONDUCTING_RATE_HZ <= 0:
    raise ValueError("conducting rate_hz must be positive")
if CONDUCTING_BEAT_SEC <= 0:
    raise ValueError("conducting beat_sec must be positive")
if CONDUCTING_RETURN_SEC <= 0:
    raise ValueError("conducting return_sec must be positive")
if not 0 < CONDUCTING_SERVO_GAIN <= 1:
    raise ValueError("conducting servo_gain must be in (0, 1]")

# April marker search. A command scans the configured pan joint until an
# AprilTag/Aruco marker is visible in the camera frame.
if _parser.has_section("marker_search"):
    MARKER_SEARCH_ENABLED = _parser.getboolean("marker_search", "enabled", fallback=True)
    MARKER_SEARCH_DICTIONARY = _parser.get(
        "marker_search",
        "dictionary",
        fallback="DICT_APRILTAG_36h11",
    ).strip()
    MARKER_SEARCH_TARGET_IDS_RAW = _parser.get(
        "marker_search",
        "target_ids",
        fallback="",
    ).strip()
    MARKER_SEARCH_PAN_JOINT = _parser.getint("marker_search", "pan_joint", fallback=1)
    MARKER_SEARCH_PAN_RANGE_DEG = _parser.getfloat("marker_search", "pan_range_deg", fallback=50.0)
    MARKER_SEARCH_STEP_DEG = _parser.getfloat("marker_search", "search_step_deg", fallback=3.0)
    MARKER_SEARCH_SPEED = _parser.getint("marker_search", "speed", fallback=25)
    MARKER_SEARCH_HZ = _parser.getfloat("marker_search", "hz", fallback=8.0)
    MARKER_SEARCH_MAX_DURATION_SEC = _parser.getfloat("marker_search", "max_duration_sec", fallback=0.0)
else:
    MARKER_SEARCH_ENABLED = True
    MARKER_SEARCH_DICTIONARY = "DICT_APRILTAG_36h11"
    MARKER_SEARCH_TARGET_IDS_RAW = ""
    MARKER_SEARCH_PAN_JOINT = 1
    MARKER_SEARCH_PAN_RANGE_DEG = 50.0
    MARKER_SEARCH_STEP_DEG = 3.0
    MARKER_SEARCH_SPEED = 25
    MARKER_SEARCH_HZ = 8.0
    MARKER_SEARCH_MAX_DURATION_SEC = 0.0

if MARKER_SEARCH_TARGET_IDS_RAW:
    try:
        MARKER_SEARCH_TARGET_IDS = {
            int(item.strip())
            for item in MARKER_SEARCH_TARGET_IDS_RAW.split(",")
            if item.strip()
        }
    except ValueError as exc:
        raise ValueError("marker_search target_ids must be comma-separated integers") from exc
else:
    MARKER_SEARCH_TARGET_IDS = set()
if not 1 <= MARKER_SEARCH_PAN_JOINT <= 6:
    raise ValueError("marker_search pan_joint must be in the range 1..6")
if MARKER_SEARCH_PAN_RANGE_DEG <= 0:
    raise ValueError("marker_search pan_range_deg must be positive")
if MARKER_SEARCH_STEP_DEG <= 0:
    raise ValueError("marker_search search_step_deg must be positive")
if MARKER_SEARCH_SPEED <= 0:
    raise ValueError("marker_search speed must be positive")
if MARKER_SEARCH_HZ <= 0:
    raise ValueError("marker_search hz must be positive")
if MARKER_SEARCH_MAX_DURATION_SEC < 0:
    raise ValueError("marker_search max_duration_sec must be >= 0")

# UI
SHOW_WINDOW = _parser.getboolean("ui", "show_window")
WINDOW_NAME = _parser.get("ui", "window_name")

# -----------------------------
# Throw mode (unchanged robot-side behavior)
# -----------------------------
THROW_START_ANGLES = [43.68, 66.62, 3.79, -38.47, 7.16, 47.90]
THROW_END_ANGLES = [47.94, -29.44, -31.03, 40.19, 4.48, 47.90]
THROW_PREP_SPEED = 30
THROW_SPEED = 100
THROW_GRIPPER_OPEN_DELAY_SEC = 0.3
THROW_ANGLE_TOLERANCE_DEG = 2.0
THROW_PREP_TIMEOUT_SEC = 15.0
THROW_END_TIMEOUT_SEC = 10.0
THROW_FINAL_TIMEOUT_SEC = 20.0
THROW_FINAL_FLANGE_COORDS = [147.4, 52.6, 241.7, -177.68, 5.26, -94.11]
THROW_FINAL_MOVE_SPEED = 50
THROW_FINAL_MOVE_MODE = 0
