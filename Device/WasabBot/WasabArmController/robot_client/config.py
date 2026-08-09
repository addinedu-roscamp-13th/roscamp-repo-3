"""Raspberry Pi-side configuration for the laptop-local grasp service.

Only ``config/client_config.ini`` is intended to be edited by the operator.
The YOLO weight, camera intrinsic file, and Hand-Eye result stay on the laptop.
"""
from __future__ import annotations

import configparser
import os
from pathlib import Path


RASPBERRY_PI_ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = RASPBERRY_PI_ROOT / "config"
DEFAULT_CONFIG_PATH = CONFIG_DIR / "client_config.ini"
ARM_IDENTITY_PATH = CONFIG_DIR / "arm_identity"


def _read_device_arm_id() -> str:
    env_arm_id = os.environ.get("WASAB_ARM_ID", "").strip().lower()
    if env_arm_id:
        return env_arm_id
    if ARM_IDENTITY_PATH.exists():
        return ARM_IDENTITY_PATH.read_text(encoding="utf-8").strip().lower()
    return ""


_CONFIG_ARM_ID = _read_device_arm_id()
if _CONFIG_ARM_ID and _CONFIG_ARM_ID not in {"left", "right"}:
    raise ValueError("config/arm_identity or WASAB_ARM_ID must be left or right")


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


def _number_list(value: str, name: str) -> list[float]:
    try:
        values = [float(item.strip()) for item in value.split(",") if item.strip()]
    except ValueError as exc:
        raise ValueError(f"{name} must be comma-separated numbers") from exc
    if not values:
        raise ValueError(f"{name} must contain at least one value")
    return values


def _load() -> configparser.ConfigParser:
    parser = configparser.ConfigParser(interpolation=None)
    if not DEFAULT_CONFIG_PATH.exists():
        raise FileNotFoundError(
            f"Raspberry Pi config not found: {DEFAULT_CONFIG_PATH}"
        )
    parser.read(DEFAULT_CONFIG_PATH, encoding="utf-8")
    selected_arm = _CONFIG_ARM_ID or parser.get(
        "arm_identity", "arm_id", fallback=""
    ).strip().lower()
    if selected_arm not in {"left", "right"}:
        raise ValueError(
            "Robot arm identity is missing. Write left or right to "
            f"{ARM_IDENTITY_PATH}"
        )

    prefix = f"{selected_arm}."
    for section_name in list(parser.sections()):
        if not section_name.startswith(prefix):
            continue
        base_section = section_name[len(prefix):]
        if not parser.has_section(base_section):
            parser.add_section(base_section)
        for option, value in parser.items(section_name, raw=True):
            parser.set(base_section, option, value)

    if not parser.has_section("arm_identity"):
        parser.add_section("arm_identity")
    parser.set("arm_identity", "arm_id", selected_arm)
    parser.set("arm_identity", "role", selected_arm)
    return parser


_parser = _load()

ARM_ID = _parser.get("arm_identity", "arm_id", fallback="left").strip().lower()
ARM_ROLE = _parser.get("arm_identity", "role", fallback=ARM_ID).strip().lower()
DUAL_ARM_ENABLED = _parser.getboolean("arm_identity", "dual_arm_enabled", fallback=False)
ARM_CONFIGURED = _parser.getboolean("arm_identity", "configured", fallback=ARM_ID == "left")
ARM_SETUP_MODE = _parser.getboolean("arm_identity", "setup_mode", fallback=False)
if ARM_ID not in {"left", "right"}:
    raise ValueError("arm_identity.arm_id must be left or right")
if ARM_ROLE not in {"left", "right"}:
    raise ValueError("arm_identity.role must be left or right")
if ARM_ID == "right" and not ARM_CONFIGURED and not ARM_SETUP_MODE:
    raise RuntimeError(
        "Right Arm is not configured. Measure its home/workspace poses, complete its own "
        "camera + hand-eye calibration, then set arm_identity.configured=true."
    )

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
PLAN_CONNECT_RETRY_COUNT = _parser.getint(
    "network", "plan_connect_retry_count", fallback=2
)
PLAN_CONNECT_RETRY_INTERVAL_SEC = _parser.getfloat(
    "network", "plan_connect_retry_interval_sec", fallback=0.5
)
HEALTH_TIMEOUT_SEC = _parser.getfloat(
    "network", "health_timeout_sec", fallback=5.0
)
CHECK_SERVER_ON_STARTUP = _parser.getboolean(
    "network", "check_server_on_startup", fallback=True
)
JPEG_QUALITY = _parser.getint("network", "jpeg_quality")
if PLAN_CONNECT_RETRY_COUNT < 0:
    raise ValueError("network plan_connect_retry_count must be >= 0")
if PLAN_CONNECT_RETRY_INTERVAL_SEC < 0:
    raise ValueError("network plan_connect_retry_interval_sec must be >= 0")
if not 1 <= JPEG_QUALITY <= 100:
    raise ValueError("jpeg_quality must be in the range 1..100")

# Optional camera preview stream to the laptop server web page.
if _parser.has_section("camera_stream"):
    CAMERA_STREAM_ENABLED = _parser.getboolean("camera_stream", "enabled", fallback=True)
    CAMERA_WORKSPACE_OVERLAY_ENABLED = _parser.getboolean(
        "camera_stream", "workspace_overlay_enabled", fallback=False
    )
    CAMERA_STREAM_FPS = _parser.getfloat("camera_stream", "fps", fallback=2.0)
    CAMERA_STREAM_TIMEOUT_SEC = _parser.getfloat("camera_stream", "timeout_sec", fallback=0.4)
    CAMERA_STREAM_JPEG_QUALITY = _parser.getint("camera_stream", "jpeg_quality", fallback=70)
else:
    CAMERA_STREAM_ENABLED = True
    CAMERA_WORKSPACE_OVERLAY_ENABLED = False
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

HAND_GESTURE_ENABLED = _parser.getboolean("hand_gesture", "enabled", fallback=False)
HAND_GESTURE_START_ENABLED = _parser.getboolean(
    "hand_gesture", "start_enabled", fallback=False
) and HAND_GESTURE_ENABLED
HAND_GESTURE_STABLE_FRAMES = _parser.getint("hand_gesture", "stable_frames", fallback=20)
HAND_GESTURE_RELEASE_FRAMES = _parser.getint("hand_gesture", "release_frames", fallback=6)
HAND_GESTURE_COOLDOWN_SEC = _parser.getfloat("hand_gesture", "cooldown_sec", fallback=8.0)
HAND_GESTURE_PROCESS_FPS = _parser.getfloat("hand_gesture", "process_fps", fallback=8.0)
HAND_GESTURE_HOLD_SEC = _parser.getfloat("hand_gesture", "hold_sec", fallback=3.0)
HAND_GESTURE_MIN_PALM_SPAN_NORM = _parser.getfloat(
    "hand_gesture", "min_palm_span_norm", fallback=0.07
)
HAND_GESTURE_MAX_PALM_SPAN_NORM = _parser.getfloat(
    "hand_gesture", "max_palm_span_norm", fallback=0.30
)
HAND_GESTURE_EDGE_MARGIN_NORM = _parser.getfloat(
    "hand_gesture", "edge_margin_norm", fallback=0.06
)
HAND_GESTURE_MIN_PALM_V_PX = _parser.getfloat(
    "hand_gesture", "min_palm_v_px", fallback=0.0
)
_camera_frame_height = _parser.getint("camera", "frame_height")
HAND_GESTURE_MIN_PALM_V_NORM = HAND_GESTURE_MIN_PALM_V_PX / _camera_frame_height
PALM_REFERENCE_ENABLED = _parser.getboolean(
    "hand_gesture", "palm_reference_enabled", fallback=False
)
PALM_REFERENCE_PIXEL_UV = (
    _numbers(
        _parser.get("hand_gesture", "palm_reference_pixel_uv"),
        2,
        "hand_gesture.palm_reference_pixel_uv",
    )
    if PALM_REFERENCE_ENABLED
    else None
)
PALM_REFERENCE_FLANGE_COORDS = (
    _numbers(
        _parser.get("hand_gesture", "palm_reference_flange_coords"),
        6,
        "hand_gesture.palm_reference_flange_coords",
    )
    if PALM_REFERENCE_ENABLED
    else None
)
PALM_REFERENCE_JOINT_ANGLES = (
    _numbers(
        _parser.get("hand_gesture", "palm_reference_joint_angles"),
        6,
        "hand_gesture.palm_reference_joint_angles",
    )
    if PALM_REFERENCE_ENABLED
    else None
)
PALM_PIXEL_TO_BASE_XY_MM = _numbers(
    _parser.get(
        "hand_gesture",
        "palm_pixel_to_base_xy_mm",
        fallback="0.0, 0.0, 0.0, 0.0",
    ),
    4,
    "hand_gesture.palm_pixel_to_base_xy_mm",
)
PALM_MAX_XY_CORRECTION_MM = _parser.getfloat(
    "hand_gesture", "palm_max_xy_correction_mm", fallback=60.0
)
PALM_PLACE_APPROACH_JOINT_FRACTION = _parser.getfloat(
    "hand_gesture", "palm_place_approach_joint_fraction", fallback=0.75
)
PALM_HITBOX_CALIBRATION_ENABLED = _parser.getboolean(
    "hand_gesture", "hitbox_calibration_enabled", fallback=False
)
PALM_HITBOX_CALIBRATION_TARGET_SAMPLES = _parser.getint(
    "hand_gesture", "hitbox_calibration_target_samples", fallback=100
)
PALM_HITBOX_CALIBRATION_OUTPUT_DIR = (
    RASPBERRY_PI_ROOT
    / _parser.get(
        "hand_gesture",
        "hitbox_calibration_output_dir",
        fallback="gesture_calibration",
    ).strip()
)
PALM_REFERENCE_Z_MM = (
    float(PALM_REFERENCE_FLANGE_COORDS[2])
    if PALM_REFERENCE_FLANGE_COORDS is not None
    else None
)
HAND_GESTURE_MIN_DETECTION_CONFIDENCE = _parser.getfloat(
    "hand_gesture", "min_detection_confidence", fallback=0.7
)
HAND_GESTURE_MIN_TRACKING_CONFIDENCE = _parser.getfloat(
    "hand_gesture", "min_tracking_confidence", fallback=0.7
)
HAND_GESTURE_AUTO_PLACE = _parser.getboolean("hand_gesture", "auto_place", fallback=True)
PALM_RELEASE_CONFIRMATION_ENABLED = _parser.getboolean(
    "hand_gesture", "release_confirmation_enabled", fallback=False
)
PALM_RELEASE_CONFIRMATION_MODE = _parser.get(
    "hand_gesture", "release_confirmation_mode", fallback="open_palm"
).strip().lower()
PALM_RELEASE_CONFIRMATION_TIMEOUT_SEC = _parser.getfloat(
    "hand_gesture", "release_confirmation_timeout_sec", fallback=15.0
)
PALM_RELEASE_CONFIRMATION_HOLD_SEC = _parser.getfloat(
    "hand_gesture", "release_confirmation_hold_sec", fallback=1.0
)
if HAND_GESTURE_STABLE_FRAMES <= 0 or HAND_GESTURE_RELEASE_FRAMES <= 0:
    raise ValueError("hand_gesture stable_frames and release_frames must be positive")
if HAND_GESTURE_COOLDOWN_SEC < 0 or HAND_GESTURE_PROCESS_FPS <= 0:
    raise ValueError("hand_gesture cooldown_sec/process_fps are invalid")
if HAND_GESTURE_HOLD_SEC <= 0:
    raise ValueError("hand_gesture hold_sec must be positive")
if PALM_RELEASE_CONFIRMATION_TIMEOUT_SEC <= 0:
    raise ValueError("hand_gesture release_confirmation_timeout_sec must be positive")
if PALM_RELEASE_CONFIRMATION_MODE not in {"open_palm", "hand"}:
    raise ValueError(
        "hand_gesture release_confirmation_mode must be open_palm or hand"
    )
if PALM_RELEASE_CONFIRMATION_HOLD_SEC <= 0:
    raise ValueError("hand_gesture release_confirmation_hold_sec must be positive")
if PALM_RELEASE_CONFIRMATION_HOLD_SEC > PALM_RELEASE_CONFIRMATION_TIMEOUT_SEC:
    raise ValueError(
        "hand_gesture release confirmation hold must not exceed its timeout"
    )
if not (
    0.0 < HAND_GESTURE_MIN_PALM_SPAN_NORM
    < HAND_GESTURE_MAX_PALM_SPAN_NORM
    < 1.0
):
    raise ValueError("hand_gesture palm span limits must satisfy 0 < min < max < 1")
if not 0.0 <= HAND_GESTURE_EDGE_MARGIN_NORM < 0.5:
    raise ValueError("hand_gesture edge_margin_norm must be within 0..0.5")
if not 0.0 <= HAND_GESTURE_MIN_PALM_V_PX < _camera_frame_height:
    raise ValueError("hand_gesture min_palm_v_px is outside the camera frame")
if PALM_MAX_XY_CORRECTION_MM <= 0:
    raise ValueError("hand_gesture palm_max_xy_correction_mm must be positive")
if not 0.1 <= PALM_PLACE_APPROACH_JOINT_FRACTION <= 0.95:
    raise ValueError(
        "hand_gesture palm_place_approach_joint_fraction must be within 0.1..0.95"
    )
if PALM_HITBOX_CALIBRATION_TARGET_SAMPLES <= 0:
    raise ValueError(
        "hand_gesture hitbox_calibration_target_samples must be positive"
    )
if PALM_REFERENCE_PIXEL_UV is not None and not (
    0.0 <= PALM_REFERENCE_PIXEL_UV[0] < _parser.getint("camera", "frame_width")
    and 0.0 <= PALM_REFERENCE_PIXEL_UV[1] < _parser.getint("camera", "frame_height")
):
    raise ValueError("hand_gesture palm_reference_pixel_uv is outside the camera frame")
if not 0.0 <= HAND_GESTURE_MIN_DETECTION_CONFIDENCE <= 1.0:
    raise ValueError("hand_gesture min_detection_confidence must be in 0..1")
if not 0.0 <= HAND_GESTURE_MIN_TRACKING_CONFIDENCE <= 1.0:
    raise ValueError("hand_gesture min_tracking_confidence must be in 0..1")

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
HOME_SETTLE_SEC = _parser.getfloat("robot", "home_settle_sec", fallback=0.0)
HOME_FLANGE_COORDS = _numbers(
    _parser.get("robot", "home_flange_coords"), 6, "home_flange_coords"
)
HOME_JOINT_ANGLES = (
    _numbers(
        _parser.get("robot", "home_joint_angles"),
        6,
        "home_joint_angles",
    )
    if _parser.has_option("robot", "home_joint_angles")
    else None
)
if HOME_SETTLE_SEC < 0:
    raise ValueError("robot home_settle_sec must be >= 0")

RIGHT_PICK_REFERENCE_ENABLED = _parser.getboolean(
    "right_pick_reference", "enabled", fallback=False
)
RIGHT_PICK_MOTION_STRATEGY = _parser.get(
    "right_pick_reference", "motion_strategy", fallback="joint"
).strip().lower()
if RIGHT_PICK_MOTION_STRATEGY not in {"joint", "ik", "hybrid"}:
    raise ValueError(
        "right_pick_reference.motion_strategy must be joint, ik, or hybrid"
    )
RIGHT_PICK_REFERENCE_FLANGE_COORDS = (
    _numbers(
        _parser.get("right_pick_reference", "flange_coords"),
        6,
        "right_pick_reference.flange_coords",
    )
    if RIGHT_PICK_REFERENCE_ENABLED
    else None
)
RIGHT_PICK_REFERENCE_JOINT_ANGLES = (
    _numbers(
        _parser.get("right_pick_reference", "joint_angles"),
        6,
        "right_pick_reference.joint_angles",
    )
    if RIGHT_PICK_REFERENCE_ENABLED
    else None
)
RIGHT_PICK_APPROACH_FLANGE_COORDS = (
    _numbers(
        _parser.get("right_pick_reference", "approach_flange_coords"),
        6,
        "right_pick_reference.approach_flange_coords",
    )
    if RIGHT_PICK_REFERENCE_ENABLED
    and _parser.has_option("right_pick_reference", "approach_flange_coords")
    else None
)
RIGHT_PICK_APPROACH_JOINT_ANGLES = (
    _numbers(
        _parser.get("right_pick_reference", "approach_joint_angles"),
        6,
        "right_pick_reference.approach_joint_angles",
    )
    if RIGHT_PICK_REFERENCE_ENABLED
    and _parser.has_option("right_pick_reference", "approach_joint_angles")
    else None
)
RIGHT_PICK_VISUAL_JOINT_CORRECTION_ENABLED = _parser.getboolean(
    "right_pick_reference",
    "visual_joint_correction_enabled",
    fallback=False,
)
RIGHT_PICK_VISUAL_REFERENCE_UV = _numbers(
    _parser.get(
        "right_pick_reference",
        "visual_reference_uv",
        fallback="0.0, 0.0",
    ),
    2,
    "right_pick_reference.visual_reference_uv",
)
RIGHT_PICK_VISUAL_J1_GAIN_DEG_PER_PX = _parser.getfloat(
    "right_pick_reference",
    "visual_j1_gain_deg_per_px",
    fallback=-0.04,
)
RIGHT_PICK_VISUAL_J2_GAIN_DEG_PER_PX = _parser.getfloat(
    "right_pick_reference",
    "visual_j2_gain_deg_per_px",
    fallback=0.025,
)
RIGHT_PICK_VISUAL_J5_GAIN_DEG_PER_PX = _parser.getfloat(
    "right_pick_reference",
    "visual_j5_gain_deg_per_px",
    fallback=0.04,
)
RIGHT_PICK_VISUAL_JOINT_CORRECTION_MAX_DEG = _parser.getfloat(
    "right_pick_reference",
    "visual_joint_correction_max_deg",
    fallback=3.0,
)
RIGHT_PICK_USE_REFERENCE_Z = (
    _parser.getboolean("right_pick_reference", "use_reference_z", fallback=True)
    if RIGHT_PICK_REFERENCE_ENABLED
    else False
)
RIGHT_PICK_USE_REFERENCE_XYZ = (
    _parser.getboolean("right_pick_reference", "use_reference_xyz", fallback=False)
    if RIGHT_PICK_REFERENCE_ENABLED
    else False
)
RIGHT_PICK_USE_REFERENCE_ORIENTATION = (
    _parser.getboolean(
        "right_pick_reference",
        "use_reference_orientation",
        fallback=True,
    )
    if RIGHT_PICK_REFERENCE_ENABLED
    else False
)
RIGHT_PICK_USE_JOINT_TARGET = (
    _parser.getboolean("right_pick_reference", "use_joint_target", fallback=False)
    if RIGHT_PICK_REFERENCE_ENABLED
    else False
)
RIGHT_PICK_APPROACH_JOINT_FRACTION = _parser.getfloat(
    "right_pick_reference", "approach_joint_fraction", fallback=0.75
)
if not 0.1 <= RIGHT_PICK_APPROACH_JOINT_FRACTION <= 0.95:
    raise ValueError(
        "right_pick_reference approach_joint_fraction must be within 0.1..0.95"
    )
RIGHT_PICK_JOINT_SPEED = (
    _parser.getint("right_pick_reference", "joint_speed", fallback=10)
    if RIGHT_PICK_REFERENCE_ENABLED
    else 10
)
if RIGHT_PICK_JOINT_SPEED <= 0:
    raise ValueError("right_pick_reference joint_speed must be positive")
RIGHT_PICK_IK_CORRECTION_MAX_XY_MM = _parser.getfloat(
    "right_pick_reference", "ik_correction_max_xy_mm", fallback=15.0
)
if RIGHT_PICK_IK_CORRECTION_MAX_XY_MM < 0:
    raise ValueError(
        "right_pick_reference ik_correction_max_xy_mm must be >= 0"
    )
RIGHT_PICK_IK_POSITION_TOL_MM = _parser.getfloat(
    "right_pick_reference", "ik_position_tol_mm", fallback=12.0
)
RIGHT_PICK_IK_ANGLE_TOL_DEG = _parser.getfloat(
    "right_pick_reference", "ik_angle_tol_deg", fallback=5.0
)
RIGHT_PICK_APPROACH_POSITION_TOL_MM = _parser.getfloat(
    "right_pick_reference",
    "approach_position_tol_mm",
    fallback=RIGHT_PICK_IK_POSITION_TOL_MM,
)
RIGHT_PICK_APPROACH_ANGLE_TOL_DEG = _parser.getfloat(
    "right_pick_reference",
    "approach_angle_tol_deg",
    fallback=RIGHT_PICK_IK_ANGLE_TOL_DEG,
)
RIGHT_PICK_APPROACH_SETTLE_SEC = _parser.getfloat(
    "right_pick_reference", "approach_settle_sec", fallback=0.0
)
RIGHT_PICK_FINAL_RETRY_COUNT = _parser.getint(
    "right_pick_reference", "final_retry_count", fallback=0
)
RIGHT_PICK_FINAL_ATTEMPT_TIMEOUT_SEC = _parser.getfloat(
    "right_pick_reference", "final_attempt_timeout_sec", fallback=MOVE_TIMEOUT_SEC
)
RIGHT_PICK_FINAL_JOINT_RETRY_TIMEOUT_SEC = _parser.getfloat(
    "right_pick_reference",
    "final_joint_retry_timeout_sec",
    fallback=MOVE_TIMEOUT_SEC,
)
RIGHT_PICK_FINAL_XY_ALIGN_LIFT_Z_MM = _parser.getfloat(
    "right_pick_reference", "final_xy_align_lift_z_mm", fallback=0.0
)
RIGHT_PICK_IK_MIN_MOTION_SEC = _parser.getfloat(
    "right_pick_reference", "ik_min_motion_sec", fallback=1.0
)
RIGHT_PICK_FINAL_DESCENT_Y_OFFSET_MM = _parser.getfloat(
    "right_pick_reference", "final_descent_y_offset_mm", fallback=0.0
)
if RIGHT_PICK_IK_POSITION_TOL_MM <= 0 or RIGHT_PICK_IK_ANGLE_TOL_DEG <= 0:
    raise ValueError(
        "right_pick_reference IK position/angle tolerances must be positive"
    )
if (
    RIGHT_PICK_APPROACH_POSITION_TOL_MM <= 0
    or RIGHT_PICK_APPROACH_ANGLE_TOL_DEG <= 0
):
    raise ValueError(
        "right_pick_reference approach position/angle tolerances must be positive"
    )
if RIGHT_PICK_APPROACH_SETTLE_SEC < 0:
    raise ValueError(
        "right_pick_reference approach_settle_sec must be >= 0"
    )
if RIGHT_PICK_FINAL_RETRY_COUNT < 0:
    raise ValueError(
        "right_pick_reference final_retry_count must be >= 0"
    )
if (
    RIGHT_PICK_FINAL_ATTEMPT_TIMEOUT_SEC <= 0
    or RIGHT_PICK_FINAL_JOINT_RETRY_TIMEOUT_SEC <= 0
):
    raise ValueError(
        "right_pick_reference final attempt/retry timeouts must be positive"
    )
if RIGHT_PICK_FINAL_XY_ALIGN_LIFT_Z_MM < 0:
    raise ValueError(
        "right_pick_reference final_xy_align_lift_z_mm must be >= 0"
    )
if RIGHT_PICK_VISUAL_JOINT_CORRECTION_MAX_DEG < 0:
    raise ValueError(
        "right_pick_reference visual_joint_correction_max_deg must be >= 0"
    )
if RIGHT_PICK_IK_MIN_MOTION_SEC < 0:
    raise ValueError("right_pick_reference ik_min_motion_sec must be >= 0")
if abs(RIGHT_PICK_FINAL_DESCENT_Y_OFFSET_MM) > 20.0:
    raise ValueError(
        "right_pick_reference final_descent_y_offset_mm must be within +/-20mm"
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

GESTURE_HOME_ENABLED = _parser.getboolean("hand_gesture", "home_enabled", fallback=False)
GESTURE_HOME_FLANGE_COORDS = _numbers(
    _parser.get(
        "hand_gesture",
        "home_flange_coords",
        fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS),
    ),
    6,
    "hand_gesture.home_flange_coords",
)
GESTURE_HOME_JOINT_ANGLES = (
    _numbers(
        _parser.get("hand_gesture", "home_joint_angles"),
        6,
        "hand_gesture.home_joint_angles",
    )
    if _parser.has_option("hand_gesture", "home_joint_angles")
    else None
)
GESTURE_HOME_SPEED = _parser.getint("hand_gesture", "home_speed", fallback=MOVE_SPEED)
if GESTURE_HOME_SPEED <= 0:
    raise ValueError("hand_gesture home_speed must be positive")
for axis, value, limits in zip(
    ("x", "y", "z"),
    GESTURE_HOME_FLANGE_COORDS[:3],
    (SAFE_X_MM, SAFE_Y_MM, SAFE_Z_MM),
):
    if not limits[0] <= value <= limits[1]:
        raise ValueError(f"hand_gesture home {axis}={value} is outside safety range {limits}")
if any(abs(value) > SAFE_EULER_ABS_DEG for value in GESTURE_HOME_FLANGE_COORDS[3:]):
    raise ValueError("hand_gesture home orientation exceeds safe_euler_abs_deg")

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

# Pick motion. Apply a small final Base-frame correction only when the laptop
# rotated the gripper for a long object.
if _parser.has_section("pick_motion"):
    PICK_AUTO_ROTATED_XY_CORRECTION_ENABLED = _parser.getboolean(
        "pick_motion",
        "auto_rotated_xy_correction_enabled",
        fallback=False,
    )
    PICK_AUTO_ROTATED_MIN_RZ_OFFSET_DEG = _parser.getfloat(
        "pick_motion",
        "auto_rotated_min_rz_offset_deg",
        fallback=45.0,
    )
    PICK_AUTO_ROTATED_BASE_XY_OFFSET_MM = _numbers(
        _parser.get(
            "pick_motion",
            "auto_rotated_base_xy_offset_mm",
            fallback="0.0, 0.0",
        ),
        2,
        "pick_motion.auto_rotated_base_xy_offset_mm",
    )
    PICK_PLAN_BASE_XY_OFFSET_MM = _numbers(
        _parser.get(
            "pick_motion",
            "plan_base_xy_offset_mm",
            fallback="0.0, 0.0",
        ),
        2,
        "pick_motion.plan_base_xy_offset_mm",
    )
    PICK_FIXED_REFERENCE_ENABLED = _parser.getboolean(
        "pick_motion",
        "fixed_reference_enabled",
        fallback=False,
    )
    PICK_FIXED_REFERENCE_FLANGE_COORDS = (
        _numbers(
            _parser.get("pick_motion", "fixed_reference_flange_coords"),
            6,
            "pick_motion.fixed_reference_flange_coords",
        )
        if _parser.has_option("pick_motion", "fixed_reference_flange_coords")
        else None
    )
    PICK_VISUAL_XY_CORRECTION_ENABLED = _parser.getboolean(
        "pick_motion",
        "visual_xy_correction_enabled",
        fallback=False,
    )
    PICK_VISUAL_REFERENCE_PLAN_XY_MM = _numbers(
        _parser.get(
            "pick_motion",
            "visual_reference_plan_xy_mm",
            fallback="0.0, 0.0",
        ),
        2,
        "pick_motion.visual_reference_plan_xy_mm",
    )
    PICK_VISUAL_XY_CORRECTION_MAX_MM = _parser.getfloat(
        "pick_motion",
        "visual_xy_correction_max_mm",
        fallback=15.0,
    )
    PICK_TWO_STAGE_APPROACH_ENABLED = _parser.getboolean(
        "pick_motion",
        "two_stage_approach_enabled",
        fallback=True,
    )
    PICK_APPROACH_LIFT_Z_MM = _parser.getfloat("pick_motion", "approach_lift_z_mm", fallback=50.0)
    PICK_APPROACH_SPEED = _parser.getint("pick_motion", "approach_speed", fallback=MOVE_SPEED)
    PICK_FINAL_APPROACH_SPEED = _parser.getint("pick_motion", "final_approach_speed", fallback=10)
    PICK_FINAL_APPROACH_MODE = _parser.getint("pick_motion", "final_approach_mode", fallback=1)
else:
    PICK_AUTO_ROTATED_XY_CORRECTION_ENABLED = False
    PICK_AUTO_ROTATED_MIN_RZ_OFFSET_DEG = 45.0
    PICK_AUTO_ROTATED_BASE_XY_OFFSET_MM = [0.0, 0.0]
    PICK_PLAN_BASE_XY_OFFSET_MM = [0.0, 0.0]
    PICK_FIXED_REFERENCE_ENABLED = False
    PICK_FIXED_REFERENCE_FLANGE_COORDS = None
    PICK_VISUAL_XY_CORRECTION_ENABLED = False
    PICK_VISUAL_REFERENCE_PLAN_XY_MM = [0.0, 0.0]
    PICK_VISUAL_XY_CORRECTION_MAX_MM = 15.0
    PICK_TWO_STAGE_APPROACH_ENABLED = True
    PICK_APPROACH_LIFT_Z_MM = 50.0
    PICK_APPROACH_SPEED = MOVE_SPEED
    PICK_FINAL_APPROACH_SPEED = 10
    PICK_FINAL_APPROACH_MODE = 1
if PICK_AUTO_ROTATED_MIN_RZ_OFFSET_DEG < 0:
    raise ValueError("pick_motion auto_rotated_min_rz_offset_deg must be >= 0")
if any(abs(value) > 20.0 for value in PICK_PLAN_BASE_XY_OFFSET_MM):
    raise ValueError("pick_motion plan_base_xy_offset_mm must be within +/-20mm")
if PICK_FIXED_REFERENCE_ENABLED and PICK_FIXED_REFERENCE_FLANGE_COORDS is None:
    raise ValueError(
        "pick_motion fixed_reference_flange_coords is required when enabled"
    )
if PICK_VISUAL_XY_CORRECTION_MAX_MM < 0:
    raise ValueError(
        "pick_motion visual_xy_correction_max_mm must be >= 0"
    )
if PICK_APPROACH_LIFT_Z_MM < 0:
    raise ValueError("pick_motion approach_lift_z_mm must be >= 0")
if PICK_APPROACH_SPEED <= 0:
    raise ValueError("pick_motion approach_speed must be positive")
if PICK_FINAL_APPROACH_SPEED <= 0:
    raise ValueError("pick_motion final_approach_speed must be positive")
if PICK_FINAL_APPROACH_MODE not in {0, 1}:
    raise ValueError("pick_motion final_approach_mode must be 0 or 1")

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
    PLACE_APPROACH_SPEED = _parser.getint("place_motion", "approach_speed", fallback=MOVE_SPEED)
    PLACE_APPROACH_SLOWDOWN_ENABLED = _parser.getboolean(
        "place_motion",
        "approach_slowdown_enabled",
        fallback=False,
    )
    PLACE_APPROACH_SPEEDS = [
        int(round(value))
        for value in _number_list(
            _parser.get(
                "place_motion",
                "approach_speeds",
                fallback=str(PLACE_APPROACH_SPEED),
            ),
            "place_motion.approach_speeds",
        )
    ]
    PLACE_RELEASE_PAUSE_SEC = _parser.getfloat("place_motion", "release_pause_sec", fallback=0.0)
    PLACE_GRIPPER_OPEN_SPEED = _parser.getint("place_motion", "gripper_open_speed", fallback=GRIPPER_SPEED)
    PLACE_GRIPPER_SETTLE_SEC = _parser.getfloat("place_motion", "gripper_settle_sec", fallback=GRIPPER_SETTLE_SEC)
else:
    PLACE_MOTION_ENABLED = True
    PLACE_FLANGE_COORDS = HOME_FLANGE_COORDS
    PLACE_APPROACH_SPEED = MOVE_SPEED
    PLACE_APPROACH_SLOWDOWN_ENABLED = False
    PLACE_APPROACH_SPEEDS = [MOVE_SPEED]
    PLACE_RELEASE_PAUSE_SEC = 0.0
    PLACE_GRIPPER_OPEN_SPEED = GRIPPER_SPEED
    PLACE_GRIPPER_SETTLE_SEC = GRIPPER_SETTLE_SEC
if PLACE_APPROACH_SPEED <= 0:
    raise ValueError("place_motion approach_speed must be positive")
if any(speed <= 0 for speed in PLACE_APPROACH_SPEEDS):
    raise ValueError("place_motion approach_speeds must contain positive values")
if PLACE_RELEASE_PAUSE_SEC < 0:
    raise ValueError("place_motion release_pause_sec must be >= 0")
if PLACE_GRIPPER_OPEN_SPEED <= 0:
    raise ValueError("place_motion gripper_open_speed must be positive")
if PLACE_GRIPPER_SETTLE_SEC < 0:
    raise ValueError("place_motion gripper_settle_sec must be >= 0")

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
    MARKER_PICKUP_IDS_RAW = _parser.get("marker_search", "pickup_ids", fallback="").strip()
    MARKER_PLACE_IDS_RAW = _parser.get("marker_search", "place_ids", fallback="").strip()
    MARKER_PREVIEW_DETECTION_ENABLED = _parser.getboolean(
        "marker_search", "preview_detection_enabled", fallback=True
    )
    MARKER_PREVIEW_DETECTION_FPS = _parser.getfloat(
        "marker_search", "preview_detection_fps", fallback=4.0
    )
    MARKER_SEARCH_PAN_JOINT = _parser.getint("marker_search", "pan_joint", fallback=1)
    MARKER_SEARCH_PAN_RANGE_DEG = _parser.getfloat("marker_search", "pan_range_deg", fallback=50.0)
    MARKER_SEARCH_STEP_DEG = _parser.getfloat("marker_search", "search_step_deg", fallback=3.0)
    MARKER_SEARCH_SPEED = _parser.getint("marker_search", "speed", fallback=25)
    MARKER_SEARCH_HZ = _parser.getfloat("marker_search", "hz", fallback=8.0)
    MARKER_SEARCH_MAX_DURATION_SEC = _parser.getfloat("marker_search", "max_duration_sec", fallback=0.0)
    MARKER_SEARCH_VIEW_JOINT_ENABLED = _parser.getboolean("marker_search", "view_joint_enabled", fallback=False)
    MARKER_SEARCH_VIEW_JOINTS_RAW = _parser.get(
        "marker_search",
        "view_joints",
        fallback=_parser.get("marker_search", "view_joint", fallback="4, 5"),
    )
    MARKER_SEARCH_VIEW_JOINTS = [
        int(item.strip())
        for item in MARKER_SEARCH_VIEW_JOINTS_RAW.split(",")
        if item.strip()
    ]
    MARKER_SEARCH_VIEW_JOINT_OFFSETS_RAW = _parser.get(
        "marker_search",
        "view_joint_offsets_deg",
        fallback=_parser.get("marker_search", "view_joint_offset_deg", fallback="0, 0"),
    )
    MARKER_SEARCH_VIEW_JOINT_OFFSETS_DEG = [
        [
            float(item.strip())
            for item in row.split(",")
            if item.strip()
        ]
        for row in MARKER_SEARCH_VIEW_JOINT_OFFSETS_RAW.split(";")
        if row.strip()
    ]
    MARKER_SEARCH_VIEW_JOINT_SPEED = _parser.getint("marker_search", "view_joint_speed", fallback=MOVE_SPEED)
else:
    MARKER_SEARCH_ENABLED = True
    MARKER_SEARCH_DICTIONARY = "DICT_APRILTAG_36h11"
    MARKER_SEARCH_TARGET_IDS_RAW = ""
    MARKER_PICKUP_IDS_RAW = ""
    MARKER_PLACE_IDS_RAW = ""
    MARKER_PREVIEW_DETECTION_ENABLED = True
    MARKER_PREVIEW_DETECTION_FPS = 4.0
    MARKER_SEARCH_PAN_JOINT = 1
    MARKER_SEARCH_PAN_RANGE_DEG = 50.0
    MARKER_SEARCH_STEP_DEG = 3.0
    MARKER_SEARCH_SPEED = 25
    MARKER_SEARCH_HZ = 8.0
    MARKER_SEARCH_MAX_DURATION_SEC = 0.0
    MARKER_SEARCH_VIEW_JOINT_ENABLED = False
    MARKER_SEARCH_VIEW_JOINTS = [4, 5]
    MARKER_SEARCH_VIEW_JOINT_OFFSETS_DEG = [[0.0, 0.0]]
    MARKER_SEARCH_VIEW_JOINT_SPEED = MOVE_SPEED

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

def _parse_marker_ids(value: str, option: str) -> set[int]:
    try:
        return {int(item.strip()) for item in value.split(",") if item.strip()}
    except ValueError as exc:
        raise ValueError(f"marker_search {option} must be comma-separated integers") from exc


MARKER_PICKUP_IDS = _parse_marker_ids(MARKER_PICKUP_IDS_RAW, "pickup_ids")
MARKER_PLACE_IDS = _parse_marker_ids(MARKER_PLACE_IDS_RAW, "place_ids")
if MARKER_PICKUP_IDS & MARKER_PLACE_IDS:
    raise ValueError("marker_search pickup_ids and place_ids must not overlap")
if not (MARKER_PICKUP_IDS | MARKER_PLACE_IDS).issubset(MARKER_SEARCH_TARGET_IDS):
    raise ValueError("marker_search pickup_ids/place_ids must be included in target_ids")
if MARKER_PREVIEW_DETECTION_FPS <= 0:
    raise ValueError("marker_search preview_detection_fps must be positive")

# Optional fixed pickup pose gated by recognition of one AprilTag. This is used
# for a tray whose grasp point is fixed relative to the robot base.
MARKER_PICKUP_ENABLED = _parser.getboolean("marker_pickup", "enabled", fallback=False)
MARKER_PICKUP_ARM_ID = _parser.get("marker_pickup", "arm_id", fallback="left").strip().lower()
MARKER_PICKUP_MARKER_ID = _parser.getint("marker_pickup", "marker_id", fallback=6)
MARKER_PICKUP_FLANGE_COORDS = _numbers(
    _parser.get(
        "marker_pickup",
        "target_flange_coords",
        fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS),
    ),
    6,
    "marker_pickup.target_flange_coords",
)
MARKER_PICKUP_TARGET_JOINT_ANGLES = _numbers(
    _parser.get(
        "marker_pickup",
        "target_joint_angles",
        fallback="0,0,0,0,0,0",
    ),
    6,
    "marker_pickup.target_joint_angles",
)
MARKER_PICKUP_USE_JOINT_TARGET = _parser.getboolean(
    "marker_pickup", "use_joint_target", fallback=False
)
MARKER_PICKUP_COMMAND_COMPENSATION_XYZ_MM = _numbers(
    _parser.get("marker_pickup", "command_compensation_xyz_mm", fallback="0,0,0"),
    3,
    "marker_pickup.command_compensation_xyz_mm",
)
MARKER_PICKUP_DYNAMIC_ALIGNMENT = _parser.getboolean(
    "marker_pickup", "dynamic_marker_alignment", fallback=True
)
MARKER_PICKUP_PLANE_Z_BASE_MM = _parser.getfloat(
    "marker_pickup", "marker_plane_z_base_mm", fallback=46.1
)
MARKER_PICKUP_TARGET_Z_OFFSET_MM = _parser.getfloat(
    "marker_pickup", "target_z_offset_mm", fallback=30.0
)
MARKER_PICKUP_TARGET_BASE_OFFSET_MM = _numbers(
    _parser.get("marker_pickup", "target_base_offset_mm", fallback="0,0,0"),
    3,
    "marker_pickup.target_base_offset_mm",
)
MARKER_PICKUP_GRASP_X_CLEARANCE_MM = _parser.getfloat(
    "marker_pickup", "grasp_x_clearance_mm", fallback=5.0
)
MARKER_PICKUP_GRASP_Z_CLEARANCE_MM = _parser.getfloat(
    "marker_pickup", "grasp_z_clearance_mm", fallback=5.0
)
MARKER_PICKUP_FINE_DESCENT_Z_MM = _parser.getfloat(
    "marker_pickup", "fine_descent_z_mm", fallback=0.0
)
MARKER_PICKUP_POST_PICK_LIFT_Z_MM = _parser.getfloat(
    "marker_pickup", "post_pick_lift_z_mm", fallback=20.0
)
MARKER_PICKUP_POST_PICK_LIFT_SPEED = _parser.getint(
    "marker_pickup", "post_pick_lift_speed", fallback=8
)
MARKER_PICKUP_TUNING_CLEARANCE_Z_MM = _parser.getfloat(
    "marker_pickup", "tuning_clearance_z_mm", fallback=15.0
)
MARKER_PICKUP_APPROACH_LIFT_Z_MM = _parser.getfloat(
    "marker_pickup", "approach_lift_z_mm", fallback=20.0
)
MARKER_PICKUP_APPROACH_SPEED = _parser.getint(
    "marker_pickup", "approach_speed", fallback=8
)
MARKER_PICKUP_APPROACH_POSITION_TOL_MM = _parser.getfloat(
    "marker_pickup", "approach_position_tolerance_mm", fallback=8.0
)
MARKER_PICKUP_DESCENT_SPEED = _parser.getint(
    "marker_pickup", "descent_speed", fallback=5
)
MARKER_PICKUP_JOINT_SEARCH_ENABLED = _parser.getboolean(
    "marker_pickup", "joint_search_enabled", fallback=False
)
MARKER_PICKUP_PAN_JOINT = _parser.getint(
    "marker_pickup", "pan_joint", fallback=1
)
MARKER_PICKUP_NEGATIVE_RANGE_DEG = _parser.getfloat(
    "marker_pickup", "negative_range_deg", fallback=30.0
)
MARKER_PICKUP_POSITIVE_RANGE_DEG = _parser.getfloat(
    "marker_pickup", "positive_range_deg", fallback=15.0
)
MARKER_PICKUP_JOINT_SEARCH_STEP_DEG = _parser.getfloat(
    "marker_pickup", "step_deg", fallback=3.0
)
MARKER_PICKUP_J5_OFFSETS_DEG = _number_list(
    _parser.get("marker_pickup", "j5_offsets_deg", fallback="0,-8,8"),
    "marker_pickup.j5_offsets_deg",
)
MARKER_PICKUP_JOINT_SEARCH_SPEED = _parser.getint(
    "marker_pickup", "joint_search_speed", fallback=8
)
MARKER_PICKUP_JOINT_SEARCH_SETTLE_SEC = _parser.getfloat(
    "marker_pickup", "joint_search_settle_sec", fallback=0.35
)
MARKER_PICKUP_JOINT_SEARCH_FRAME_TIMEOUT_SEC = _parser.getfloat(
    "marker_pickup", "joint_search_frame_timeout_sec", fallback=1.4
)
MARKER_PICKUP_RETURN_TO_START_IF_MISSING = _parser.getboolean(
    "marker_pickup", "return_to_start_if_missing", fallback=True
)
if MARKER_PICKUP_ARM_ID not in {"left", "right"}:
    raise ValueError("marker_pickup arm_id must be left or right")
if MARKER_PICKUP_ENABLED and MARKER_PICKUP_MARKER_ID not in MARKER_PICKUP_IDS:
    raise ValueError("marker_pickup marker_id must be included in marker_search.pickup_ids")
if MARKER_PICKUP_ENABLED:
    for axis, value, limits in zip(
        ("x", "y", "z"),
        MARKER_PICKUP_FLANGE_COORDS[:3],
        (SAFE_X_MM, SAFE_Y_MM, SAFE_Z_MM),
    ):
        if not limits[0] <= value <= limits[1]:
            raise ValueError(f"marker_pickup {axis}={value} is outside safety range {limits}")
    if any(abs(value) > SAFE_EULER_ABS_DEG for value in MARKER_PICKUP_FLANGE_COORDS[3:]):
        raise ValueError("marker_pickup orientation exceeds safe_euler_abs_deg")
if MARKER_PICKUP_USE_JOINT_TARGET:
    for joint_id, value in enumerate(MARKER_PICKUP_TARGET_JOINT_ANGLES, start=1):
        if not -180.0 <= value <= 180.0:
            raise ValueError(
                f"marker_pickup J{joint_id}={value} is outside +/-180 degrees"
            )
if any(abs(value) > 30.0 for value in MARKER_PICKUP_COMMAND_COMPENSATION_XYZ_MM):
    raise ValueError("marker_pickup command compensation must be within +/-30mm per axis")
if not 1 <= MARKER_PICKUP_PAN_JOINT <= 6:
    raise ValueError("marker_pickup pan_joint must be in the range 1..6")
if MARKER_PICKUP_NEGATIVE_RANGE_DEG <= 0 or MARKER_PICKUP_POSITIVE_RANGE_DEG <= 0:
    raise ValueError("marker_pickup angle ranges must be positive")
if MARKER_PICKUP_JOINT_SEARCH_STEP_DEG <= 0:
    raise ValueError("marker_pickup step_deg must be positive")
if MARKER_PICKUP_JOINT_SEARCH_SPEED <= 0:
    raise ValueError("marker_pickup joint_search_speed must be positive")
if MARKER_PICKUP_JOINT_SEARCH_SETTLE_SEC < 0:
    raise ValueError("marker_pickup joint_search_settle_sec must be >= 0")
if MARKER_PICKUP_JOINT_SEARCH_FRAME_TIMEOUT_SEC <= 0:
    raise ValueError("marker_pickup joint_search_frame_timeout_sec must be positive")
if MARKER_PICKUP_APPROACH_LIFT_Z_MM <= 0:
    raise ValueError("marker_pickup approach_lift_z_mm must be positive")
if MARKER_PICKUP_APPROACH_SPEED <= 0 or MARKER_PICKUP_DESCENT_SPEED <= 0:
    raise ValueError("marker_pickup approach/descent speeds must be positive")
if not 3.0 <= MARKER_PICKUP_APPROACH_POSITION_TOL_MM <= 10.0:
    raise ValueError("marker_pickup approach tolerance must be within 3..10mm")
if MARKER_PICKUP_GRASP_Z_CLEARANCE_MM < 0:
    raise ValueError("marker_pickup grasp_z_clearance_mm must be >= 0")
if not 0.0 <= MARKER_PICKUP_FINE_DESCENT_Z_MM <= 10.0:
    raise ValueError("marker_pickup fine_descent_z_mm must be within 0..10mm")
if abs(MARKER_PICKUP_GRASP_X_CLEARANCE_MM) > 20.0:
    raise ValueError("marker_pickup grasp_x_clearance_mm must be within +/-20mm")
if MARKER_PICKUP_POST_PICK_LIFT_Z_MM <= 0:
    raise ValueError("marker_pickup post_pick_lift_z_mm must be positive")
if MARKER_PICKUP_POST_PICK_LIFT_SPEED <= 0:
    raise ValueError("marker_pickup post_pick_lift_speed must be positive")
if MARKER_PICKUP_TUNING_CLEARANCE_Z_MM <= 0:
    raise ValueError("marker_pickup tuning_clearance_z_mm must be positive")

# Help-only placement. AprilTag ID 0 pickup returns HOME, moves to this
# operator-measured joint pose, releases the object, then returns HOME again.
HELP_PICK_JOINT_ANGLES = _numbers(
    _parser.get(
        "help",
        "pick_joint_angles",
        fallback="108.72, -45.52, -61.78, 23.02, -1.58, 63.19",
    ),
    6,
    "help.pick_joint_angles",
)
HELP_PICK_SPEED = _parser.getint("help", "pick_speed", fallback=5)
HELP_PLACE_JOINT_ANGLES = _numbers(
    _parser.get(
        "help",
        "place_joint_angles",
        fallback="77.87, -37.7, -58.71, 21.09, -2.54, 35.85",
    ),
    6,
    "help.place_joint_angles",
)
HELP_PLACE_SPEED = _parser.getint("help", "place_speed", fallback=10)
if any(
    abs(value) > 180.0
    for value in HELP_PICK_JOINT_ANGLES + HELP_PLACE_JOINT_ANGLES
):
    raise ValueError("help pick/place joint angles must be within +/-180 degrees")
if HELP_PICK_SPEED <= 0:
    raise ValueError("help pick_speed must be positive")
if HELP_PLACE_SPEED <= 0:
    raise ValueError("help place_speed must be positive")

BLACK_TABLE_ENABLED = _parser.getboolean("black_table", "enabled", fallback=False)
BLACK_TABLE_SURFACE_Z_BASE_MM = _parser.getfloat(
    "black_table", "surface_z_base_mm", fallback=0.0
)
BLACK_TABLE_MAX_VALUE = _parser.getint("black_table", "max_value", fallback=70)
BLACK_TABLE_MIN_AREA_RATIO = _parser.getfloat(
    "black_table", "min_area_ratio", fallback=0.08
)
BLACK_TABLE_MAX_AREA_RATIO = _parser.getfloat(
    "black_table", "max_area_ratio", fallback=0.70
)
BLACK_TABLE_BORDER_MARGIN_PX = _parser.getint(
    "black_table", "border_margin_px", fallback=8
)
BLACK_TABLE_APPROACH_LIFT_Z_MM = _parser.getfloat(
    "black_table", "approach_lift_z_mm", fallback=30.0
)
BLACK_TABLE_FIXED_PLACE_ENABLED = _parser.getboolean(
    "black_table", "fixed_place_enabled", fallback=False
)
BLACK_TABLE_FIXED_PLACE_FLANGE_COORDS = _numbers(
    _parser.get(
        "black_table",
        "fixed_place_flange_coords",
        fallback="115.1, 227.9, 187.6, -178.09, -3.63, -48.81",
    ),
    6,
    "black_table.fixed_place_flange_coords",
)
BLACK_TABLE_FINAL_JOINT_ANGLES = (
    _numbers(
        _parser.get("black_table", "final_joint_angles"),
        6,
        "black_table.final_joint_angles",
    )
    if _parser.has_option("black_table", "final_joint_angles")
    else None
)
BLACK_TABLE_FINAL_USE_JOINT_ANGLES = _parser.getboolean(
    "black_table", "final_use_joint_angles", fallback=True
)
BLACK_TABLE_FIXED_PLACE_SPEED = _parser.getint(
    "black_table", "fixed_place_speed", fallback=15
)
BLACK_TABLE_FIXED_PLACE_FINAL_SPEED = _parser.getint(
    "black_table", "fixed_place_final_speed", fallback=BLACK_TABLE_FIXED_PLACE_SPEED
)
BLACK_TABLE_FIXED_PLACE_APPROACH_TOL_MM = _parser.getfloat(
    "black_table", "fixed_place_approach_tol_mm", fallback=POSE_POSITION_TOL_MM
)
BLACK_TABLE_FIXED_PLACE_RETREAT_Z_TOL_MM = _parser.getfloat(
    "black_table", "fixed_place_retreat_z_tolerance_mm", fallback=3.0
)
BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_COUNT = _parser.getint(
    "black_table", "fixed_place_approach_retry_count", fallback=1
)
BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_INTERVAL_SEC = _parser.getfloat(
    "black_table", "fixed_place_approach_retry_interval_sec", fallback=0.5
)
BLACK_TABLE_SEARCH_MOVE_TO_START_ENABLED = _parser.getboolean(
    "black_table", "search_move_to_start_enabled", fallback=False
)
BLACK_TABLE_SEARCH_START_JOINT_ANGLES = _numbers(
    _parser.get(
        "black_table",
        "search_start_joint_angles",
        fallback="78.66, -49.39, -19.86, -21.44, -4.04, 37.44",
    ),
    6,
    "black_table.search_start_joint_angles",
)
BLACK_TABLE_SEARCH_START_SPEED = _parser.getint(
    "black_table", "search_start_speed", fallback=15
)
BLACK_TABLE_FINAL_FLANGE_Z_MM = _parser.getfloat(
    "black_table", "final_flange_z_mm", fallback=168.1
)
BLACK_TABLE_FINAL_FLANGE_ORIENTATION_DEG = _numbers(
    _parser.get(
        "black_table",
        "final_flange_orientation_deg",
        fallback="-175.86, 0.88, -57.17",
    ),
    3,
    "black_table.final_flange_orientation_deg",
)
BLACK_TABLE_SEARCH_Z_MM = _parser.getfloat(
    "black_table", "search_z_mm", fallback=SAFE_Z_MM[1]
)
BLACK_TABLE_SEARCH_X_RANGE_MM = _parser.getfloat(
    "black_table", "search_x_range_mm", fallback=100.0
)
BLACK_TABLE_SEARCH_X_STEP_MM = _parser.getfloat(
    "black_table", "search_x_step_mm", fallback=20.0
)
if not 0 <= BLACK_TABLE_MAX_VALUE <= 255:
    raise ValueError("black_table max_value must be within 0..255")
if not 0.0 < BLACK_TABLE_MIN_AREA_RATIO < BLACK_TABLE_MAX_AREA_RATIO < 1.0:
    raise ValueError("black_table area ratios must satisfy 0 < min < max < 1")
if BLACK_TABLE_BORDER_MARGIN_PX < 0:
    raise ValueError("black_table border_margin_px must be >= 0")
if BLACK_TABLE_APPROACH_LIFT_Z_MM <= 0:
    raise ValueError("black_table approach_lift_z_mm must be positive")
if BLACK_TABLE_FIXED_PLACE_SPEED <= 0:
    raise ValueError("black_table fixed_place_speed must be positive")
if BLACK_TABLE_FIXED_PLACE_FINAL_SPEED <= 0:
    raise ValueError("black_table fixed_place_final_speed must be positive")
if BLACK_TABLE_FIXED_PLACE_APPROACH_TOL_MM <= 0:
    raise ValueError("black_table fixed_place_approach_tol_mm must be positive")
if BLACK_TABLE_FIXED_PLACE_RETREAT_Z_TOL_MM < 0:
    raise ValueError(
        "black_table fixed_place_retreat_z_tolerance_mm must be >= 0"
    )
if BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_COUNT < 0:
    raise ValueError(
        "black_table fixed_place_approach_retry_count must be >= 0"
    )
if BLACK_TABLE_FIXED_PLACE_APPROACH_RETRY_INTERVAL_SEC < 0:
    raise ValueError(
        "black_table fixed_place_approach_retry_interval_sec must be >= 0"
    )
if BLACK_TABLE_SEARCH_START_SPEED <= 0:
    raise ValueError("black_table search_start_speed must be positive")
if (
    BLACK_TABLE_ENABLED
    and not SAFE_Z_MM[0] <= BLACK_TABLE_FINAL_FLANGE_Z_MM <= SAFE_Z_MM[1]
):
    raise ValueError("black_table final_flange_z_mm is outside the configured safe Z range")
if any(abs(angle) > 180.0 for angle in BLACK_TABLE_FINAL_FLANGE_ORIENTATION_DEG):
    raise ValueError("black_table final_flange_orientation_deg must be within +/-180deg")
if not SAFE_Z_MM[0] <= BLACK_TABLE_SEARCH_Z_MM <= SAFE_Z_MM[1]:
    raise ValueError("black_table search_z_mm is outside the configured safe Z range")
if BLACK_TABLE_SEARCH_X_RANGE_MM <= 0:
    raise ValueError("black_table search_x_range_mm must be positive")
if not 0 < BLACK_TABLE_SEARCH_X_STEP_MM <= BLACK_TABLE_SEARCH_X_RANGE_MM:
    raise ValueError("black_table search_x_step_mm must be within the search range")

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
if not MARKER_SEARCH_VIEW_JOINTS:
    raise ValueError("marker_search view_joints must contain at least one joint")
if any(not 1 <= joint <= 6 for joint in MARKER_SEARCH_VIEW_JOINTS):
    raise ValueError("marker_search view_joints must be in the range 1..6")
if MARKER_SEARCH_PAN_JOINT in MARKER_SEARCH_VIEW_JOINTS:
    raise ValueError("marker_search view_joints must not include pan_joint")
if not MARKER_SEARCH_VIEW_JOINT_OFFSETS_DEG:
    raise ValueError("marker_search view_joint_offsets_deg must contain at least one row")
if any(len(row) != len(MARKER_SEARCH_VIEW_JOINTS) for row in MARKER_SEARCH_VIEW_JOINT_OFFSETS_DEG):
    raise ValueError("marker_search view_joint_offsets_deg rows must match view_joints count")
if MARKER_SEARCH_VIEW_JOINT_SPEED <= 0:
    raise ValueError("marker_search view_joint_speed must be positive")

# Place marker view. Before marker search, rotate the flange around RZ while
# holding the object so the gripper is less likely to hide the AprilTag.
if _parser.has_section("place_marker_view"):
    PLACE_MARKER_VIEW_ROTATE_ENABLED = _parser.getboolean(
        "place_marker_view",
        "rotate_before_search_enabled",
        fallback=True,
    )
    PLACE_MARKER_VIEW_RZ_OFFSET_DEG = _parser.getfloat(
        "place_marker_view",
        "rz_offset_deg",
        fallback=25.0,
    )
    PLACE_MARKER_VIEW_SPEED = _parser.getint(
        "place_marker_view",
        "speed",
        fallback=15,
    )
else:
    PLACE_MARKER_VIEW_ROTATE_ENABLED = True
    PLACE_MARKER_VIEW_RZ_OFFSET_DEG = 25.0
    PLACE_MARKER_VIEW_SPEED = 15
if PLACE_MARKER_VIEW_SPEED <= 0:
    raise ValueError("place_marker_view speed must be positive")
if not -180.0 <= PLACE_MARKER_VIEW_RZ_OFFSET_DEG <= 180.0:
    raise ValueError("place_marker_view rz_offset_deg must be in -180..180")

# Pick search. If the first pick request cannot find a target, the client can
# move a small set of joints to get a different camera view and retry planning.
if _parser.has_section("pick_search"):
    PICK_SEARCH_ENABLED = _parser.getboolean("pick_search", "enabled", fallback=True)
    PICK_STATIONARY_RETRY_COUNT = _parser.getint(
        "pick_search", "stationary_retry_count", fallback=5
    )
    PICK_STATIONARY_RETRY_INTERVAL_SEC = _parser.getfloat(
        "pick_search", "stationary_retry_interval_sec", fallback=0.15
    )
    PICK_SEARCH_MOVE_TO_START_ENABLED = _parser.getboolean(
        "pick_search", "move_to_start_enabled", fallback=False
    )
    PICK_SEARCH_START_FLANGE_COORDS = _numbers(
        _parser.get(
            "pick_search",
            "start_flange_coords",
            fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS),
        ),
        6,
        "pick_search.start_flange_coords",
    )
    PICK_SEARCH_USE_FLANGE_POSE = _parser.getboolean("pick_search", "use_flange_pose", fallback=False)
    PICK_SEARCH_MAX_FLANGE_COORDS = _numbers(
        _parser.get(
            "pick_search",
            "max_flange_coords",
            fallback=", ".join(str(value) for value in HOME_FLANGE_COORDS),
        ),
        6,
        "pick_search.max_flange_coords",
    )
    PICK_SEARCH_TRANSLATION_AXIS = _parser.get(
        "pick_search", "translation_axis", fallback=""
    ).strip().lower()
    PICK_SEARCH_TRANSLATION_OFFSETS_MM = [
        float(item.strip())
        for item in _parser.get(
            "pick_search", "translation_offsets_mm", fallback="-20,20"
        ).split(",")
        if item.strip()
    ]
    PICK_SEARCH_LIFT_OFFSETS_MM = [
        float(item.strip())
        for item in _parser.get(
            "pick_search", "lift_offsets_mm", fallback="20"
        ).split(",")
        if item.strip()
    ]
    PICK_SEARCH_ANGLE_AXIS = _parser.get("pick_search", "angle_axis", fallback="rz").strip().lower()
    PICK_SEARCH_ANGLE_OFFSETS_DEG = [
        float(item.strip())
        for item in _parser.get("pick_search", "angle_offsets_deg", fallback="0,-25,25,-45,45").split(",")
        if item.strip()
    ]
    PICK_SEARCH_JOINTS = [
        int(item.strip())
        for item in _parser.get("pick_search", "joints", fallback="1,4").split(",")
        if item.strip()
    ]
    PICK_SEARCH_OFFSETS_DEG = [
        float(item.strip())
        for item in _parser.get("pick_search", "offsets_deg", fallback="0,-8,8,-16,16").split(",")
        if item.strip()
    ]
    PICK_SEARCH_GRID_ENABLED = _parser.getboolean("pick_search", "grid_enabled", fallback=False)
    PICK_SEARCH_GRID_JOINTS = [
        int(item.strip())
        for item in _parser.get("pick_search", "grid_joints", fallback="1,5").split(",")
        if item.strip()
    ]
    PICK_SEARCH_GRID_OFFSETS_DEG = [
        float(item.strip())
        for item in _parser.get("pick_search", "grid_offsets_deg", fallback="-10,0,10").split(",")
        if item.strip()
    ]
    PICK_SEARCH_SPEED = _parser.getint("pick_search", "speed", fallback=20)
    PICK_SEARCH_SETTLE_SEC = _parser.getfloat("pick_search", "settle_sec", fallback=0.25)
    PICK_SEARCH_FRAME_TIMEOUT_SEC = _parser.getfloat("pick_search", "frame_timeout_sec", fallback=1.0)
    PICK_SEARCH_RETURN_TO_START = _parser.getboolean("pick_search", "return_to_start", fallback=True)
else:
    PICK_STATIONARY_RETRY_COUNT = 5
    PICK_STATIONARY_RETRY_INTERVAL_SEC = 0.15
    PICK_SEARCH_ENABLED = True
    PICK_SEARCH_MOVE_TO_START_ENABLED = False
    PICK_SEARCH_START_FLANGE_COORDS = HOME_FLANGE_COORDS
    PICK_SEARCH_USE_FLANGE_POSE = False
    PICK_SEARCH_MAX_FLANGE_COORDS = HOME_FLANGE_COORDS
    PICK_SEARCH_TRANSLATION_AXIS = ""
    PICK_SEARCH_TRANSLATION_OFFSETS_MM = [-20.0, 20.0]
    PICK_SEARCH_LIFT_OFFSETS_MM = [20.0]
    PICK_SEARCH_ANGLE_AXIS = "rz"
    PICK_SEARCH_ANGLE_OFFSETS_DEG = [0.0, -25.0, 25.0, -45.0, 45.0]
    PICK_SEARCH_JOINTS = [1, 4]
    PICK_SEARCH_OFFSETS_DEG = [0.0, -8.0, 8.0, -16.0, 16.0]
    PICK_SEARCH_GRID_ENABLED = False
    PICK_SEARCH_GRID_JOINTS = [1, 5]
    PICK_SEARCH_GRID_OFFSETS_DEG = [-10.0, 0.0, 10.0]
    PICK_SEARCH_SPEED = 20
    PICK_SEARCH_SETTLE_SEC = 0.25
    PICK_SEARCH_FRAME_TIMEOUT_SEC = 1.0
    PICK_SEARCH_RETURN_TO_START = True

if PICK_STATIONARY_RETRY_COUNT < 0:
    raise ValueError("pick_search stationary_retry_count must be >= 0")
if PICK_STATIONARY_RETRY_INTERVAL_SEC < 0:
    raise ValueError("pick_search stationary_retry_interval_sec must be >= 0")
if PICK_SEARCH_MOVE_TO_START_ENABLED:
    for axis, value, limits in zip(
        ("x", "y", "z"),
        PICK_SEARCH_START_FLANGE_COORDS[:3],
        (SAFE_X_MM, SAFE_Y_MM, SAFE_Z_MM),
    ):
        if not limits[0] <= value <= limits[1]:
            raise ValueError(f"pick_search start {axis}={value} is outside safety range {limits}")
    if any(abs(value) > SAFE_EULER_ABS_DEG for value in PICK_SEARCH_START_FLANGE_COORDS[3:]):
        raise ValueError("pick_search start orientation exceeds safe_euler_abs_deg")
if not PICK_SEARCH_JOINTS:
    raise ValueError("pick_search joints must contain at least one joint")
if any(not 1 <= joint <= 6 for joint in PICK_SEARCH_JOINTS):
    raise ValueError("pick_search joints must be in the range 1..6")
if not PICK_SEARCH_OFFSETS_DEG:
    raise ValueError("pick_search offsets_deg must contain at least one value")
if PICK_SEARCH_GRID_ENABLED:
    if len(PICK_SEARCH_GRID_JOINTS) != 2:
        raise ValueError("pick_search grid_joints must contain exactly two joints")
    if any(not 1 <= joint <= 6 for joint in PICK_SEARCH_GRID_JOINTS):
        raise ValueError("pick_search grid_joints must be in the range 1..6")
    if not PICK_SEARCH_GRID_OFFSETS_DEG:
        raise ValueError("pick_search grid_offsets_deg must contain at least one value")
if PICK_SEARCH_ANGLE_AXIS not in {"rx", "ry", "rz"}:
    raise ValueError("pick_search angle_axis must be rx, ry, or rz")
if not PICK_SEARCH_ANGLE_OFFSETS_DEG:
    raise ValueError("pick_search angle_offsets_deg must contain at least one value")
if PICK_SEARCH_TRANSLATION_AXIS not in {"", "x", "y", "z"}:
    raise ValueError("pick_search translation_axis must be empty, x, y, or z")
if PICK_SEARCH_TRANSLATION_AXIS and not PICK_SEARCH_TRANSLATION_OFFSETS_MM:
    raise ValueError("pick_search translation_offsets_mm must contain at least one value")
if any(offset <= 0 for offset in PICK_SEARCH_LIFT_OFFSETS_MM):
    raise ValueError("pick_search lift_offsets_mm values must be positive")
if PICK_SEARCH_SPEED <= 0:
    raise ValueError("pick_search speed must be positive")
if PICK_SEARCH_SETTLE_SEC < 0:
    raise ValueError("pick_search settle_sec must be >= 0")

GIFT_SUPPLY_SEARCH_ENABLED = _parser.getboolean(
    "gift_supply_search", "enabled", fallback=True
)
GIFT_SUPPLY_SEARCH_VIEW_JOINT_ANGLES = _numbers(
    _parser.get(
        "gift_supply_search",
        "search_view_joint_angles",
        fallback="120.32, -49.74, 22.06, -58.0, 0.7, 76.11",
    ),
    6,
    "gift_supply_search.search_view_joint_angles",
)
GIFT_SUPPLY_PICKUP_JOINT_ANGLES = _numbers(
    _parser.get(
        "gift_supply_search",
        "pickup_joint_angles",
        fallback="123.39, -80.94, 20.74, -25.4, -2.46, 80.15",
    ),
    6,
    "gift_supply_search.pickup_joint_angles",
)
GIFT_SUPPLY_PICKUP_FLANGE_COORDS = _numbers(
    _parser.get(
        "gift_supply_search",
        "pickup_flange_coords",
        fallback="-92.3, 261.3, 153.4, -176.82, 3.90, -46.74",
    ),
    6,
    "gift_supply_search.pickup_flange_coords",
)
GIFT_SUPPLY_RESTOCK_PLACE_FLANGE_COORDS = _numbers(
    _parser.get(
        "gift_supply_search",
        "restock_place_flange_coords",
        fallback="-16.6, 275.8, 129.6, 175.52, -1.88, -37.98",
    ),
    6,
    "gift_supply_search.restock_place_flange_coords",
)
GIFT_SUPPLY_RESTOCK_PLACE_JOINT_ANGLES = _numbers(
    _parser.get(
        "gift_supply_search",
        "restock_place_joint_angles",
        fallback="106.52, -75.49, 0.26, -18.89, 2.54, 54.49",
    ),
    6,
    "gift_supply_search.restock_place_joint_angles",
)
GIFT_SUPPLY_MOTION_Z_OFFSET_MM = _parser.getfloat(
    "gift_supply_search",
    "motion_z_offset_mm",
    fallback=0.0,
)
GIFT_SUPPLY_SEARCH_J5_OFFSETS_DEG = _number_list(
    _parser.get(
        "gift_supply_search",
        "j5_offsets_deg",
        fallback="0, -8, 8, -15, 15",
    ),
    "gift_supply_search.j5_offsets_deg",
)
GIFT_SUPPLY_SEARCH_SPEED = _parser.getint(
    "gift_supply_search", "speed", fallback=8
)
GIFT_SUPPLY_SEARCH_SETTLE_SEC = _parser.getfloat(
    "gift_supply_search", "settle_sec", fallback=0.35
)
GIFT_SUPPLY_SEARCH_FRAME_TIMEOUT_SEC = _parser.getfloat(
    "gift_supply_search", "frame_timeout_sec", fallback=1.0
)
GIFT_SUPPLY_RESTOCK_PLACE_Z_OFFSET_MM = _parser.getfloat(
    "gift_supply_search",
    "restock_place_z_offset_mm",
    fallback=0.0,
)
GIFT_SUPPLY_RESTOCK_PLACE_Y_OFFSET_MM = _parser.getfloat(
    "gift_supply_search",
    "restock_place_y_offset_mm",
    fallback=0.0,
)
if GIFT_SUPPLY_SEARCH_SPEED <= 0:
    raise ValueError("gift_supply_search speed must be positive")
if (
    GIFT_SUPPLY_SEARCH_SETTLE_SEC < 0
    or GIFT_SUPPLY_SEARCH_FRAME_TIMEOUT_SEC <= 0
):
    raise ValueError("gift_supply_search settle/frame timeout values are invalid")
if not 0.0 <= GIFT_SUPPLY_RESTOCK_PLACE_Z_OFFSET_MM <= 50.0:
    raise ValueError(
        "gift_supply_search restock_place_z_offset_mm must be within 0..50mm"
    )
if abs(GIFT_SUPPLY_RESTOCK_PLACE_Y_OFFSET_MM) > 30.0:
    raise ValueError(
        "gift_supply_search restock_place_y_offset_mm must be within +/-30mm"
    )
if not 0.0 <= GIFT_SUPPLY_MOTION_Z_OFFSET_MM <= 30.0:
    raise ValueError(
        "gift_supply_search motion_z_offset_mm must be within 0..30mm"
    )
if PICK_SEARCH_FRAME_TIMEOUT_SEC <= 0:
    raise ValueError("pick_search frame_timeout_sec must be positive")

# Pick centering. When the server sees only partial detections near the image
# edge, move the camera view toward the detected bbox center and retry before
# falling back to the wider pick_search view list.
if _parser.has_section("pick_centering"):
    PICK_CENTERING_ENABLED = _parser.getboolean("pick_centering", "enabled", fallback=True)
    PICK_CENTERING_MAX_ATTEMPTS = _parser.getint("pick_centering", "max_attempts", fallback=4)
    PICK_CENTERING_DEADBAND_PX = _parser.getfloat("pick_centering", "deadband_px", fallback=50.0)
    PICK_CENTERING_J1_GAIN_DEG_PER_PX = _parser.getfloat(
        "pick_centering",
        "j1_gain_deg_per_px",
        fallback=-0.04,
    )
    PICK_CENTERING_J2_GAIN_DEG_PER_PX = _parser.getfloat(
        "pick_centering",
        "j2_gain_deg_per_px",
        fallback=0.025,
    )
    PICK_CENTERING_J5_GAIN_DEG_PER_PX = _parser.getfloat(
        "pick_centering",
        "j5_gain_deg_per_px",
        fallback=0.04,
    )
    PICK_CENTERING_MAX_STEP_DEG = _parser.getfloat("pick_centering", "max_step_deg", fallback=8.0)
    PICK_CENTERING_SPEED = _parser.getint("pick_centering", "speed", fallback=PICK_SEARCH_SPEED)
    PICK_CENTERING_SETTLE_SEC = _parser.getfloat("pick_centering", "settle_sec", fallback=PICK_SEARCH_SETTLE_SEC)
else:
    PICK_CENTERING_ENABLED = True
    PICK_CENTERING_MAX_ATTEMPTS = 4
    PICK_CENTERING_DEADBAND_PX = 50.0
    PICK_CENTERING_J1_GAIN_DEG_PER_PX = -0.04
    PICK_CENTERING_J2_GAIN_DEG_PER_PX = 0.025
    PICK_CENTERING_J5_GAIN_DEG_PER_PX = 0.04
    PICK_CENTERING_MAX_STEP_DEG = 8.0
    PICK_CENTERING_SPEED = PICK_SEARCH_SPEED
    PICK_CENTERING_SETTLE_SEC = PICK_SEARCH_SETTLE_SEC
if PICK_CENTERING_MAX_ATTEMPTS < 0:
    raise ValueError("pick_centering max_attempts must be >= 0")
if PICK_CENTERING_DEADBAND_PX < 0:
    raise ValueError("pick_centering deadband_px must be >= 0")
if PICK_CENTERING_MAX_STEP_DEG <= 0:
    raise ValueError("pick_centering max_step_deg must be positive")
if PICK_CENTERING_SPEED <= 0:
    raise ValueError("pick_centering speed must be positive")
if PICK_CENTERING_SETTLE_SEC < 0:
    raise ValueError("pick_centering settle_sec must be >= 0")

# Left Arm recycling. YOLO decides whether the picked object is trash or water.
# The color detector confirms that the matching bin is visible before the arm
# follows the operator-measured joint path to the center of that bin.
RECYCLE_ENABLED = _parser.getboolean("recycle", "enabled", fallback=False)
RECYCLE_DYNAMIC_COLOR_TARGET = _parser.getboolean(
    "recycle",
    "dynamic_color_target",
    fallback=False,
)
RECYCLE_BLUE_FIXED_TARGET = _parser.getboolean(
    "recycle",
    "blue_fixed_target",
    fallback=True,
)
RECYCLE_RED_FIXED_TARGET = _parser.getboolean(
    "recycle",
    "red_fixed_target",
    fallback=True,
)
RECYCLE_BIN_PLANE_Z_BASE_MM = _parser.getfloat(
    "recycle",
    "bin_plane_z_base_mm",
    fallback=90.0,
)
RECYCLE_DYNAMIC_MAX_XY_OFFSET_MM = _parser.getfloat(
    "recycle",
    "dynamic_max_xy_offset_mm",
    fallback=40.0,
)
RECYCLE_VIEW_FLANGE_COORDS = _numbers(
    _parser.get(
        "recycle",
        "view_flange_coords",
        fallback="14.0, 142.0, 318.8, -164.32, 21.22, -40.96",
    ),
    6,
    "recycle.view_flange_coords",
)
RECYCLE_VIEW_JOINT_ANGLES = _numbers(
    _parser.get(
        "recycle",
        "view_joint_angles",
        fallback="113.29, -20.21, 20.3, -64.42, -5.27, 66.0",
    ),
    6,
    "recycle.view_joint_angles",
)
RECYCLE_VIEW_OBSERVE_SEC = _parser.getfloat(
    "recycle",
    "view_observe_sec",
    fallback=3.0,
)
RECYCLE_RED_FLANGE_COORDS = _numbers(
    _parser.get(
        "recycle",
        "red_flange_coords",
        fallback="-64.4, 214.3, 196.1, -169.52, 6.98, 47.90",
    ),
    6,
    "recycle.red_flange_coords",
)
RECYCLE_RED_JOINT_ANGLES = _numbers(
    _parser.get(
        "recycle",
        "red_joint_angles",
        fallback="121.37, -21.97, -68.29, 8.43, 9.58, -15.2",
    ),
    6,
    "recycle.red_joint_angles",
)
RECYCLE_BLUE_FLANGE_COORDS = _numbers(
    _parser.get(
        "recycle",
        "blue_flange_coords",
        fallback="-94.5, 239.8, 193.8, -178.80, 5.06, -35.96",
    ),
    6,
    "recycle.blue_flange_coords",
)
RECYCLE_BLUE_JOINT_ANGLES = _numbers(
    _parser.get(
        "recycle",
        "blue_joint_angles",
        fallback="125.94, -70.4, 24.96, -39.37, 0.43, 71.98",
    ),
    6,
    "recycle.blue_joint_angles",
)
RECYCLE_MOTION_SPEED = _parser.getint("recycle", "motion_speed", fallback=20)
RECYCLE_BLUE_MOTION_SPEED = _parser.getint(
    "recycle",
    "blue_motion_speed",
    fallback=10,
)
RECYCLE_COLOR_MIN_AREA_RATIO = _parser.getfloat(
    "recycle",
    "color_min_area_ratio",
    fallback=0.01,
)
RECYCLE_COLOR_SATURATION_MIN = _parser.getint(
    "recycle",
    "color_saturation_min",
    fallback=90,
)
RECYCLE_COLOR_VALUE_MIN = _parser.getint(
    "recycle",
    "color_value_min",
    fallback=50,
)
if RECYCLE_MOTION_SPEED <= 0:
    raise ValueError("recycle motion_speed must be positive")
if RECYCLE_BLUE_MOTION_SPEED <= 0:
    raise ValueError("recycle blue_motion_speed must be positive")
if RECYCLE_DYNAMIC_MAX_XY_OFFSET_MM <= 0:
    raise ValueError("recycle dynamic_max_xy_offset_mm must be positive")
if RECYCLE_VIEW_OBSERVE_SEC <= 0:
    raise ValueError("recycle view_observe_sec must be positive")
if not 0.0 < RECYCLE_COLOR_MIN_AREA_RATIO < 1.0:
    raise ValueError("recycle color_min_area_ratio must be within 0..1")
if not 0 <= RECYCLE_COLOR_SATURATION_MIN <= 255:
    raise ValueError("recycle color_saturation_min must be within 0..255")
if not 0 <= RECYCLE_COLOR_VALUE_MIN <= 255:
    raise ValueError("recycle color_value_min must be within 0..255")

# UI
SHOW_WINDOW = _parser.getboolean("ui", "show_window")
WINDOW_NAME = _parser.get("ui", "window_name")
