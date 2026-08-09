"""노트북 로컬 YOLO/파지계획 설정 로더.

사용자가 수정하는 파일은 ``config/server_config.ini`` 하나입니다.
환경변수 export 없이 ``python run_server.py``로 실행할 수 있도록 구성했습니다.
"""
from __future__ import annotations

import configparser
from dataclasses import dataclass
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config" / "server_config.ini"


def _require(parser: configparser.ConfigParser, section: str, option: str) -> str:
    try:
        value = parser.get(section, option)
    except (configparser.NoSectionError, configparser.NoOptionError) as exc:
        raise RuntimeError(f"Missing setting [{section}] {option} in {DEFAULT_CONFIG_PATH}") from exc
    return value.strip()


def _resolve_path(value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else (PROJECT_ROOT / path).resolve()


def _csv_triplet(value: str, name: str) -> tuple[float, float, float]:
    try:
        numbers = tuple(float(item.strip()) for item in value.split(","))
    except ValueError as exc:
        raise ValueError(f"{name} must be three comma-separated numbers") from exc
    if len(numbers) != 3:
        raise ValueError(f"{name} must contain exactly three values")
    return numbers  # type: ignore[return-value]


@dataclass(frozen=True)
class Settings:
    model_path: Path
    device: str
    default_conf: float
    default_imgsz: int

    save_results: bool
    save_root_dir: Path

    intrinsic_file: Path
    handeye_result_json: Path
    euler_order: str
    object_plane_z_base_mm: float
    default_target_z_offset_mm: float
    place_extra_z_offset_mm: float
    marker_place_tcp_offset_base_mm: tuple[float, float, float]
    target_z_offsets_mm: dict[str, float]
    pick_target_base_offsets_mm: dict[str, tuple[float, float, float]]
    tcp_offset_flange_to_tcp_mm: tuple[float, float, float]
    pick_flange_orientation_deg: tuple[float, float, float] | None
    long_object_end_grip_enabled: bool
    long_object_end_grip_aspect_ratio: float
    long_object_end_grip_offset_fraction: float
    long_object_end_grip_side: str
    gripper_auto_rotate_long_bbox_enabled: bool
    gripper_auto_rotate_aspect_ratio: float
    gripper_auto_rotate_rz_offset_deg: float

    max_upload_bytes: int
    expected_image_width: int
    expected_image_height: int
    plan_safe_x_mm: tuple[float, float]
    plan_safe_y_mm: tuple[float, float]
    plan_safe_z_mm: tuple[float, float]
    plan_safe_euler_abs_deg: float
    right_intrinsic_file: Path | None = None
    right_handeye_result_json: Path | None = None
    handover_zone_norm: tuple[float, float, float, float] = (0.05, 0.95, 0.08, 0.96)

    pick_roi_enabled: bool = False
    pick_roi_x_px: tuple[int, int] = (0, 640)
    pick_roi_y_px: tuple[int, int] = (0, 480)

    udp_stream_enabled: bool = True
    udp_stream_host: str = "0.0.0.0"
    udp_stream_port: int = 8001
    udp_stream_max_datagram_bytes: int = 1400
    udp_stream_frame_timeout_sec: float = 1.0

    # run_server.py에서만 사용하는 노트북 수신 주소입니다. 기본값을 두어
    # 기존 geometry/unit-test 코드가 Settings를 직접 생성해도 호환됩니다.
    host: str = "0.0.0.0"
    port: int = 8000

    @classmethod
    def from_file(cls, path: Path = DEFAULT_CONFIG_PATH) -> "Settings":
        parser = configparser.ConfigParser(interpolation=None)
        if not path.exists():
            raise FileNotFoundError(f"Server config not found: {path}")
        parser.read(path, encoding="utf-8")

        target_z_offsets_mm: dict[str, float] = {}
        if parser.has_section("target_z_offsets_mm"):
            for label, value in parser.items("target_z_offsets_mm"):
                label = label.strip()
                if not label:
                    continue
                try:
                    target_z_offsets_mm[label] = float(value)
                except ValueError as exc:
                    raise ValueError(
                        f"[target_z_offsets_mm] {label} must be a number"
                    ) from exc

        pick_target_base_offsets_mm: dict[str, tuple[float, float, float]] = {}
        if parser.has_section("pick_target_base_offsets_mm"):
            for label, value in parser.items("pick_target_base_offsets_mm"):
                label = label.strip()
                if not label:
                    continue
                pick_target_base_offsets_mm[label] = _csv_triplet(
                    value,
                    f"pick_target_base_offsets_mm.{label}",
                )

        return cls(
            model_path=_resolve_path(_require(parser, "model", "model_path")),
            device=_require(parser, "model", "device") or "cpu",
            default_conf=parser.getfloat("model", "default_conf"),
            default_imgsz=parser.getint("model", "default_imgsz"),
            save_results=parser.getboolean("logging", "save_results"),
            save_root_dir=_resolve_path(_require(parser, "logging", "save_root_dir")),
            intrinsic_file=_resolve_path(_require(parser, "calibration", "intrinsic_file")),
            handeye_result_json=_resolve_path(_require(parser, "calibration", "handeye_result_json")),
            euler_order=_require(parser, "calibration", "euler_order").lower(),
            object_plane_z_base_mm=parser.getfloat("calibration", "object_plane_z_base_mm"),
            default_target_z_offset_mm=parser.getfloat(
                "calibration",
                "default_target_z_offset_mm",
                fallback=40.0,
            ),
            place_extra_z_offset_mm=parser.getfloat("calibration", "place_extra_z_offset_mm", fallback=0.0),
            marker_place_tcp_offset_base_mm=_csv_triplet(
                parser.get("calibration", "marker_place_tcp_offset_base_mm", fallback="30.0, 0.0, 0.0"),
                "marker_place_tcp_offset_base_mm",
            ),
            target_z_offsets_mm=target_z_offsets_mm,
            pick_target_base_offsets_mm=pick_target_base_offsets_mm,
            tcp_offset_flange_to_tcp_mm=_csv_triplet(
                _require(parser, "calibration", "tcp_offset_flange_to_tcp_mm"),
                "tcp_offset_flange_to_tcp_mm",
            ),
            pick_flange_orientation_deg=(
                _csv_triplet(
                    parser.get("calibration", "pick_flange_orientation_deg", fallback="").strip(),
                    "pick_flange_orientation_deg",
                )
                if parser.get("calibration", "pick_flange_orientation_deg", fallback="").strip()
                else None
            ),
            long_object_end_grip_enabled=parser.getboolean(
                "grip_target",
                "long_object_end_grip_enabled",
                fallback=False,
            ),
            long_object_end_grip_aspect_ratio=parser.getfloat(
                "grip_target",
                "long_object_end_grip_aspect_ratio",
                fallback=1.35,
            ),
            long_object_end_grip_offset_fraction=parser.getfloat(
                "grip_target",
                "long_object_end_grip_offset_fraction",
                fallback=0.35,
            ),
            long_object_end_grip_side=parser.get(
                "grip_target",
                "long_object_end_grip_side",
                fallback="lower",
            ).strip().lower() or "lower",
            gripper_auto_rotate_long_bbox_enabled=parser.getboolean(
                "gripper_orientation",
                "auto_rotate_long_bbox_enabled",
                fallback=True,
            ),
            gripper_auto_rotate_aspect_ratio=parser.getfloat(
                "gripper_orientation",
                "auto_rotate_aspect_ratio",
                fallback=1.35,
            ),
            gripper_auto_rotate_rz_offset_deg=parser.getfloat(
                "gripper_orientation",
                "auto_rotate_rz_offset_deg",
                fallback=90.0,
            ),
            max_upload_bytes=parser.getint("request_validation", "max_upload_bytes"),
            expected_image_width=parser.getint("request_validation", "expected_image_width"),
            expected_image_height=parser.getint("request_validation", "expected_image_height"),
            plan_safe_x_mm=(
                parser.getfloat("plan_safety", "safe_x_min_mm", fallback=-260.0),
                parser.getfloat("plan_safety", "safe_x_max_mm", fallback=260.0),
            ),
            plan_safe_y_mm=(
                parser.getfloat("plan_safety", "safe_y_min_mm", fallback=-260.0),
                parser.getfloat("plan_safety", "safe_y_max_mm", fallback=260.0),
            ),
            plan_safe_z_mm=(
                parser.getfloat("plan_safety", "safe_z_min_mm", fallback=-50.0),
                parser.getfloat("plan_safety", "safe_z_max_mm", fallback=350.0),
            ),
            plan_safe_euler_abs_deg=parser.getfloat("plan_safety", "safe_euler_abs_deg", fallback=360.0),
            right_intrinsic_file=(
                _resolve_path(parser.get("right_calibration", "intrinsic_file"))
                if parser.has_option("right_calibration", "intrinsic_file")
                else None
            ),
            right_handeye_result_json=(
                _resolve_path(parser.get("right_calibration", "handeye_result_json"))
                if parser.has_option("right_calibration", "handeye_result_json")
                else None
            ),
            handover_zone_norm=(
                parser.getfloat("handover_zone", "x_min_norm", fallback=0.05),
                parser.getfloat("handover_zone", "x_max_norm", fallback=0.95),
                parser.getfloat("handover_zone", "y_min_norm", fallback=0.08),
                parser.getfloat("handover_zone", "y_max_norm", fallback=0.96),
            ),
            pick_roi_enabled=parser.getboolean("pick_roi", "enabled", fallback=False),
            pick_roi_x_px=(
                parser.getint("pick_roi", "x_min_px", fallback=0),
                parser.getint("pick_roi", "x_max_px", fallback=640),
            ),
            pick_roi_y_px=(
                parser.getint("pick_roi", "y_min_px", fallback=0),
                parser.getint("pick_roi", "y_max_px", fallback=480),
            ),
            udp_stream_enabled=parser.getboolean("udp_stream", "enabled", fallback=True),
            udp_stream_host=parser.get("udp_stream", "host", fallback="0.0.0.0").strip() or "0.0.0.0",
            udp_stream_port=parser.getint("udp_stream", "port", fallback=8001),
            udp_stream_max_datagram_bytes=parser.getint("udp_stream", "max_datagram_bytes", fallback=1400),
            udp_stream_frame_timeout_sec=parser.getfloat("udp_stream", "frame_timeout_sec", fallback=1.0),
            host=_require(parser, "server", "host") or "0.0.0.0",
            port=parser.getint("server", "port"),
        )


settings = Settings.from_file()
