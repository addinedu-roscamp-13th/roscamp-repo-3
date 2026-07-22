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
    default_target_label: str | None

    save_results: bool
    save_root_dir: Path

    intrinsic_file: Path
    handeye_result_json: Path
    euler_order: str
    object_plane_z_base_mm: float
    toothbrush_target_z_offset_mm: float
    target_z_offsets_mm: dict[str, float]
    tcp_offset_flange_to_tcp_mm: tuple[float, float, float]

    max_upload_bytes: int
    expected_image_width: int
    expected_image_height: int

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

        target_label = _require(parser, "model", "default_target_label")
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

        return cls(
            model_path=_resolve_path(_require(parser, "model", "model_path")),
            device=_require(parser, "model", "device") or "cpu",
            default_conf=parser.getfloat("model", "default_conf"),
            default_imgsz=parser.getint("model", "default_imgsz"),
            default_target_label=target_label or None,
            save_results=parser.getboolean("logging", "save_results"),
            save_root_dir=_resolve_path(_require(parser, "logging", "save_root_dir")),
            intrinsic_file=_resolve_path(_require(parser, "calibration", "intrinsic_file")),
            handeye_result_json=_resolve_path(_require(parser, "calibration", "handeye_result_json")),
            euler_order=_require(parser, "calibration", "euler_order").lower(),
            object_plane_z_base_mm=parser.getfloat("calibration", "object_plane_z_base_mm"),
            toothbrush_target_z_offset_mm=parser.getfloat("calibration", "toothbrush_target_z_offset_mm"),
            target_z_offsets_mm=target_z_offsets_mm,
            tcp_offset_flange_to_tcp_mm=_csv_triplet(
                _require(parser, "calibration", "tcp_offset_flange_to_tcp_mm"),
                "tcp_offset_flange_to_tcp_mm",
            ),
            max_upload_bytes=parser.getint("request_validation", "max_upload_bytes"),
            expected_image_width=parser.getint("request_validation", "expected_image_width"),
            expected_image_height=parser.getint("request_validation", "expected_image_height"),
            udp_stream_enabled=parser.getboolean("udp_stream", "enabled", fallback=True),
            udp_stream_host=parser.get("udp_stream", "host", fallback="0.0.0.0").strip() or "0.0.0.0",
            udp_stream_port=parser.getint("udp_stream", "port", fallback=8001),
            udp_stream_max_datagram_bytes=parser.getint("udp_stream", "max_datagram_bytes", fallback=1400),
            udp_stream_frame_timeout_sec=parser.getfloat("udp_stream", "frame_timeout_sec", fallback=1.0),
            host=_require(parser, "server", "host") or "0.0.0.0",
            port=parser.getint("server", "port"),
        )


settings = Settings.from_file()
