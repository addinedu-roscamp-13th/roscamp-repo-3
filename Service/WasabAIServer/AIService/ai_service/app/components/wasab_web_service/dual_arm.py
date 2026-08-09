"""Configuration and safety gate for the two-JetCobot scenario."""
from __future__ import annotations

import configparser
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


CONFIG_PATH = Path(__file__).resolve().parents[3] / "config" / "dual_arm_config.ini"
REQUIRED_RIGHT_FIELDS = (
    "robot_ip",
    "pickup_view_flange_coords",
    "place_flange_coords",
    "home_flange_coords",
    "intrinsic_file",
    "handeye_result_json",
)


def _six_numbers(parser: configparser.ConfigParser, section: str, option: str) -> list[float] | None:
    raw = parser.get(section, option, fallback="").strip()
    if not raw:
        return None
    values = [float(item.strip()) for item in raw.split(",") if item.strip()]
    if len(values) != 6:
        raise ValueError(f"{section}.{option} must contain six comma-separated values")
    return values


@dataclass
class DualArmRuntime:
    lock: threading.Lock = field(default_factory=threading.Lock)
    running: bool = False
    phase: str = "idle"
    last_error: str | None = None
    worker: threading.Thread | None = None


runtime = DualArmRuntime()


def load_dual_arm_config() -> dict[str, Any]:
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(CONFIG_PATH, encoding="utf-8")
    steps = [item.strip() for item in parser.get("sequence", "steps").split(",") if item.strip()]
    right_missing = [
        option for option in REQUIRED_RIGHT_FIELDS
        if not parser.get("right_arm", option, fallback="").strip()
    ]
    right_ready_requested = parser.getboolean("right_arm", "ready", fallback=False)
    return {
        "enabled": parser.getboolean("dual_arm", "enabled", fallback=False),
        "scenario_name": parser.get("dual_arm", "scenario_name"),
        "steps": steps,
        "left": {
            "arm_id": parser.get("left_arm", "arm_id", fallback="left"),
            "ready": parser.getboolean("left_arm", "ready", fallback=False),
            "robot_ip": parser.get("left_arm", "robot_ip"),
            "pickup_marker_id": parser.getint("left_arm", "pickup_marker_id"),
            "handoff_flange_coords": _six_numbers(parser, "left_arm", "handoff_flange_coords"),
            "home_flange_coords": _six_numbers(parser, "left_arm", "home_flange_coords"),
            "intrinsic_file": parser.get("left_arm", "intrinsic_file"),
            "handeye_result_json": parser.get("left_arm", "handeye_result_json"),
        },
        "right": {
            "arm_id": parser.get("right_arm", "arm_id", fallback="right"),
            "ready": right_ready_requested and not right_missing,
            "ready_requested": right_ready_requested,
            "missing_fields": right_missing,
            "pickup_marker_id": parser.getint("right_arm", "pickup_marker_id", fallback=2),
        },
    }


def dual_arm_status() -> dict[str, Any]:
    config = load_dual_arm_config()
    with runtime.lock:
        config.update({
            "running": runtime.running,
            "phase": runtime.phase,
            "last_error": runtime.last_error,
        })
    config["can_start"] = bool(
        config["enabled"] and config["left"]["ready"] and config["right"]["ready"]
        and not config["running"]
    )
    return config
