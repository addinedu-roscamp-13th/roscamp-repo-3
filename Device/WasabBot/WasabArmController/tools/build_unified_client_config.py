#!/usr/bin/env python3
"""Build one client_config.ini with Left defaults and Right section overrides."""
from __future__ import annotations

import configparser
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CONFIG_DIR = ROOT / "config"
LEFT_PATH = CONFIG_DIR / "client_config.left.ini"
RIGHT_PATH = CONFIG_DIR / "client_config.right.ini"
OUTPUT_PATH = CONFIG_DIR / "client_config.ini"


def load(path: Path) -> configparser.ConfigParser:
    parser = configparser.ConfigParser(interpolation=None)
    parser.read(path, encoding="utf-8")
    return parser


left = load(LEFT_PATH)
right = load(RIGHT_PATH)
right.setdefault("right_pick_reference", {})
right.setdefault("hand_gesture", {})
right["hand_gesture"]["hitbox_calibration_enabled"] = "false"
right["hand_gesture"]["release_confirmation_enabled"] = "true"
right["hand_gesture"]["release_confirmation_mode"] = "hand"
right["hand_gesture"]["release_confirmation_timeout_sec"] = "15.0"
right["hand_gesture"]["release_confirmation_hold_sec"] = "1.0"
right["right_pick_reference"]["motion_strategy"] = "hybrid"
right["right_pick_reference"]["ik_correction_max_xy_mm"] = "5.0"
right["right_pick_reference"]["ik_position_tol_mm"] = "12.0"
right["right_pick_reference"]["ik_angle_tol_deg"] = "5.0"
right["right_pick_reference"]["ik_min_motion_sec"] = "1.0"
right["right_pick_reference"]["use_reference_xyz"] = "false"
right["right_pick_reference"]["use_reference_z"] = "true"
right["right_pick_reference"]["use_reference_orientation"] = "true"
right["right_pick_reference"]["use_joint_target"] = "false"

lines = [
    LEFT_PATH.read_text(encoding="utf-8").rstrip(),
    "",
    "; ============================================================================",
    "; Right Arm overrides",
    "; The device-local config/arm_identity file selects these sections.",
    "; ============================================================================",
]

for section in right.sections():
    differences: list[tuple[str, str]] = []
    for option, value in right.items(section, raw=True):
        left_value = left.get(section, option, fallback=None, raw=True)
        if left_value != value:
            differences.append((option, value))
    if not differences:
        continue
    lines.extend(["", f"[right.{section}]"])
    lines.extend(f"{option} = {value}" for option, value in differences)

OUTPUT_PATH.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(OUTPUT_PATH)
