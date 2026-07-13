#!/usr/bin/env python3
"""Check the direct Raspberry Pi -> laptop-local YOLO service connection.

This script does not initialize the robot or send a motion command.

Usage:
  cd raspberry_pi_laptop_client
  python3 check_server_connection.py
"""
from __future__ import annotations

from robot_client import config
from robot_client.api_client import WaSaBServiceError, check_wasab_service_health


def main() -> int:
    print("[CHECK] laptop grasp_server_url =", config.GRASP_SERVER_URL)
    try:
        health = check_wasab_service_health()
    except WaSaBServiceError as exc:
        print("[FAIL]", exc)
        return 1

    print("[OK] Laptop server reachable")
    print("[OK] runtime =", health.get("runtime"))
    print("[OK] device =", health.get("device"))
    print("[OK] model_path =", health.get("model_path"))
    print(
        "[OK] expected calibration image size =",
        "640x480 (verify against laptop server_config.ini)",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
