# wasab_docking/estop.py
"""E-STOP 페이로드 파싱(순수, rclpy 무관). /wasab/estop = {"target","source","active":bool}."""
import json


def parse_estop(data):
    """/wasab/estop JSON → active(bool). 파싱 실패/필드 없음/비-dict → False.

    명시적 active=True일 때만 정지 트리거(오작동 정지 방지). target은 무시하고
    어떤 E-STOP이든 이 노드를 멈춘다(안전 우선 — 넓게 정지)."""
    try:
        d = json.loads(data)
    except (ValueError, TypeError):
        return False
    return bool(isinstance(d, dict) and d.get("active", False))
