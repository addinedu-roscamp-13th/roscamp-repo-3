"""웹앱 인텐트 → 콘솔 명령(topic, payload) 매핑 (순수, ROS/Qt 무관). spec §10.4.

기존 콘솔 계약을 재사용한다: `roles.cmd_mode_payload`, `roles.MODES`, `fleet.robot_topic`.
새 통신 체계를 만들지 않는다 — 이 모듈은 기존 topic/payload를 웹앱 인텐트에 매핑만 한다.
"""
import json

from . import fleet, roles

CMD_MODE_TOPIC = "/wasab/cmd_mode"
ESTOP_TOPIC = "/wasab/estop"
ESTOP_SOURCE = "webapp"

_VALID_MODES = {m["id"] for m in roles.MODES}          # idle / patrol / assist
_RELOCALIZE = {"start": "detector:start", "stop": "detector:stop", "rearm": "rearm"}


def cmd_mode(robot_id, mode):
    """순찰/대기/보조 모드 명령 → (topic, payload_json). mode ∈ roles.MODES id."""
    if mode not in _VALID_MODES:
        raise ValueError(f"unknown mode: {mode!r} (expected one of {sorted(_VALID_MODES)})")
    return CMD_MODE_TOPIC, roles.cmd_mode_payload(robot_id, mode)


def _estop_payload(target, active, command_id):
    """/wasab/estop payload — ros_control.estop_payload 계약과 동일 형식."""
    return json.dumps({"target": str(target), "source": ESTOP_SOURCE, "active": bool(active), "command_id": int(command_id)})


def estop_msg(target, active, command_id=0):
    """긴급정지 명령 → (topic, payload). target="all" 또는 로봇 id. (agent가 필터·relay)"""
    return ESTOP_TOPIC, _estop_payload(target, active, command_id)


def estop_robot(robot_id, active=True, command_id=0):
    """특정 로봇 비상정지 → (topic, payload)."""
    return estop_msg(str(int(robot_id)), active, command_id)


def dock_start(robot_id, tag_id):
    """도킹/홈복귀(교사) → (topic, payload). 태그 기반 도킹."""
    return fleet.robot_topic(robot_id, "dock_cmd"), f"start:{int(tag_id)}"


def relocalize(robot_id, action):
    """재측위 명령 → (topic, payload). action ∈ {start, stop, rearm}."""
    if action not in _RELOCALIZE:
        raise ValueError(f"unknown relocalize action: {action!r} (expected one of {sorted(_RELOCALIZE)})")
    return fleet.robot_topic(robot_id, "relocalize_cmd"), _RELOCALIZE[action]
