"""다중 로봇 heartbeat 순수 헬퍼 (Qt/ROS 무관, pytest)."""
import json

ROBOT_STALE_S = 3.0


def parse_heartbeat(json_str):
    """heartbeat JSON → dict|None. id 필수. x/y/yaw는 셋 다 있을 때만 pose 포함."""
    try:
        d = json.loads(json_str)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict) or "id" not in d:
        return None
    out = {
        "id": int(d["id"]),
        "ip": d.get("ip"),
        "battery_v": (float(d["battery_v"]) if d.get("battery_v") is not None else None),
        "mode": d.get("mode"),
    }
    x, y, yaw = d.get("x"), d.get("y"), d.get("yaw")
    if x is not None and y is not None and yaw is not None:
        out["x"] = float(x)
        out["y"] = float(y)
        out["yaw"] = float(yaw)
    return out


def prune_stale(robots, now_monotonic, max_age=ROBOT_STALE_S):
    """rx(monotonic) 기준 age>max_age 로봇 제거."""
    return {rid: r for rid, r in robots.items()
            if now_monotonic - r.get("rx", 0.0) <= max_age}


def robot_topic(rid, suffix):
    """/robot_<rid>/<suffix> (앞 슬래시 제거)."""
    return f"/robot_{int(rid)}/{suffix.lstrip('/')}"


def parse_tag_status(json_str):
    """/robots/tag_status JSON → dict|None. id 필수."""
    try:
        d = json.loads(json_str)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict) or d.get("id") is None:
        return None
    return {"id": int(d["id"]), "tag_id": d.get("tag_id"),
            "dist_cm": d.get("dist_cm"), "lateral_cm": d.get("lateral_cm"),
            "age_ms": d.get("age_ms")}


def parse_relocalize_event(json_str):
    """/robots/relocalize_event JSON → dict|None. id·event 필수, event ∈ {success,timeout}."""
    try:
        d = json.loads(json_str)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict) or d.get("id") is None:
        return None
    event = d.get("event")
    if event not in ("success", "timeout"):
        return None
    return {"id": int(d["id"]), "event": event, "tag_id": d.get("tag_id")}


def parse_dock_event(json_str):
    """/robots/dock_event JSON → dict|None. id·event 필수, failed면 reason 필수."""
    try:
        d = json.loads(json_str)
    except (ValueError, TypeError):
        return None
    if not isinstance(d, dict) or d.get("id") is None:
        return None
    event = d.get("event")
    if event not in ("started", "done", "failed"):
        return None
    reason = d.get("reason")
    if event == "failed" and not reason:
        return None
    return {"id": int(d["id"]), "event": event,
            "tag_id": d.get("tag_id"), "reason": reason}


def liveness_state(ping_ok, hb_fresh):
    """ping 도달 + heartbeat 신선 → 'online'|'powered'|'offline'.

    heartbeat가 오면 ping 무관하게 online(더 강한 신호). 그다음 ping만 있으면
    powered(전원켜짐·agent대기), 둘 다 없으면 offline.
    """
    if hb_fresh:
        return "online"
    if ping_ok:
        return "powered"
    return "offline"
