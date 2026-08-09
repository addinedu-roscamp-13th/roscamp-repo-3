"""로봇 역할(보조/순찰)과 역할별 제어 모드 버튼 — 순수(Qt/ROS 무관).

cmd_mode 페이로드 형식은 행동코어 계약 확정 전 임시값(JSON {robot_id, mode}).
"""
import json

# ---------------------------------------------------------------------------
# 공용 모드 모델 — 모든 로봇 공유. id=ROS cmd_mode 페이로드 값, label=UI 한글.
# 1차 목록(§10 최종목록 추후): 추종/운반/음악지휘/감지대응 등은 확정 후 추가.
# ---------------------------------------------------------------------------
MODES = [
    {"id": "idle", "label": "대기"},
    {"id": "patrol", "label": "교내순찰"},
    {"id": "assist", "label": "선생님보조"},
]
_MODE_LABELS = {m["id"]: m["label"] for m in MODES}


def mode_label(mode_id):
    """모드 id → UI 표시 label. 미정의면 id 그대로."""
    return _MODE_LABELS.get(mode_id, mode_id)


def load_robots(path):
    """robots.yaml(robots:{id:{name,ip,camera}}) → {int: {"name": str, "ip": str|None, "camera": dict|None}}.
    실패 시 {}(never-raises)."""
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except Exception:
        return {}
    raw = data.get("robots", {}) if isinstance(data, dict) else {}
    out = {}
    if isinstance(raw, dict):
        for k, v in raw.items():
            try:
                rid = int(k)
            except (ValueError, TypeError):
                continue
            if isinstance(v, dict):
                name = str(v.get("name", f"로봇 {rid}"))
                ip = v.get("ip")
                ip = str(ip) if ip is not None else None
                camera = v.get("camera")
                domain = v.get("domain")
            else:
                name, ip, camera, domain = f"로봇 {rid}", None, None, None
            out[rid] = {"name": name, "ip": ip, "camera": camera, "domain": domain}
    return out


def cmd_mode_payload(robot_id, mode):
    """cmd_mode 토픽 페이로드(JSON 문자열). 임시 계약."""
    return json.dumps({"robot_id": int(robot_id), "mode": mode})
