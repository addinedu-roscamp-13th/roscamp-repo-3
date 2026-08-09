"""fleet-state 애그리게이터 (순수, ROS/Qt 무관). spec §10.3.

robots.yaml 구성(`roles.load_robots`)과 라이브 heartbeat(`fleet.parse_heartbeat`)를
프론트 UI 모델로 병합만 한다. 새 통신/파서를 만들지 않는다.
"""

# 콘솔 모드 wire값(roles.MODES: idle/patrol/assist) → 프론트 마커 kind(색상)
_MODE_KIND = {"assist": "teacher", "patrol": "safety", "idle": "idle"}


def mode_to_kind(mode):
    """로봇 mode(idle/patrol/assist) → 프론트 kind(teacher/safety/idle). 미정의는 idle."""
    return _MODE_KIND.get(mode, "idle")


def robot_view(rid, cfg, hb, tag=None, reloc=None, dock=None):
    """구성(cfg={name,ip,camera}) + heartbeat(hb dict|None) → 로봇 1대 UI 모델.

    tag/reloc/dock는 상태 이벤트 최신 스냅샷(dict|None): /robots/tag_status·relocalize_event·
    dock_event 원문. 없으면 None. (P4-b: 도킹·재측위 진행/태그 정렬 표시용.)
    """
    cfg = cfg or {}
    online = hb is not None
    mode = hb.get("mode") if online else None
    pose = None
    if online and hb.get("x") is not None and hb.get("y") is not None and hb.get("yaw") is not None:
        pose = {"x": hb["x"], "y": hb["y"], "yaw": hb["yaw"]}
    return {
        "id": rid,
        "name": cfg.get("name"),
        "ip": cfg.get("ip"),
        "mode": mode,
        "kind": mode_to_kind(mode),
        "domain": cfg.get("domain"),
        "battery_v": hb.get("battery_v") if online else None,
        "age_ms": hb.get("age_ms") if online else None,
        "online": online,
        "pose": pose,
        "tag": tag,
        "relocalize": reloc,
        "dock": dock,
    }


def fleet_view(config, heartbeats, tags=None, relocs=None, docks=None):
    """구성 dict{rid:cfg} + heartbeats dict{rid:hb} → id 오름차순 UI 모델 리스트.

    ⚠ 계약: `heartbeats`는 **이미 신선한(fresh) 것만** 담긴 dict여야 한다. stale 판정·제거는
    호출자(rclpy 브리지)가 **`fleet.prune_stale(robots, now)` 재사용**으로 수행한다 —
    heartbeats에 없는 로봇은 offline. (disconnected robot이 web UI에서 online으로 남지 않게 함.)
    각 heartbeat에 `age_ms`가 있으면 뷰로 통과시킨다(신선도 표시용).
    """
    tags = tags or {}
    relocs = relocs or {}
    docks = docks or {}
    return [robot_view(rid, config.get(rid), heartbeats.get(rid),
                       tags.get(rid), relocs.get(rid), docks.get(rid))
            for rid in sorted(config)]
