"""재측위 순수 로직 — observation JSON, 후보 선택, 명령 파싱, freshness. rclpy 무관."""
import json


def serialize_observation(tags, sec, nanosec):
    return json.dumps({"stamp": {"sec": int(sec), "nanosec": int(nanosec)},
                       "tags": tags})


def parse_observation(s):
    d = json.loads(s)
    st = d["stamp"]
    return (float(st["sec"]) + float(st["nanosec"]) * 1e-9, d.get("tags", []))


def select_registered_max_area(tags, registered_ids):
    cands = [t for t in tags if int(t["tag_id"]) in registered_ids]
    if not cands:
        return None
    best = max(cands, key=lambda t: t["area"])
    return (int(best["tag_id"]),
            (float(best["x"]), float(best["y"]), float(best["yaw"])))


def find_tag(tags, tag_id):
    for t in tags:
        if int(t["tag_id"]) == int(tag_id):
            return (float(t["x"]), float(t["y"]), float(t["yaw"]))
    return None


def parse_cmd(s):
    s = (s or "").strip()
    if s == "rearm":
        return ("rearm", None)
    if s.startswith("calibrate:"):
        rest = s.split(":", 1)[1]
        try:
            return ("calibrate", int(rest))
        except ValueError:
            return (None, None)
    return (None, None)


def is_fresh(stamp_sec, now_sec, max_age_s):
    return (now_sec - stamp_sec) <= max_age_s


class RelocalizeState:
    """정지-게이트 원샷 재측위 + 재무장 상태머신. now = 초 단위 float."""

    def __init__(self, cfg):
        self.cfg = cfg
        self._armed = True
        self._last_tag_id = None
        self._last_reloc_t = None
        self._travel = 0.0
        self._stationary = 0

    def on_odom(self, vx, wz, dist_delta):
        if abs(vx) < self.cfg["v_thresh"] and abs(wz) < self.cfg["w_thresh"]:
            self._stationary += 1
        else:
            self._stationary = 0
            self._travel += abs(dist_delta)

    def on_rearm_cmd(self):
        self._armed = True                       # 수동: cooldown 무관 즉시 재무장

    def is_stationary(self):
        return self._stationary >= self.cfg["stationary_frames"]

    def _cooldown_ok(self, now):
        return self._last_reloc_t is None or (now - self._last_reloc_t) >= self.cfg["cooldown_s"]

    def _maybe_auto_rearm(self, selected_tag_id, now):
        if self._armed or not self.cfg["auto_rearm_enabled"] or not self._cooldown_ok(now):
            return
        if self._travel >= self.cfg["min_rearm_travel_m"]:
            self._armed = True
        elif self._last_tag_id is not None and selected_tag_id != self._last_tag_id:
            self._armed = True

    def try_relocalize(self, selected_tag_id, now):
        self._maybe_auto_rearm(selected_tag_id, now)
        if not (self._armed and self.is_stationary()):
            return False
        self._armed = False
        self._last_tag_id = selected_tag_id
        self._last_reloc_t = now
        self._travel = 0.0
        return True
