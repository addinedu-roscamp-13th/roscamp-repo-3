"""순수 fleet 상태 저장/집계 (ROS/Qt 무관). spec §10.3.

기존 재사용: fleet.parse_heartbeat(파싱), fleet.prune_stale(stale 제거), state.fleet_view(집계).
rclpy 브리지가 ingest_heartbeat로 원문 JSON을 흘려넣고, server가 fleet()로 스냅샷을 읽는다.

⚠ 스레드 안전: ingest_heartbeat()는 ROS spin 스레드, fleet()는 FastAPI(HTTP/WS) 스레드에서
호출되므로 _hb 접근을 threading.Lock으로 보호한다.
"""
import json
import threading
import time

from . import fleet, state


class FleetStore:
    def __init__(self, config, clock=time.monotonic, stale_after_s=fleet.ROBOT_STALE_S):
        self._config = config
        self._clock = clock
        self._stale_after_s = stale_after_s
        self._hb = {}                      # id -> parsed heartbeat + "rx"(monotonic)
        self._tag = {}                     # id -> 최신 tag_status 원문 dict
        self._reloc = {}                   # id -> 최신 relocalize_event 원문 dict
        self._dock = {}                    # id -> 최신 dock_event 원문 dict
        self._lock = threading.Lock()

    def ingest_heartbeat(self, json_str):
        r = fleet.parse_heartbeat(json_str)     # 파싱은 lock 밖(순수)
        if r is None:
            return
        r["rx"] = self._clock()
        with self._lock:
            self._hb[r["id"]] = r

    @staticmethod
    def _parse_event(json_str):
        """상태 이벤트 JSON → (id:int, dict) 또는 (None, None). id 없거나 파싱 실패면 None."""
        try:
            d = json.loads(json_str)
            return int(d["id"]), d
        except (ValueError, TypeError, KeyError):
            return None, None

    def ingest_tag_status(self, json_str):
        rid, d = self._parse_event(json_str)
        if rid is None:
            return
        with self._lock:
            self._tag[rid] = d

    def ingest_relocalize_event(self, json_str):
        rid, d = self._parse_event(json_str)
        if rid is None:
            return
        with self._lock:
            self._reloc[rid] = d

    def ingest_dock_event(self, json_str):
        rid, d = self._parse_event(json_str)
        if rid is None:
            return
        with self._lock:
            self._dock[rid] = d

    def fleet(self):
        now = self._clock()
        with self._lock:                        # _hb 읽기/prune/쓰기를 한 lock 안에서
            fresh = fleet.prune_stale(self._hb, now, self._stale_after_s)
            self._hb = fresh
            hbs = {}
            for rid, r in fresh.items():
                hb = dict(r)
                hb["age_ms"] = int((now - r.get("rx", now)) * 1000)
                hbs[rid] = hb
            tags = dict(self._tag)                   # 이벤트 스냅샷도 lock 안에서 copy
            relocs = dict(self._reloc)
            docks = dict(self._dock)
        return state.fleet_view(self._config, hbs, tags, relocs, docks)   # 집계는 lock 밖
