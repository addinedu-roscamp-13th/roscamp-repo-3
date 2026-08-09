# wasab_docking/state_machine.py
"""정밀 주차 순수 상태머신 (ROS 무관, pytest). 시간·오차·nav 상태를 인자로 주입.

Phase 2: IDLE → NAV_TO_APPROACH → (NAV_CANCELING) → SEARCH_TAG →
         TAG_SERVO_ALIGN → SETTLE → DONE / FAILED.
nav 부수효과(goal 전송/취소)는 nav_cmd로 반환하고 노드가 수행한다(순수 유지).
"""
from wasab_docking import pid

_ZERO = {"vx": 0.0, "wz": 0.0}
_NAV_ACTIVE = ("inactive", "waiting_server", "pending", "active")


class DockingStateMachine:
    def __init__(self, cfg):
        self.cfg = cfg
        self.nav_enabled = cfg.get("nav_enabled", False)
        self.state = "IDLE"
        self._start = 0.0
        self._precision_start = 0.0      # SEARCH_TAG 진입 시각(overall_timeout 기준, 리뷰 #1)
        self._search_start = 0.0
        self._nav_start = 0.0
        self._cancel_start = 0.0
        self._last_seen = 0.0
        self._settle_start = 0.0
        self._settle_count = 0           # 연속 tolerance 프레임(리뷰 #4)
        self._nav_send_pending = False
        self._fail_reason = None
        self._done_error = None

    # --- 수명주기 ---
    def start(self, now):
        self._start = now
        self._last_seen = now
        self._settle_count = 0
        self._fail_reason = None
        self._done_error = None
        if self.nav_enabled:
            if not self.cfg.get("approach_pose_set", False):
                self.state = "FAILED"
                self._fail_reason = "approach_pose_missing"   # 원점 오발송 차단(리뷰 #3)
                return
            self.state = "NAV_TO_APPROACH"
            self._nav_start = now
            self._nav_send_pending = True
        else:
            self._enter_search(now)

    def _enter_search(self, now):
        self.state = "SEARCH_TAG"
        self._precision_start = now      # 정밀단계 타이머 리셋(리뷰 #1)
        self._search_start = now
        self._last_seen = now
        self._settle_count = 0

    # --- 출력 ---
    def _out(self, cmd, settled=False, nav_cmd=None):
        s = settled or (self.state == "DONE")
        return {"state": self.state, "vx": cmd["vx"], "wz": cmd["wz"],
                "settled": s, "nav_cmd": nav_cmd, "fail_reason": self._fail_reason,
                "settle_count": self._settle_count, "done_error": self._done_error}

    def _fail(self, reason):
        self.state = "FAILED"
        self._fail_reason = reason
        return self._out(_ZERO)

    def _servo(self, errors):
        cmd = pid.servo_cmd(errors, self.cfg["tag_goal"], self.cfg["gains"],
                            self.cfg["limits"], self.cfg["tols"])
        return self._out(cmd)

    # --- 메인 ---
    def update(self, now, errors, nav=None):
        if self.state in ("DONE", "FAILED"):
            return self._out(_ZERO)
        if errors is not None:
            self._last_seen = now

        st = self.state
        if st == "IDLE":
            return self._out(_ZERO)
        if st == "NAV_TO_APPROACH":
            return self._nav_update(now, nav)
        if st == "NAV_CANCELING":
            return self._cancel_update(now, nav)
        if st == "SEARCH_TAG":
            return self._search_update(now, errors)
        if st == "TAG_SERVO_ALIGN":
            return self._align_update(now, errors)
        if st == "SETTLE":
            return self._settle_update(now, errors)
        return self._out(_ZERO)

    def _nav_update(self, now, nav):
        status = nav or "inactive"
        if self._nav_send_pending:
            self._nav_send_pending = False
            self._nav_start = now
            return self._out(_ZERO, nav_cmd="send")
        if status == "succeeded":
            self._enter_search(now)
            return self._out(_ZERO)
        if status == "server_unavailable":
            return self._fail("nav_server_unavailable")
        if status == "rejected":
            return self._fail("nav_goal_rejected")
        if status == "failed":
            return self._fail("nav_failed")
        if status == "canceled":
            return self._fail("cancelled")
        if (now - self._nav_start) > self.cfg["nav_result_timeout_s"]:
            self.state = "NAV_CANCELING"
            self._cancel_start = now
            return self._out(_ZERO, nav_cmd="cancel")   # 아직 FAILED 아님(리뷰 #2)
        return self._out(_ZERO)

    def _cancel_update(self, now, nav):
        status = nav or "inactive"
        if status in ("canceled", "failed", "inactive"):
            return self._fail("nav_timeout")
        if (now - self._cancel_start) > self.cfg["nav_cancel_wait_s"]:
            return self._fail("nav_timeout")            # 강제 종료(status active면 노드가 warning)
        return self._out(_ZERO)

    def _search_update(self, now, errors):
        if errors is None:
            if (now - self._search_start) > self.cfg["search_timeout_s"]:
                return self._fail("tag_search_timeout")
            return self._out(_ZERO)
        self.state = "TAG_SERVO_ALIGN"
        self._settle_count = 0
        return self._servo(errors)

    def _align_update(self, now, errors):
        if errors is None:
            if (now - self._last_seen) > self.cfg["tag_lost_timeout_s"]:
                return self._fail("tag_lost")
            return self._out(_ZERO)
        if (now - self._precision_start) > self.cfg["overall_timeout_s"]:
            return self._fail("overall_timeout")
        if pid.within_tolerance(errors, self.cfg["tols"]):
            self._settle_count += 1
            if self._settle_count >= self.cfg["settle_min_frames"]:   # 연속 N회(리뷰 #4)
                self.state = "SETTLE"
                self._settle_start = now
        else:
            self._settle_count = 0
        return self._servo(errors)

    def _settle_update(self, now, errors):
        if (now - self._precision_start) > self.cfg["overall_timeout_s"]:
            return self._fail("overall_timeout")
        if errors is None or not pid.within_tolerance(errors, self.cfg["tols"]):
            self.state = "TAG_SERVO_ALIGN"
            self._settle_count = 0
            return self._servo(errors) if errors is not None else self._out(_ZERO)
        if (now - self._settle_start) >= self.cfg["settle_time_s"]:
            self.state = "DONE"
            self._done_error = dict(errors)                # DONE snapshot(리뷰 #4·#5)
            return self._out(_ZERO, settled=True)
        return self._servo(errors)
