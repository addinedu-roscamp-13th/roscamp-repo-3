"""순수 순찰 FSM (ROS/rclpy 무관). 웨이포인트 무한 순회 + 타 로봇 근접 시 양보.

좌표는 (x, y) 튜플(map frame, m). 순찰로봇은 최하위 우선순위 —
타 로봇이 yield_radius 안에 들면 정지(YIELD), 모두 clear_radius 밖이면 재개(히스테리시스).
'비켜 돌기'는 하지 않는다(좁은 아레나 전제). ROS 노드가 own_xy/other_xys/at_goal을 주입한다.
"""
import math


class PatrolPlanner:
    def __init__(self, waypoints, yield_radius, clear_radius, reach_tol=0.15):
        if not waypoints:
            raise ValueError("waypoints empty")
        if float(clear_radius) < float(yield_radius):
            raise ValueError("clear_radius must be >= yield_radius (hysteresis)")
        self._wps = list(waypoints)
        self._yield_r = float(yield_radius)
        self._clear_r = float(clear_radius)
        self._reach_tol = float(reach_tol)
        self._idx = 0
        self._state = "PATROL"

    @property
    def state(self):
        return self._state

    @property
    def target(self):
        return self._wps[self._idx]

    def step(self, own_xy, other_xys, at_goal=False):
        d = self._nearest(own_xy, other_xys)
        if self._state == "PATROL" and d < self._yield_r:
            self._state = "YIELD"
        elif self._state == "YIELD" and d >= self._clear_r:
            self._state = "PATROL"
        if self._state == "YIELD":
            return {"action": "stop", "waypoint": self._wps[self._idx], "state": "YIELD"}
        if at_goal:
            self._idx = (self._idx + 1) % len(self._wps)
        return {"action": "goto", "waypoint": self._wps[self._idx], "state": "PATROL"}

    @staticmethod
    def _nearest(own_xy, other_xys):
        if not other_xys:
            return float("inf")
        ox, oy = own_xy
        return min(math.hypot(ox - x, oy - y) for (x, y) in other_xys)
