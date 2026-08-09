# wasab_docking/pose_filter.py
"""tag pose outlier reject (ROS 무관, pytest). 직전 유효 pose 대비 큰 튐을 폐기.

전이(새 goal / NAV_TO_APPROACH→SEARCH_TAG / terminal→IDLE)에서 reset()해
접근 전/중의 오래된 pose로 첫 검출을 오탈락시키지 않는다(리뷰 2차 #1).
"""
import math


class PoseFilter:
    def __init__(self, max_jump_m, max_yaw_jump_rad):
        self.max_jump_m = max_jump_m
        self.max_yaw_jump_rad = max_yaw_jump_rad
        self._prev = None

    def reset(self):
        self._prev = None

    def accept(self, pose):
        x, y, yaw = pose
        if self._prev is None:
            self._prev = (x, y, yaw)
            return True
        px, py, pyaw = self._prev
        dist = math.hypot(x - px, y - py)
        dyaw = abs(math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw)))
        if dist > self.max_jump_m or dyaw > self.max_yaw_jump_rad:
            return False
        self._prev = (x, y, yaw)
        return True


class YawFilter:
    """평면 마커 pose ambiguity로 프레임마다 ±부호 flip하는 tag yaw를 안정화.

    최근 window개 yaw의 circular mean(atan2(Σsin, Σcos))을 반환한다. 대칭 ±flip은
    평균이 참값(정면 근처 ~0)으로 수렴해 servo wz 요동을 없앤다. x/y는 안정적이라
    필터하지 않는다(위치 lag 방지). 전이에서 reset().
    """

    def __init__(self, window=6):
        self.window = max(1, int(window))
        self._buf = []

    def reset(self):
        self._buf = []

    def update(self, yaw):
        self._buf.append(yaw)
        if len(self._buf) > self.window:
            self._buf.pop(0)
        s = sum(math.sin(a) for a in self._buf)
        c = sum(math.cos(a) for a in self._buf)
        return math.atan2(s, c)
