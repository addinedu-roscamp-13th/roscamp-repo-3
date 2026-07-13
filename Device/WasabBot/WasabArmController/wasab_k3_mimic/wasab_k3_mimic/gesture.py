# Copyright 2026 gjkong
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
"""제스처 인식 순수로직 (MediaPipe Hands 21 landmark → 명령)."""
import math

# 검지·중지·약지·새끼 (tip, pip) landmark 인덱스.
# 엄지는 손목 거리 판정이 불안정(굽혀도 멀어 +1 오인식)해 세지 않는다.
_FINGER_JOINTS = ((8, 6), (12, 10), (16, 14), (20, 18))
_WRIST = 0

# 펴진 (비-엄지) 손가락 수 → 발행 명령.
# 3(MOVE_OBJECT)은 검지 원-모션(CircleDetector)이 담당하므로 정적 매핑에서 제외.
_COUNT_TO_COMMAND = {
    0: 'PAUSE',        # ✊ 주먹: 추종 일시정지
    1: 'START',        # ☝️ 검지: 추종 시작(정지 시)
    2: 'HOME',         # ✌️ 브이: 추종 종료 + 홈복귀
    4: 'GREET',        # ✋ 보자기(네 손가락): 시간대별 인사
}


def _dist_sq(a, b):
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def count_extended_fingers(landmarks):
    """손목에서 tip 이 pip 보다 멀면 편 것으로 보고 편 손가락 수를 센다."""
    wrist = landmarks[_WRIST]
    return sum(
        1 for tip, pip in _FINGER_JOINTS
        if _dist_sq(landmarks[tip], wrist) > _dist_sq(landmarks[pip], wrist)
    )


def classify_gesture(landmarks):
    """21 landmark 를 명령 문자열로 분류한다(미정의 손가락 수면 None)."""
    return _COUNT_TO_COMMAND.get(count_extended_fingers(landmarks))


class GestureDebouncer:
    """같은 명령이 stable_frames 연속이어야 확정. 변화 시 1회만 발행."""

    def __init__(self, stable_frames):
        self._stable_frames = stable_frames
        self._candidate = None
        self._count = 0
        self._confirmed = None

    def update(self, command):
        """프레임마다 호출해 새로 확정된 명령(없으면 None)을 돌려준다."""
        if command == self._candidate:
            self._count += 1
        else:
            self._candidate = command
            self._count = 1
        if (self._count >= self._stable_frames
                and self._candidate != self._confirmed):
            self._confirmed = self._candidate
            return self._confirmed   # None(손 내림)이면 idle 리셋 의미
        return None


class CircleDetector:
    """검지 끝 궤적으로 원-모션(CIRCLE)/정지(STILL)/이동(MOVING)을 구분."""

    def __init__(self, min_angle_deg=300.0, min_radius=0.05,
                 still_radius=0.03, max_points=30):
        self._min_angle = math.radians(min_angle_deg)
        self._min_radius = min_radius
        self._still_radius = still_radius
        self._max_points = max_points
        self._pts = []

    def update(self, point):
        """검지끝 (x, y)(없으면 None) 한 프레임 입력 → 상태 문자열 반환."""
        if point is None:
            self._pts = []
            return 'IDLE'
        self._pts.append(point)
        if len(self._pts) > self._max_points:
            self._pts.pop(0)
        cx = sum(p[0] for p in self._pts) / len(self._pts)
        cy = sum(p[1] for p in self._pts) / len(self._pts)
        max_r = max(math.hypot(p[0] - cx, p[1] - cy) for p in self._pts)
        total = 0.0
        for a, b in zip(self._pts, self._pts[1:]):
            d = (math.atan2(b[1] - cy, b[0] - cx)
                 - math.atan2(a[1] - cy, a[0] - cx))
            total += (d + math.pi) % (2 * math.pi) - math.pi
        if abs(total) >= self._min_angle and max_r >= self._min_radius:
            self._pts = []
            return 'CIRCLE'
        if max_r < self._still_radius:
            return 'STILL'
        return 'MOVING'
