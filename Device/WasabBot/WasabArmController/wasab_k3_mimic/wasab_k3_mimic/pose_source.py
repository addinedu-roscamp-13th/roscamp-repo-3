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

"""MediaPipe Hands 경계 래퍼. detector 는 의존성 주입(테스트는 fake)."""


class HandObservation:
    """프레임에서 추출한 손목 키포인트(정규화 좌표 + 신뢰도)."""

    def __init__(self, x, y, confidence):
        self.x = x
        self.y = y
        self.confidence = confidence


class PoseSource:
    """프레임 -> 첫 손의 지정 landmark(기본 0=손목) HandObservation 또는 None."""

    def __init__(self, detector, landmark_index=0):
        self._detector = detector
        self._index = landmark_index

    def observe(self, frame):
        """프레임에서 손목 키포인트를 추출한다. 미검출 시 None."""
        results = self._detector.process(frame)
        hands = getattr(results, 'multi_hand_landmarks', None)
        if not hands:
            return None
        lm = hands[0].landmark[self._index]
        confidence = 1.0
        handed = getattr(results, 'multi_handedness', None)
        if handed:
            confidence = handed[0].classification[0].score
        return HandObservation(lm.x, lm.y, confidence)


def make_detector():
    """실기 MediaPipe Hands 생성. import 는 함수 내부에서(의존성 격리)."""
    import mediapipe as mp
    return mp.solutions.hands.Hands(max_num_hands=1)
