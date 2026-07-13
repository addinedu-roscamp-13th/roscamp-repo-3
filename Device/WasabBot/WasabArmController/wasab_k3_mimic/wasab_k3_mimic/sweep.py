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
"""스윕 탐색 웨이포인트 생성 순수로직 (eye-in-hand 카메라 손 탐색)."""


def _linspace(low, high, n):
    if n <= 1:
        return [(low + high) / 2.0]
    step = (high - low) / (n - 1)
    return [low + step * i for i in range(n)]


def sweep_waypoints(yaw_amp, pitch_amp, yaw_steps, pitch_steps):
    """(yaw, pitch) 스네이크 래스터 생성. ±yaw_amp × ±pitch_amp 대칭."""
    yaws = _linspace(-yaw_amp, yaw_amp, yaw_steps)
    pitches = _linspace(-pitch_amp, pitch_amp, pitch_steps)
    points = []
    for i, pitch in enumerate(pitches):
        row = yaws if i % 2 == 0 else list(reversed(yaws))
        for yaw in row:
            points.append((yaw, pitch))
    return points


def expanding_offsets(yaw_step, pitch_step, yaw_max, pitch_max):
    """중심에서 점점 넓어지는 (yaw, pitch) offset 목록(가까운 ring 부터)."""
    offsets = [(0.0, 0.0)]
    ring = 1
    while yaw_step * ring <= yaw_max or pitch_step * ring <= pitch_max:
        ya = min(yaw_step * ring, yaw_max)
        pa = min(pitch_step * ring, pitch_max)
        offsets.extend([
            (ya, 0.0), (-ya, 0.0), (0.0, pa), (0.0, -pa),
            (ya, pa), (-ya, pa), (ya, -pa), (-ya, -pa),
        ])
        ring += 1
    return offsets


class Sweeper:
    """스윕 웨이포인트를 순환하며 home 자세 기반 6관절 명령을 만든다."""

    def __init__(self, waypoints, home, yaw_joint, pitch_joint):
        self._waypoints = list(waypoints)
        self._home = list(home)
        self._yaw_joint = yaw_joint
        self._pitch_joint = pitch_joint
        self._index = 0

    def next_waypoint(self):
        """다음 (yaw, pitch) 웨이포인트를 반환하고 인덱스를 전진한다."""
        yaw, pitch = self._waypoints[self._index]
        self._index = (self._index + 1) % len(self._waypoints)
        return yaw, pitch

    def next_command(self):
        """다음 웨이포인트를 home 기준 offset 으로 더한 6관절 명령을 반환한다."""
        yaw, pitch = self.next_waypoint()
        command = list(self._home)
        command[self._yaw_joint] = self._home[self._yaw_joint] + yaw
        command[self._pitch_joint] = self._home[self._pitch_joint] + pitch
        return command
