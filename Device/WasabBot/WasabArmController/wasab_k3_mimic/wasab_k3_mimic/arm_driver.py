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

"""pymycobot send_coords boundary wrapper. mc is dependency injection (test is fake)."""


class ArmDriver:
    """Thin wrapper sending Cartesian coordinates to robot arm."""

    def __init__(self, mc, default_speed=30):
        self._mc = mc
        self._default_speed = default_speed

    def move_to(self, coords, speed=None, mode=1):
        """Send coords ([x, y, z, rx, ry, rz]) via send_coords."""
        self._mc.send_coords(
            list(coords), self._default_speed if speed is None else speed, mode)

    def move_joints(self, angles, speed=None):
        """Send joint angles ([j1..j6] deg) via send_angles (non-blocking)."""
        # _async=True: 펌웨어 이동완료 피드백을 안 기다림. 동기 기본값은
        # 호출당 ~1.5s 블로킹이라 제어 루프가 0.6Hz 로 떨어진다.
        self._mc.send_angles(
            list(angles), self._default_speed if speed is None else speed,
            _async=True)

    def open_gripper(self, speed=50):
        """그리퍼 열기."""
        self._mc.set_gripper_state(0, speed)

    def close_gripper(self, value=0, speed=50):
        """그리퍼 닫기(화재 진압 동작으로 사용)."""
        self._mc.set_gripper_value(value, speed)

    def stop(self):
        """현재 실행 중인 로봇팔 움직임을 즉시 중단한다."""
        return self._mc.stop()
