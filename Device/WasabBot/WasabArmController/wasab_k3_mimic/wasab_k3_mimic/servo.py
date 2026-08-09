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
"""비주얼 서보 순수로직: 이미지 중심오차로 yaw/pitch를 비례보정 (eye-in-hand)."""


def _clamp(value, low, high):
    return max(low, min(high, value))


def servo_step(yaw, pitch, x, y, kx, ky, yaw_limit, pitch_limit, deadzone=0.0):
    """손목 (x, y)를 화면 중앙으로 비례보정(부호는 게인으로). deadzone 미만 오차인 축은 그대로 유지."""
    ex = x - 0.5
    ey = y - 0.5
    new_yaw = yaw if abs(ex) < deadzone else _clamp(yaw - kx * ex, -yaw_limit, yaw_limit)
    new_pitch = pitch if abs(ey) < deadzone else _clamp(pitch - ky * ey, -pitch_limit, pitch_limit)
    return new_yaw, new_pitch
