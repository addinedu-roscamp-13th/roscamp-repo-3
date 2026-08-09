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

"""사람 손목 정규화 좌표를 로봇 Cartesian 좌표로 매핑하는 순수로직."""


def _clamp(value, low, high):
    return max(low, min(high, value))


class Workspace:
    """로봇 작업공간 박스와 고정 자세. 박스 범위는 config 정의(reach는 T7 튜닝)."""

    def __init__(self, x_depth, y_center, y_span, z_center, z_span,
                 orientation):
        self.x_depth = x_depth
        self.y_center = y_center
        self.y_span = y_span
        self.z_center = z_center
        self.z_span = z_span
        self.orientation = list(orientation)


def map_to_coords(nx, ny, ws):
    """정규화 손목 (nx, ny) 를 [x, y, z, rx, ry, rz] 로 매핑·클램프."""
    y = ws.y_center + (nx - 0.5) * ws.y_span
    z = ws.z_center + (0.5 - ny) * ws.z_span
    y = _clamp(y, ws.y_center - ws.y_span / 2, ws.y_center + ws.y_span / 2)
    z = _clamp(z, ws.z_center - ws.z_span / 2, ws.z_center + ws.z_span / 2)
    rx, ry, rz = ws.orientation
    return [ws.x_depth, y, z, rx, ry, rz]
