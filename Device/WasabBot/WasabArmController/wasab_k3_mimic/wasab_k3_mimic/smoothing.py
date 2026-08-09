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

"""좌표 타깃 평활(EMA)과 스텝 변위 클램프 순수로직."""


def _clamp(value, low, high):
    return max(low, min(high, value))


class Smoother:
    """축별 EMA + 직전 출력 대비 변위 제한으로 급격한 점프를 억제한다."""

    def __init__(self, alpha, max_step):
        self._alpha = alpha
        self._max_step = max_step
        self._state = None

    def update(self, target):
        """타깃으로 한 스텝 평활. 첫 호출은 타깃을 그대로 통과시킨다."""
        if self._state is None:
            self._state = list(target)
            return list(self._state)
        out = []
        for prev, tgt in zip(self._state, target):
            ema = prev + self._alpha * (tgt - prev)
            step = _clamp(ema - prev, -self._max_step, self._max_step)
            out.append(prev + step)
        self._state = out
        return list(out)
