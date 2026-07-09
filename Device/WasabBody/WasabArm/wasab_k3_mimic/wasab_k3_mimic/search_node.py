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
"""선생님 얼굴 추종 + 마지막 위치 기준 expanding 재탐색 RPi 노드 (K3)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from wasab_k3_mimic.arm_driver import ArmDriver
from wasab_k3_mimic.servo import servo_step
from wasab_k3_mimic.sweep import expanding_offsets


def _connect(port, baud):
    """실기 로봇 연결 + 서보 전원 ON. import 는 함수 내부에서(의존성 격리)."""
    import time

    from pymycobot.mycobot280 import MyCobot280
    mc = MyCobot280(port, baud)
    time.sleep(0.5)
    mc.power_on()
    time.sleep(0.3)
    return mc


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


class SearchNode(Node):
    """얼굴 있으면 TRACK(servo), 없으면 마지막 위치 중심 expanding 재탐색."""

    def __init__(self, mc_factory=_connect):
        super().__init__('arm_search')
        self.declare_parameter('face_topic', '/wasab/k3/face')
        self.declare_parameter('face_timeout', 2.0)
        self.declare_parameter('face_conf_threshold', 0.3)
        self.declare_parameter('track_rate_hz', 10.0)
        self.declare_parameter('search_dwell_sec', 1.0)
        self.declare_parameter('kx', 8.0)
        self.declare_parameter('ky', 6.0)
        self.declare_parameter('yaw_limit', 90.0)
        self.declare_parameter('pitch_limit', 50.0)
        self.declare_parameter('yaw_joint', 0)
        self.declare_parameter('pitch_joint', 3)
        self.declare_parameter('search_yaw_step', 10.0)
        self.declare_parameter('search_pitch_step', 8.0)
        self.declare_parameter('home', [0.0, 0.0, 0.0, -15.0, 0.0, -45.0])
        self.declare_parameter('track_speed', 25)
        self.declare_parameter('search_speed', 55)
        self.declare_parameter('port', '/dev/ttyJETCOBOT')
        self.declare_parameter('baud', 1000000)

        self._face_timeout = self.get_parameter('face_timeout').value
        self._face_conf = self.get_parameter('face_conf_threshold').value
        self._search_dwell = self.get_parameter('search_dwell_sec').value
        self._kx = self.get_parameter('kx').value
        self._ky = self.get_parameter('ky').value
        self._yaw_limit = self.get_parameter('yaw_limit').value
        self._pitch_limit = self.get_parameter('pitch_limit').value
        self._yaw_joint = self.get_parameter('yaw_joint').value
        self._pitch_joint = self.get_parameter('pitch_joint').value
        self._home = list(self.get_parameter('home').value)

        # 마지막 위치 중심에서 가까운 곳부터 넓혀가는 재탐색 offset
        self._search_offsets = expanding_offsets(
            self.get_parameter('search_yaw_step').value,
            self.get_parameter('search_pitch_step').value,
            self._yaw_limit,
            self._pitch_limit,
        )
        self._search_speed = self.get_parameter('search_speed').value
        self._driver = ArmDriver(
            mc_factory(
                self.get_parameter('port').value,
                self.get_parameter('baud').value,
            ),
            default_speed=self.get_parameter('track_speed').value,  # TRACK 기본
        )

        # 실시간 추적: 현재 명령 중인 관절 절대각(마지막 검출 위치 포함)
        self._yaw = self._home[self._yaw_joint]
        self._pitch = self._home[self._pitch_joint]
        self._last_face = None
        self._face = (0.5, 0.5, 0.0)
        # 재탐색 상태
        self._searching = False
        self._search_center = (self._yaw, self._pitch)
        self._search_idx = 0
        self._last_sweep_step = None

        self.create_subscription(
            Float64MultiArray, self.get_parameter('face_topic').value,
            self._on_face, 10)

        self._driver.move_joints(self._home)   # 시작 home 정렬
        self.create_timer(
            1.0 / self.get_parameter('track_rate_hz').value, self._tick)

    def _on_face(self, msg):
        """선생님 얼굴 중심 [cx, cy, conf] 수신."""
        if len(msg.data) >= 3:
            self._last_face = self.get_clock().now()
            self._face = (msg.data[0], msg.data[1], msg.data[2])

    def _fresh(self, stamp, timeout):
        """주어진 수신 시각이 timeout 초 이내인지."""
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 < timeout

    def _has_face(self):
        """최근 선생님 얼굴(conf 충분)을 받았는지."""
        return (self._fresh(self._last_face, self._face_timeout)
                and self._face[2] >= self._face_conf)

    def _command(self):
        """현재 yaw/pitch 절대각으로 6관절 명령을 만든다."""
        command = list(self._home)
        command[self._yaw_joint] = self._yaw
        command[self._pitch_joint] = self._pitch
        return command

    def _tick(self):
        if self._has_face():                                # TRACK
            fx, fy, _ = self._face
            self._yaw, self._pitch = servo_step(
                self._yaw, self._pitch, fx, fy,
                self._kx, self._ky, self._yaw_limit, self._pitch_limit)
            self._searching = False
            self.get_logger().info(
                'TRACK 선생님 얼굴 추종', throttle_duration_sec=2.0)
            self._driver.move_joints(self._command())
            return

        # SEARCH: 마지막 검출 위치 중심에서 가까운 곳부터 expanding
        if not self._searching:
            self._search_center = (self._yaw, self._pitch)   # 마지막 각도 기억
            self._search_idx = 0
            self._searching = True
            self._last_sweep_step = None

        now = self.get_clock().now()
        if (self._last_sweep_step is None
                or (now - self._last_sweep_step).nanoseconds / 1e9
                >= self._search_dwell):
            off_yaw, off_pitch = self._search_offsets[
                self._search_idx % len(self._search_offsets)]
            self._search_idx += 1
            self._yaw = _clamp(
                self._search_center[0] + off_yaw,
                -self._yaw_limit, self._yaw_limit)
            self._pitch = _clamp(
                self._search_center[1] + off_pitch,
                -self._pitch_limit, self._pitch_limit)
            self._last_sweep_step = now
            self.get_logger().info(
                'SEARCH 재탐색(마지막 위치 중심)', throttle_duration_sec=2.0)
            self._driver.move_joints(self._command(), self._search_speed)


def main(args=None):
    """ROS2 엔트리포인트. 노드를 생성하고 spin 한다."""
    rclpy.init(args=args)
    node = SearchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
