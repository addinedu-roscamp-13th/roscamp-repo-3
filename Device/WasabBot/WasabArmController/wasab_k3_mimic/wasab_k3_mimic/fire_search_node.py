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
"""화재(불꽃) 추종 + 마지막 위치 기준 expanding 재탐색 RPi 노드 (K3)."""
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from wasab_k3_mimic.arm_driver import ArmDriver
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


class FireSearchNode(Node):
    """불꽃 있으면 TRACK(servo), 없으면 마지막 위치 중심 expanding 재탐색."""

    def __init__(self, mc_factory=_connect):
        super().__init__('fire_arm_search')
        self.declare_parameter('face_topic', '/wasab/k3/fire')
        self.declare_parameter('face_timeout', 1.0)
        self.declare_parameter('face_conf_threshold', 0.3)
        self.declare_parameter('track_rate_hz', 10.0)
        self.declare_parameter('search_dwell_sec', 1.0)
        self.declare_parameter('kx', -4.0)
        self.declare_parameter('ky', 3.0)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('yaw_limit', 90.0)
        self.declare_parameter('pitch_limit', 50.0)
        self.declare_parameter('yaw_joint', 0)
        self.declare_parameter('pitch_joint', 3)
        self.declare_parameter('search_yaw_step', 14.0)
        self.declare_parameter('search_pitch_step', 6.0)
        self.declare_parameter('home', [0.0, 0.0, 0.0, -15.0, 0.0, -45.0])
        self.declare_parameter('track_speed', 10)
        self.declare_parameter('search_speed', 15)
        self.declare_parameter('port', '/dev/ttyJETCOBOT')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('fire_suppress_topic', '/wasab/k3/fire_suppress')
        self.declare_parameter('gripper_close_value', 0)   # 2026-07-28 실기 확인: 0=닫힘, 100=열림
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('arm_still_topic', '/wasab/k3/arm_still')
        self.declare_parameter('settle_delay_sec', 0.3)

        self._face_timeout = self.get_parameter('face_timeout').value
        self._face_conf = self.get_parameter('face_conf_threshold').value
        self._search_dwell = self.get_parameter('search_dwell_sec').value
        self._kx = self.get_parameter('kx').value
        self._ky = self.get_parameter('ky').value
        self._deadzone = self.get_parameter('deadzone').value
        self._yaw_limit = self.get_parameter('yaw_limit').value
        self._pitch_limit = self.get_parameter('pitch_limit').value
        self._yaw_joint = self.get_parameter('yaw_joint').value
        self._pitch_joint = self.get_parameter('pitch_joint').value
        self._home = list(self.get_parameter('home').value)
        self._settle_delay = self.get_parameter('settle_delay_sec').value

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
        self._last_move_time = self.get_clock().now()

        self.create_subscription(
            Float64MultiArray, self.get_parameter('face_topic').value,
            self._on_face, 10)
        self.create_subscription(
            String, self.get_parameter('fire_suppress_topic').value,
            self._on_suppress, 10)
        self._still_pub = self.create_publisher(
            Bool, self.get_parameter('arm_still_topic').value, 10)

        self._driver.move_joints(self._home)   # 시작 home 정렬
        self._last_move_time = self.get_clock().now()
        self.create_timer(
            1.0 / self.get_parameter('track_rate_hz').value, self._tick)

    def _on_face(self, msg):
        """불꽃 중심 [cx, cy, conf] 수신 시, 새 데이터에 한해 즉시 1회 보정."""
        if len(msg.data) < 3:
            return
        self._last_face = self.get_clock().now()
        self._face = (msg.data[0], msg.data[1], msg.data[2])
        fx, fy, conf = self._face
        if conf < self._face_conf:
            return
        # 2026-07-28: 감지되면 센터링(servo_step) 없이 그 자리에서 바로 정지.
        # 이유 1) 계속 재보정하면 팔이 찔끔찔끔 계속 움직여 보임(하드웨어 명령 처리 주기 때문에
        #        매끄럽게 안 되고 뚝뚝 끊겨 보임). 2) 계속 움직이면 arm_still이 계속 리셋되어
        #        fire_detect_arm.py 쪽 감지 버퍼가 못 차서 화재 확정이 계속 밀림.
        # 2026-07-28: ArmDriver에 stop() 메서드 자체가 없어서(move_to/move_joints/gripper뿐) 여기서
        # AttributeError가 나고 있었음 — SEARCH 중 새 move_joints를 더 안 부르는 것만으로 "정지"는
        # 이미 달성되므로(위 주석 참고) 별도 정지 호출 없이 그냥 제거.
        self.get_logger().info(
            f'[TRACK-DBG] fx={fx:.3f} fy={fy:.3f} conf={conf:.2f} '
            f'ex={fx-0.5:+.3f} ey={fy-0.5:+.3f} — 센터링 안 함, 제자리 정지')
        self._searching = False
        self.get_logger().info(
            'TRACK 불꽃 감지(정지 유지)', throttle_duration_sec=2.0)

    def _on_suppress(self, msg):
        """fire_detect_node.py 가 선생님 "Yes" 응답을 중계하면 그리퍼로 진압 동작."""
        if msg.data == 'close':
            self.get_logger().info('진압 명령 수신 — 그리퍼 여닫기(x2) 후 닫힘 유지')
            value = self.get_parameter('gripper_close_value').value
            speed = self.get_parameter('gripper_speed').value
            settle = 0.4
            for _ in range(2):
                self._driver.close_gripper(value, speed)
                time.sleep(settle)
                self._driver.open_gripper(speed)
                time.sleep(settle)
            self._driver.close_gripper(value, speed)
        elif msg.data == 'open':
            self._driver.open_gripper(self.get_parameter('gripper_speed').value)

    def _fresh(self, stamp, timeout):
        """주어진 수신 시각이 timeout 초 이내인지."""
        if stamp is None:
            return False
        return (self.get_clock().now() - stamp).nanoseconds / 1e9 < timeout

    def _has_face(self):
        """최근 불꽃(신뢰도 충분)을 받았는지."""
        return (self._fresh(self._last_face, self._face_timeout)
                and self._face[2] >= self._face_conf)

    def _command(self):
        """현재 yaw/pitch 절대각으로 6관절 명령을 만든다."""
        command = list(self._home)
        command[self._yaw_joint] = self._yaw
        command[self._pitch_joint] = self._pitch
        return command

    def _tick(self):
        # 마지막 관절 이동 이후 settle_delay_sec가 지났으면 "카메라 정지" 신호 발행.
        # fire_detect_node.py(PC)가 이걸 구독해서, 팔이 움직인 직후(화면 전체가 바뀌는 구간)의
        # 프레임을 화재 판정용 버퍼에서 제외하는 데 쓴다.
        still = ((self.get_clock().now() - self._last_move_time).nanoseconds / 1e9
                 >= self._settle_delay)
        self._still_pub.publish(Bool(data=still))

        if self._has_face():                # TRACK (보정은 _on_face 에서 이미 처리됨)
            return

        # SEARCH: 마지막 검출 위치 중심에서 가까운 곳부터 expanding
        if not self._searching:
            self._search_center = (self._yaw, self._pitch)   # 마지막 각도 기억
            self._search_idx = 0
            self._searching = True
            self._last_sweep_step = None
            # 재탐색 시작 시점의 중심 각도/마지막 감지 좌표 로그
            fx, fy, conf = self._face
            self.get_logger().info(
                f'[SEARCH-DBG] center_yaw={self._search_center[0]:.1f} '
                f'center_pitch={self._search_center[1]:.1f} '
                f'last_face=({fx:.3f},{fy:.3f},{conf:.2f})')

        now = self.get_clock().now()
        if (self._last_sweep_step is None
                or (now - self._last_sweep_step).nanoseconds / 1e9
                >= self._search_dwell):
            off_yaw, off_pitch = self._search_offsets[
                self._search_idx % len(self._search_offsets)]
            self._search_idx += 1
            prev_yaw, prev_pitch = self._yaw, self._pitch
            self._yaw = _clamp(
                self._search_center[0] + off_yaw,
                -self._yaw_limit, self._yaw_limit)
            self._pitch = _clamp(
                self._search_center[1] + off_pitch,
                -self._pitch_limit, self._pitch_limit)
            self._last_sweep_step = now
            # 재탐색 스텝별 offset/각도 로그
            label = '탐색' if self._last_face is None else '재탐색'
            self.get_logger().info(
                f'SEARCH {label} idx={self._search_idx-1} '
                f'off=({off_yaw:.1f},{off_pitch:.1f}) '
                f'yaw={self._yaw:.1f} pitch={self._pitch:.1f}')
            # 2026-07-28 밤: off=(0,0)처럼 실제 각도가 안 바뀌는 스텝에서도 move_joints를 불러서
            # arm_still이 매번 리셋되고, fire_detect_arm.py 모션버퍼가 씻겨나가 SEARCH↔TRACK이
            # 계속 반복되는 원인이었음 — 실제로 각도가 바뀔 때만 이동/리셋.
            if (self._yaw, self._pitch) != (prev_yaw, prev_pitch):
                self._driver.move_joints(self._command(), self._search_speed)
                self._last_move_time = now


def main(args=None):
    """ROS2 엔트리포인트. 노드를 생성하고 spin 한다."""
    rclpy.init(args=args)
    node = FireSearchNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
