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
"""좌표 토픽을 구독해 실기 팔로 send_coords 하는 브리지 노드 (K3 분산, RPi 측)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from wasab_k3_mimic.arm_driver import ArmDriver


def _connect(port, baud):
    """실기 로봇 연결 + 서보 전원 ON. import 는 함수 내부에서(의존성 격리)."""
    import time

    from pymycobot.mycobot280 import MyCobot280
    mc = MyCobot280(port, baud)
    time.sleep(0.5)
    mc.power_on()
    time.sleep(0.3)
    return mc


class ArmBridgeNode(Node):
    """target_coords 토픽을 구독해 ArmDriver 로 팔을 움직인다."""

    def __init__(self, mc_factory=_connect):
        super().__init__('arm_bridge')
        self.declare_parameter('coords_topic', '/wasab/k3/target_coords')
        self.declare_parameter('port', '/dev/ttyJETCOBOT')
        self.declare_parameter('baud', 1000000)
        self.declare_parameter('speed', 30)
        self._driver = ArmDriver(
            mc_factory(
                self.get_parameter('port').value,
                self.get_parameter('baud').value,
            ),
            default_speed=self.get_parameter('speed').value,
        )
        self._sub = self.create_subscription(
            Float64MultiArray,
            self.get_parameter('coords_topic').value,
            self._on_coords,
            10,
        )

    def _on_coords(self, msg):
        """수신한 좌표를 팔로 전송. 6원소가 아니면 안전을 위해 무시."""
        coords = list(msg.data)
        if len(coords) != 6:
            self.get_logger().warning(
                f'ignored coords of length {len(coords)}')
            return
        self._driver.move_to(coords)


def main(args=None):
    """ROS2 엔트리포인트. 노드를 생성하고 spin 한다."""
    rclpy.init(args=args)
    node = ArmBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
