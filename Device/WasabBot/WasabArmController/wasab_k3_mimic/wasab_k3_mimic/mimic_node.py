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
"""손목 (x, y, conf) 를 발행하는 노트북 인지 노드 (K3 분산, 게이트는 search_node)."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

from wasab_k3_mimic.pose_source import make_detector, PoseSource


def _open_camera(source):
    """실기 카메라 연결(스트림 URL 또는 인덱스). import 는 함수 내부에서."""
    import cv2
    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 최신 프레임만(딜레이 누적 방지)
    return cap


class MimicNode(Node):
    """카메라 손목 추종 루프. 손목 (x, y, conf) 를 토픽으로 발행한다."""

    def __init__(self, camera_factory=_open_camera,
                 detector_factory=make_detector, pub_factory=None):
        super().__init__('motion_mimic')
        self.declare_parameter('camera_url', 'http://192.168.0.86:8090/stream')
        self.declare_parameter('wrist_topic', '/wasab/k3/wrist')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('landmark_index', 0)
        self.declare_parameter('show_view', False)
        self.declare_parameter('mirror', True)

        self._show_view = self.get_parameter('show_view').value
        self._mirror = self.get_parameter('mirror').value
        self._pose = PoseSource(
            detector_factory(),
            self.get_parameter('landmark_index').value,
        )
        topic = self.get_parameter('wrist_topic').value
        self._pub = (pub_factory(topic) if pub_factory is not None
                     else self.create_publisher(Float64MultiArray, topic, 10))
        self._camera = camera_factory(self.get_parameter('camera_url').value)
        period = 1.0 / self.get_parameter('rate_hz').value
        self.create_timer(period, self._tick)

    def step(self, frame):
        """프레임 하나 처리. 손목 검출 시 [x, y, conf] 발행, 미검출 시 None."""
        if frame is None:
            return None
        return self._process(self._pose.observe(frame))

    def _process(self, obs):
        """손목 관측을 [x, y, conf] 로 발행(게이팅은 search_node). 발행값 또는 None."""
        if obs is None:
            self.get_logger().debug(
                'no hand detected', throttle_duration_sec=2.0)
            return None
        data = [float(obs.x), float(obs.y), float(obs.confidence)]
        msg = Float64MultiArray()
        msg.data = data
        self._pub.publish(msg)
        return data

    def _tick(self):
        ret, frame = self._camera.read()
        if not ret:
            self.get_logger().warning(
                'camera read failed', throttle_duration_sec=5.0)
            return
        import cv2
        if self._mirror:
            frame = cv2.flip(frame, 1)
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        obs = self._pose.observe(rgb)
        sent = self._process(obs)
        if self._show_view:
            self._draw(frame, obs, sent)

    def _draw(self, frame, obs, sent):
        """디버그 뷰: 손목 위치·신뢰도·발행상태를 프레임에 표시."""
        import cv2
        h, w = frame.shape[:2]
        if obs is None:
            cv2.putText(frame, 'NO HAND', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            px, py = int(obs.x * w), int(obs.y * h)
            color = (0, 200, 0) if sent is not None else (0, 0, 255)
            cv2.circle(frame, (px, py), 10, color, 2)
            cv2.putText(frame, f'wrist conf={obs.confidence:.2f} PUBLISH',
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow('K3 mimic (laptop)', frame)
        cv2.waitKey(1)


def main(args=None):
    """ROS2 엔트리포인트. 노드를 생성하고 spin 한다."""
    rclpy.init(args=args)
    node = MimicNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
