import sys
sys.path.insert(0, "/opt/ros/jazzy/lib/python3.12/site-packages")
sys.path.insert(0, "/home/pinky/.local/lib/python3.12/site-packages")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
import numpy as np
from PIL import Image as PILImage
from pinkylib import Camera, Motor, LED
from pinky_lcd import LCD

PUBLISH_HZ   = 10
JPEG_QUALITY = 80
MOTOR_MIN    = -100
MOTOR_MAX    = 100
LCD_EVERY    = 3   # N 프레임마다 LCD 갱신

# ── 제스처 연동 LED (PC ai_node.py → /wasab/led_state) ───────────────────────
LED_STATE_TOPIC = "/wasab/led_state"
LED_RED   = (255, 0, 0)
LED_GREEN = (0, 255, 0)
LED_PAUSE_SEC = 3.0   # PAUSED 진입 시 빨강 유지 시간, 이후 꺼짐


class PinkyNode(Node):
    def __init__(self):
        super().__init__("pinky_node")

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # 카메라 publisher
        self.pub = self.create_publisher(CompressedImage, "/camera/image/compressed", qos)
        self.cam = Camera()
        self.cam.start()
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.timer_cb)
        self.frame_count = 0

        # 모터 (cmd_vel subscriber)
        self.motor = Motor()
        self.motor.enable_motor()
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10)

        # LCD
        self.lcd = LCD()

        # LED (제스처 상태 표시: PAUSED=빨강 3초 후 꺼짐, FOLLOWING=초록 유지)
        self.led = LED()
        self._led_off_timer = None
        self.create_subscription(String, LED_STATE_TOPIC, self.led_state_cb, 10)

        self.get_logger().info("PinkyNode 시작 — 카메라 & 모터 & LCD & LED 준비됨")

    def timer_cb(self):
        frame = self.cam.get_frame()
        if frame is None:
            return
        ok, encoded = cv2.imencode(".jpg", frame,
                                   [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = encoded.tobytes()
        self.pub.publish(msg)
        self.frame_count += 1
        if self.frame_count % (PUBLISH_HZ * 5) == 0:
            self.get_logger().info(f"카메라 발행 중... ({self.frame_count} 프레임)")

        # LCD 갱신
        if self.frame_count % LCD_EVERY == 0:
            rgb = cv2.cvtColor(cv2.resize(frame, (320, 240)), cv2.COLOR_BGR2RGB)
            self.lcd.img_show(PILImage.fromarray(rgb))

    def cmd_vel_cb(self, msg: Twist):
        linear  = msg.linear.x   # 전진 속도 (모터 단위)
        angular = msg.angular.z  # 조향 보정값

        left  = int(np.clip(linear - angular, MOTOR_MIN, MOTOR_MAX))
        right = int(np.clip(linear + angular, MOTOR_MIN, MOTOR_MAX))
        self.motor.move(left, right)

    def led_state_cb(self, msg: String):
        if self._led_off_timer is not None:
            self._led_off_timer.cancel()
            self._led_off_timer = None

        if msg.data == "PAUSED":
            self.led.fill(LED_RED)
            self._led_off_timer = self.create_timer(LED_PAUSE_SEC, self._led_off_once)
        elif msg.data == "FOLLOWING":
            self.led.fill(LED_GREEN)
        elif msg.data == "OFF":
            self.led.clear()

    def _led_off_once(self):
        self.led.clear()
        if self._led_off_timer is not None:
            self._led_off_timer.cancel()
            self._led_off_timer = None

    def destroy_node(self):
        self.motor.stop()
        self.motor.disable_motor()
        self.motor.close()
        self.cam.close()
        self.lcd.close()
        self.led.clear()
        self.led.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = PinkyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
