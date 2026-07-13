import sys
sys.path.insert(0, "/opt/ros/jazzy/lib/python3.12/site-packages")
sys.path.insert(0, "/home/pinky/.local/lib/python3.12/site-packages")

import os
import signal
import subprocess

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

# ── 라이다 드라이버(sllidar_ros2) 서브프로세스 기동 ────────────────────────────
# 주의: PC 저장소의 bringup_robot.launch.xml 원본은 serial_port 기본값이 ttyAMA0로
# 되어 있지만, 이 로봇에 실제 설치된 사본(~/pinky_pro/install/.../bringup_robot.launch.xml)은
# ttyS0로 커스터마이징되어 있음 — 로봇 실기로 직접 확인(2026-07-10, 오전 캘리브레이션
# 세션에서 정상 동작 검증된 값). PC 원본과 로봇 실치본이 다르다는 점 주의.
# 추종 로봇은 그 launch를 안 쓰므로 여기서 직접 띄운다
# (wasab_robot_agent의 서브프로세스 launch 제어와 같은 패턴).
# shell=True로 bash를 거쳐야 ros2 CLI가 PATH에서 정상적으로 찾아짐
# (subprocess에 인자 리스트를 직접 넘기면 ros2를 못 찾는 것 실기로 확인, 2026-07-10).
# sllidar_ros2는 /opt/ros/jazzy가 아니라 ~/pinky_pro/install 워크스페이스에 설치돼
# 있어서, local_setup.bash도 같이 소싱해야 패키지를 찾음(실기로 원인 확인, 2026-07-10).
LIDAR_LAUNCH_CMD = (
    "source /opt/ros/jazzy/setup.bash && "
    "source /home/pinky/pinky_pro/install/local_setup.bash && "
    "ros2 launch sllidar_ros2 sllidar_c1_launch.py "
    "serial_port:=/dev/ttyS0 frame_id:=rplidar_link "
    "inverted:=false angle_compensate:=true scan_mode:=DenseBoost"
)
LIDAR_SHUTDOWN_TIMEOUT = 5.0

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

        # 라이다 드라이버 서브프로세스 (거리재급/회피용 /scan 발행)
        self._lidar_proc = subprocess.Popen(
            LIDAR_LAUNCH_CMD, shell=True, executable="/bin/bash", start_new_session=True)

        self.get_logger().info("PinkyNode 시작 — 카메라 & 모터 & LCD & LED & 라이다 준비됨")

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

    def _stop_lidar(self):
        if self._lidar_proc is None:
            return
        try:
            os.killpg(os.getpgid(self._lidar_proc.pid), signal.SIGINT)
            self._lidar_proc.wait(timeout=LIDAR_SHUTDOWN_TIMEOUT)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self._lidar_proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        self._lidar_proc = None

    def destroy_node(self):
        self.motor.stop()
        self.motor.disable_motor()
        self.motor.close()
        self.cam.close()
        self.lcd.close()
        self.led.clear()
        self.led.close()
        self._stop_lidar()
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
