import os
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from ultralytics import YOLO

try:
    from pinky_yolo.face_recognizer import FaceRecognizer
except ImportError:
    from face_recognizer import FaceRecognizer

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
_PKG_DIR    = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FACE_DB_DIR = os.path.join(_PKG_DIR, "face_db")
YOLO_MODEL  = "yolov8n.pt"

# ── 얼굴 인식 (InsightFace SCRFD + ArcFace) ───────────────────────────────────
FACE_TOLERANCE = 0.40   # 먼 거리에서도 인식되도록 완화 (기존 0.45) — 오인식 늘어나면 다시 올릴 것
FACE_MIN_SIZE  = 40

# ── PD 제어 ──────────────────────────────────────────────────────────────────
MOTOR_BASE  = 40.0
MOTOR_MIN   =  5.0
MOTOR_MAX   = 80.0
KP          = 0.05
KD          = 0.2
DEADZONE    = 20
MAX_ANGULAR = 11.0

FRAME_W  = 640
CENTER_X = FRAME_W // 2

# ── 거리 임계값 ───────────────────────────────────────────────────────────────
DIST_STOP   = 150   # 얼굴 bbox 높이 기준 ~10cm 이상에서 정지하도록 하향 (기존 250)
DIST_FOLLOW = 300   # 이보다 멀면 전진

# ── 두리번거리기 탐색 ─────────────────────────────────────────────────────────
SEARCH_ANGULAR    = 10.0
SEARCH_SWING_TIME = 1.5
LOST_TIMEOUT      = 9.0

# ── YOLO 스킵 (InsightFace는 스레드로 분리되어 스킵 불필요) ──────────────────
YOLO_SKIP  = 3
SEEN_GRACE = 0.5

# ── 제스처 연동 (JetCobot perception_node.py → domain 51 브리지) ─────────────
GESTURE_CMD_TOPIC = "/wasab/gesture_cmd"   # 구독: PAUSE/START
LED_STATE_TOPIC    = "/wasab/led_state"    # 발행: raspi pinky_node.py가 LED 제어


def select_teacher(face_db_dir: str) -> str:
    known_dir = os.path.join(face_db_dir, "known")
    if not os.path.exists(known_dir):
        raise RuntimeError(
            f"face_db/known 폴더가 없습니다: {known_dir}\n"
            f"  → mkdir -p {known_dir}/<이름>  후 사진을 넣어주세요."
        )

    names = [
        d for d in sorted(os.listdir(known_dir))
        if os.path.isdir(os.path.join(known_dir, d))
    ]
    if not names:
        raise RuntimeError(
            f"등록된 얼굴이 없습니다: {known_dir}\n"
            f"  → {known_dir}/<이름>/*.jpg 형식으로 사진을 저장하세요."
        )

    print("\n등록된 프로필:")
    for i, name in enumerate(names, 1):
        print(f"  {i}. {name}")

    while True:
        try:
            choice = int(input("\n추종할 선생님 번호 선택: "))
            if 1 <= choice <= len(names):
                selected = names[choice - 1]
                print(f"선택됨: {selected}\n")
                return selected
        except ValueError:
            pass
        print(f"1~{len(names)} 사이 숫자를 입력하세요.")


def decode_compressed(msg: CompressedImage) -> np.ndarray:
    arr = np.frombuffer(msg.data, dtype=np.uint8)
    return cv2.imdecode(arr, cv2.IMREAD_COLOR)


def detect_persons(model, frame, conf=0.45):
    results = model(frame, verbose=False, conf=conf, classes=[0], imgsz=320)
    boxes = []
    for r in results:
        for b in r.boxes:
            x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
            boxes.append((x1, y1, x2, y2, float(b.conf)))
    return boxes


class AiNode(Node):
    def __init__(self, teacher: str):
        super().__init__("ai_node")
        self.teacher = teacher

        # InsightFace
        self.recognizer = FaceRecognizer(
            face_db_dir=FACE_DB_DIR,
            tolerance=FACE_TOLERANCE,
            min_face_size=FACE_MIN_SIZE,
            model_name="buffalo_sc",
            det_size=640,   # 320→640: 먼 거리 작은 얼굴도 감지/인식되도록 원복 (처리속도는 느려짐)
        )
        if not self.recognizer.is_ready:
            self.get_logger().error("InsightFace 모델 로드 실패")
            raise RuntimeError("InsightFace 로드 실패")
        self.get_logger().info(f"추종 대상: {self.teacher}")

        # YOLO
        self.model = YOLO(YOLO_MODEL)
        self.get_logger().info("YOLO 로드 완료")

        # ROS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.sub = self.create_subscription(
            CompressedImage, "/camera/image/compressed", self.image_cb, qos)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # 제스처 연동: PAUSE/START → e_stop 토글 + LED 상태 발행
        self.led_pub = self.create_publisher(String, LED_STATE_TOPIC, 10)
        self.create_subscription(
            String, GESTURE_CMD_TOPIC, self.gesture_cmd_cb, 10)

        # ── InsightFace 백그라운드 스레드 ────────────────────────────────────
        self._latest_frame  = None          # 메인 → 스레드로 최신 프레임 전달
        self._face_results  = []            # 스레드 → 메인으로 인식 결과 전달
        self._frame_lock    = threading.Lock()
        self._face_lock     = threading.Lock()
        self._running       = True
        self._face_thread   = threading.Thread(target=self._face_worker, daemon=True)
        self._face_thread.start()

        # 상태
        self.frame_count      = 0
        self.last_boxes       = []
        self.last_teacher_box = None
        self.last_seen        = time.time()  # 시작부터 SEARCH 상태로
        self.last_error_dir   = 1.0
        self.prev_error       = 0.0
        self.t_prev           = time.time()
        self.state            = "WAIT"
        self.e_stop           = True   # 시작 시 대기 상태 — START 제스처(또는 Space) 전엔 얼굴 보여도 추종 안 함
        self.motor_base       = MOTOR_BASE
        self._last_led_state  = None   # FOLLOW 진입/이탈 감지용(LED 중복 발행 방지)

        self.get_logger().info("AiNode 시작 — 대기 중 (START 제스처 또는 Space로 추종 시작)")
        self.get_logger().info("[Space] 추종 시작/정지  [+/-] 속도조절  [q] 종료")

    def _set_e_stop(self, value: bool):
        """e_stop 토글. PAUSE만 LED를 직접 켬(3초 빨강) — FOLLOWING 초록은 실제 FOLLOW 상태에서만."""
        self.e_stop = value
        self.get_logger().info("긴급 정지 ON" if value else "긴급 정지 해제")
        if value:
            self.led_pub.publish(String(data="PAUSED"))

    def _sync_led(self):
        """매 프레임 state 변화 감지 → FOLLOW면 초록, 그 외(SEARCH/LOST/TOO_CLOSE)면 꺼짐.
        E-STOP 진입 시의 빨강 3초는 _set_e_stop이 이미 처리하므로 여기선 건드리지 않는다."""
        if self.state == self._last_led_state:
            return
        self._last_led_state = self.state
        if self.state == "FOLLOW":
            self.led_pub.publish(String(data="FOLLOWING"))
        elif self.state != "E-STOP":
            self.led_pub.publish(String(data="OFF"))

    def gesture_cmd_cb(self, msg: String):
        if msg.data == "PAUSE" and not self.e_stop:
            self.get_logger().info("제스처 PAUSE 수신")
            self._set_e_stop(True)
        elif msg.data == "START" and self.e_stop:
            self.get_logger().info("제스처 START 수신")
            self._set_e_stop(False)

    def _face_worker(self):
        """InsightFace를 별도 스레드에서 실행 — 메인 루프를 블로킹하지 않음."""
        while self._running:
            with self._frame_lock:
                frame = self._latest_frame
            if frame is None:
                time.sleep(0.005)
                continue

            results = self.recognizer.identify(frame)

            with self._face_lock:
                self._face_results = results

    def image_cb(self, msg: CompressedImage):
        frame = decode_compressed(msg)
        if frame is None:
            return
        frame = cv2.flip(frame, 1)

        t_now = time.time()
        dt    = max(t_now - self.t_prev, 1e-3)
        self.t_prev = t_now
        self.frame_count += 1

        # 최신 프레임을 스레드에 전달
        with self._frame_lock:
            self._latest_frame = frame.copy()

        # 스레드에서 계산된 얼굴 인식 결과 읽기
        with self._face_lock:
            face_results = list(self._face_results)

        # YOLO 사람 감지 (거리 추정용)
        if self.frame_count % YOLO_SKIP == 0:
            self.last_boxes = detect_persons(self.model, frame)

        # 선생님 얼굴 찾기 (거리 계산은 얼굴 박스로 통일)
        for fr in face_results:
            if fr.name == self.teacher:
                fx1, fy1, fx2, fy2 = fr.bbox
                self.last_teacher_box = (fx1, fy1, fx2, fy2)
                self.last_seen = t_now
                error = (fx1 + fx2) // 2 - CENTER_X
                self.last_error_dir = 1.0 if error >= 0 else -1.0
                break

        # SEEN_GRACE 이내 인식된 박스 사용
        teacher_box = (
            self.last_teacher_box
            if self.last_teacher_box is not None and (t_now - self.last_seen) < SEEN_GRACE
            else None
        )

        # ── 상태 머신 + cmd_vel ───────────────────────────────────────────────
        twist = Twist()

        if self.e_stop:
            self.state = "E-STOP"

        elif teacher_box is not None:
            x1, y1, x2, y2 = teacher_box
            bbox_h = y2 - y1
            error  = (x1 + x2) // 2 - CENTER_X

            if bbox_h >= DIST_STOP:
                self.state = "TOO_CLOSE"
            else:
                self.state = "FOLLOW"
                if abs(error) >= DEADZONE:
                    d_err = (error - self.prev_error) / dt
                    correction = float(np.clip(KP * error + KD * d_err, -MAX_ANGULAR, MAX_ANGULAR))
                else:
                    correction = 0.0
                centering = 1.0 - min(abs(error) / CENTER_X, 1.0)
                twist.linear.x  = float(np.clip(self.motor_base * centering, 0.0, MOTOR_MAX))
                twist.angular.z = correction

            self.prev_error = float(error)

        else:
            elapsed = t_now - self.last_seen
            if elapsed < LOST_TIMEOUT:
                # 두리번거리기: 매 SEARCH_SWING_TIME 초마다 방향 전환
                self.state = "SEARCH"
                swing_idx = int(elapsed / SEARCH_SWING_TIME)
                direction = self.last_error_dir * ((-1.0) ** swing_idx)
                twist.angular.z = float(direction * SEARCH_ANGULAR)
            else:
                # LOST: 느린 속도로 한 방향 계속 회전하며 탐색
                self.state = "LOST"
                twist.angular.z = float(self.last_error_dir * SEARCH_ANGULAR * 0.5)
            self.prev_error = 0.0

        self.pub.publish(twist)
        self._sync_led()

        # ── 시각화 ───────────────────────────────────────────────────────────
        vis = frame.copy()

        for (x1, y1, x2, y2, _) in self.last_boxes:
            cv2.rectangle(vis, (x1, y1), (x2, y2), (255, 140, 0), 1)

        for fr in face_results:
            fx1, fy1, fx2, fy2 = fr.bbox
            is_teacher = (fr.name == self.teacher)
            color     = (0, 220, 0) if is_teacher else (100, 100, 200)
            thickness = 3 if is_teacher else 1
            cv2.rectangle(vis, (fx1, fy1), (fx2, fy2), color, thickness)
            label = f"{fr.name} {fr.similarity:.2f}" if fr.name else "???"
            cv2.putText(vis, label, (fx1, fy1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        state_color = (0, 0, 220) if self.state == "E-STOP" else (0, 255, 255)
        cv2.putText(vis, f"[{self.state}]  speed:{self.motor_base:.0f}", (8, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, state_color, 2)

        # 선생님 얼굴 박스 높이 표시 (DIST_STOP / DIST_FOLLOW 값 조정용)
        if teacher_box is not None:
            bh = teacher_box[3] - teacher_box[1]
            cv2.putText(vis, f"bbox_h:{bh}  stop:{DIST_STOP} follow:{DIST_FOLLOW}", (8, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)
        else:
            cv2.putText(vis, "Space:stop  +/-:speed  q:quit", (8, 58),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        cv2.imshow("Pinky AI", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            self._set_e_stop(not self.e_stop)
        elif key in (ord('+'), ord('=')):
            self.motor_base = min(self.motor_base + 5, MOTOR_MAX)
            self.get_logger().info(f"속도: {self.motor_base}")
        elif key == ord('-'):
            self.motor_base = max(self.motor_base - 5, MOTOR_MIN)
            self.get_logger().info(f"속도: {self.motor_base}")
        elif key == ord('q'):
            raise KeyboardInterrupt

    def destroy_node(self):
        self._running = False
        self._face_thread.join(timeout=2.0)
        self.pub.publish(Twist())
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    teacher = select_teacher(FACE_DB_DIR)
    rclpy.init(args=args)
    node = AiNode(teacher)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
