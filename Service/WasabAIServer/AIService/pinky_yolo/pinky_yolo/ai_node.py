import math
import os
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import String

try:
    from pinky_yolo.face_recognizer import FaceRecognizer
except ImportError:
    from face_recognizer import FaceRecognizer

# ── 경로 설정 ─────────────────────────────────────────────────────────────────
def _find_face_db_dir():
    # 1) 소스에서 직접 실행한 경우: ai_node.py 위치(.../AIService/pinky_yolo/pinky_yolo/)
    #    기준 4단계 위가 Service/WasabAIServer, 그 밑에 FaceDB가 있다.
    src_relative = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))),
        "FaceDB",
    )
    if os.path.isdir(os.path.join(src_relative, "known")):
        return src_relative
    # 2) colcon install 후 ros2 run(예: run_ai.sh)으로 실행한 경우.
    #    __file__은 install/pinky_yolo/lib/.../site-packages/pinky_yolo/ai_node.py로 복사되어
    #    소스 트리와 무관해지므로, 패키지 prefix로 워크스페이스 루트를 찾아 되짚어간다.
    try:
        from ament_index_python.packages import get_package_prefix
        ws_root = os.path.dirname(os.path.dirname(get_package_prefix("pinky_yolo")))
        candidate = os.path.join(ws_root, "src", "roscamp-repo-3", "Service", "WasabAIServer", "FaceDB")
        if os.path.isdir(candidate):
            return candidate
    except Exception:
        pass
    return src_relative  # 못 찾으면 1번 값이라도 반환

FACE_DB_DIR = _find_face_db_dir()

# ── 얼굴 인식 (InsightFace SCRFD + ArcFace) ───────────────────────────────────
FACE_TOLERANCE = 0.40   # 먼 거리에서도 인식되도록 완화 (기존 0.45) — 오인식 늘어나면 다시 올릴 것
FACE_MIN_SIZE  = 40

# ── PD 제어 ──────────────────────────────────────────────────────────────────
MOTOR_BASE  = 40.0
MOTOR_MIN   =  5.0
MOTOR_MAX   = 80.0
KP          = 0.05
KD          = 0.05
DEADZONE    = 20
MAX_ANGULAR = 6.0

FRAME_W  = 640
CENTER_X = FRAME_W // 2

# ── 두리번거리기 탐색 ─────────────────────────────────────────────────────────
SEARCH_ANGULAR         = 10.0
SEEN_GRACE             = 0.5
SEARCH_SWING_INCREMENT = 0.25
SEARCH_SWING_CYCLES    = 3

# 객체가 사라진 방향(+1)을 먼저 본 뒤 중심, 반대편, 중심으로 돌아온다.
# 첫 0.5초는 SEEN_GRACE 구간 자체를 SEARCH 1단계로 사용한다.
SEARCH_PHASES = tuple(
    (direction_scale, SEEN_GRACE + cycle * SEARCH_SWING_INCREMENT)
    for cycle in range(SEARCH_SWING_CYCLES)
    for direction_scale in (1.0, -1.0, -1.0, 1.0)
)
LOST_TIMEOUT           = sum(duration for _, duration in SEARCH_PHASES)
LOST_ESTOP_SEC         = 33.0

# ── 제스처 연동 (JetCobot perception_node.py → domain 51 브리지) ─────────────
GESTURE_CMD_TOPIC = "/wasab/gesture_cmd"   # 구독: PAUSE/START
LED_STATE_TOPIC   = "/wasab/led_state"     # 발행: raspi pinky_node.py가 LED 제어

# ── 추종 대상 지정 (웹앱 "교실이동" → wasab_robot_agent → 로컬 재발행) ────────
# 웹앱에서 얼굴인증(session.face_name, 예: "HUIWON.teacher")을 그대로 페이로드로 보낸다 —
# 로그인 아이디와 매칭할 필요 없음(얼굴인증 결과 자체가 face_db 등록명이라 바로 쓸 수 있음).
FOLLOW_CMD_TOPIC = "/wasab/follow_cmd"     # 구독: 추종 대상 이름(문자열, face_db 폴더명과 동일)

# ── 웹앱 "일시정지"/"수업 동행 정지" 버튼 (→ wasab_robot_agent → 로컬 재발행) ──
# 기존 cmd_mode(idle)는 ai_node.py가 구독을 안 해서 버튼 눌러도 실제로는 안 멈췄음 — 별도 채널 추가.
ASSIST_STOP_TOPIC = "/wasab/assist_stop_cmd"   # 구독: 아무 페이로드든 수신 시 즉시 E-STOP

# ── 라이다 기반 거리재급/회피 ─────────────────────────────────────────────────
TOO_CLOSE_DIST   = 0.15   # m, 이내면 TOO_CLOSE (얼굴 bbox 기준과 OR) — 목표 정지거리 10cm
OBSTACLE_DIST    = 0.10
LIDAR_SLOW_DIST  = 0.25   # m, 이 거리부터 서서히 감속 시작 (LIDAR_STOP_DIST에서 속도 0)
FRONT_CONE_DEG   = 30.0   # 정면 판단 반각(양쪽 합 60도) — angle 0 = 로봇 정면 가정
FRONT_OFFSET_DEG = 180.0  # 라이다가 물리적으로 반대(뒤)를 보게 장착되어 있어서 보정
                          # (2026-07-15 실기 확인: 진짜 정면에 장애물을 놔도 F/L/R 전부 무반응 → 180도 회전 장착으로 판단)


def get_search_phase(elapsed: float):
    """객체 상실 후 경과 시간에 해당하는 (방향 배율, 단계 시간)을 반환한다."""
    remaining = max(0.0, elapsed)
    for direction_scale, duration in SEARCH_PHASES:
        if remaining < duration:
            return direction_scale, duration
        remaining -= duration
    return None


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


class AiNode(Node):
    def __init__(self, teacher: str | None, robot_id: int | None = None, console_domain: int = 50):
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
        self.get_logger().info(f"추종 대상: {self.teacher}" if self.teacher
                                else "추종 대상: 미지정 — follow_cmd 명령 대기 중")


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

        # 추종 대상 지정: 웹앱 "교실이동"(얼굴인증된 세션) → wasab_robot_agent 로컬 재발행.
        # 대상 지정과 동시에 추종 시작(e_stop 해제)까지 겸한다 — 버튼 자체가 "지금 이 사람 쫓아가라"는 뜻.
        self.create_subscription(
            String, FOLLOW_CMD_TOPIC, self.follow_cmd_cb, 10)

        # 웹앱(콘솔 도메인)이 agent_node.py 브리지 없이 identify_cmd를 직접 보낼 수 있도록,
        # fire_detect_node.py의 _console_ctx와 동일한 2-Context 패턴을 이 노드 안에 둔다.
        # agent_node.py는 로봇 디스크 리소스 문제로 수정 불가라 이 방식으로 대체.
        self._console_ctx = None
        if robot_id is not None:
            self._console_ctx = rclpy.Context()
            rclpy.init(context=self._console_ctx, domain_id=console_domain)
            self._console_node = Node("ai_identify_bridge", context=self._console_ctx)
            self._console_node.create_subscription(
                String, f"/robot_{int(robot_id)}/identify_cmd", self.identify_cmd_cb, 10)
            from rclpy.executors import SingleThreadedExecutor
            self._console_exec = SingleThreadedExecutor(context=self._console_ctx)
            self._console_exec.add_node(self._console_node)
            self.get_logger().info(
                f"콘솔 도메인({console_domain}) identify_cmd 브리지 활성화: robot_{robot_id}")

        # 웹앱 "일시정지"/"수업 동행 정지" 버튼 → 즉시 E-STOP
        self.create_subscription(
            String, ASSIST_STOP_TOPIC, self.assist_stop_cb, 10)

        # 라이다: 거리재급/회피 (PinkyNode가 sllidar_ros2를 서브프로세스로 기동)
        self.create_subscription(LaserScan, "/scan", self.scan_cb, 10)
        self.scan_front = None
        self.scan_left  = None
        self.scan_right = None

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


    def _set_e_stop(self, value: bool, reason: str = "MANUAL"):

        """e_stop 토글. TOO_CLOSE/SEARCH(장애물)는 이 함수를 안 거치는 별개의 자동 상태(START 불필요) —
        이 함수가 True로 호출되면 항상 E-STOP(빨강, START로만 해제)이다.
        reason은 로그 구분용(MANUAL/LOST)."""

        self.e_stop = value
        self.get_logger().info(f"긴급 정지 ON ({reason})" if value else f"긴급 정지 해제")

        if value:
            self.led_pub.publish(String(data="PAUSED"))


    def _sync_led(self):

        """매 프레임 state 변화 감지 → FOLLOW면 초록, 그 외(SEARCH/LOST/TOO_CLOSE)면 꺼짐.
        E-STOP 진입 시의 빨강 3초는 _set_e_stop이 이미 처리하므로 여기선 건드리지 않는다."""

        if self.state == self._last_led_state:
            return
        
        self._last_led_state = self.state

        if self.state == "TOO_CLOSE":
            self.led_pub.publish(String(data="HOLD"))

        elif self.state == "FOLLOW":
            self.led_pub.publish(String(data="FOLLOWING"))

        elif self.state in ("SEARCH", "LOST"):
            self.led_pub.publish(String(data="WARNING"))

        elif self.state == "E-STOP":
            self.led_pub.publish(String(data="PAUSED"))

        else:
            self.led_pub.publish(String(data="OFF"))


    def gesture_cmd_cb(self, msg: String):
        if msg.data == "PAUSE" and not self.e_stop:
            self.get_logger().info("제스처 PAUSE 수신")
            self._set_e_stop(True)
        elif msg.data == "START" and self.e_stop:
            self.get_logger().info("제스처 START 수신")
            self._set_e_stop(False)

    def follow_cmd_cb(self, msg: String):
        name = msg.data.strip()
        if not name:
            return
        self.teacher = name
        self.get_logger().info(f"추종 대상 지정: {name}")
        if self.e_stop:
            self._set_e_stop(False)

    def identify_cmd_cb(self, msg: String):
        """얼굴인증 완료 시점 자동 호출 — 대상만 등록, E-STOP은 그대로 유지(움직이지 않음).
        실제 추종 시작은 Space/제스처/웹앱 버튼 중 하나로 별도로 해야 한다."""
        name = msg.data.strip()
        if not name:
            return
        self.teacher = name
        self.get_logger().info(f"추종 대상 자동 지정(얼굴인증): {name} — 시작은 별도로 해야 함")

    def assist_stop_cb(self, msg: String):
        self.get_logger().info("웹앱 일시정지/수업동행 정지 수신")
        if not self.e_stop:
            self._set_e_stop(True, "WEBAPP")

    def scan_cb(self, msg: LaserScan):

        """정면(±FRONT_CONE_DEG)/좌/우 최소 거리 계산. FRONT_OFFSET_DEG로 장착 방향 보정."""

        front_cone = math.radians(FRONT_CONE_DEG)
        front_offset = math.radians(FRONT_OFFSET_DEG)
        front_vals, left_vals, right_vals = [], [], []

        for i, r in enumerate(msg.ranges):
            if not (msg.range_min < r < msg.range_max):
                continue

            angle = msg.angle_min + i * msg.angle_increment + front_offset
            a = (angle + math.pi) % (2 * math.pi) - math.pi  # -pi..pi 정규화

            if abs(a) <= front_cone:
                front_vals.append(r)
            elif 0 < a <= math.pi / 2:
                left_vals.append(r)
            elif -math.pi / 2 <= a < 0:
                right_vals.append(r)

        self.scan_front = min(front_vals) if front_vals else None
        self.scan_left  = min(left_vals) if left_vals else None
        self.scan_right = min(right_vals) if right_vals else None

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
        # 콘솔 도메인 identify_cmd 펌프 — 별도 context라 전역 executor가 자동으로 안 돌려준다.
        # 여기서 안 부르면 콜백이 영원히 안 불림(fire_detect_node.py의 spin_console과 동일 이유).
        if self._console_ctx is not None:
            self._console_exec.spin_once(timeout_sec=0.0)

        frame = decode_compressed(msg)
        if frame is None:
            return

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

        # 선생님 얼굴 찾기 (거리 계산은 얼굴 박스로 통일)
        # teacher_box는 SEEN_GRACE 동안 남아 있으므로, 현재 프레임에서 실제로
        # 검출됐는지는 teacher_seen_now로 별도 구분한다.
        # self.teacher가 None(미지정)일 때 fr.name도 None(미인식 얼굴)이면 None==None이 True가 되어
        # 엉뚱한 미인식 얼굴을 추종 대상으로 착각하는 버그가 있었음 — self.teacher is not None 가드 필수.
        teacher_seen_now = False
        for fr in face_results:
            if self.teacher is not None and fr.name == self.teacher:
                teacher_seen_now = True
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


        # ── 근접 판정 ────────────────────────────────────────────────────────
        # 사람/물체 구분 없음, bbox도 안 봄 — 선생님 얼굴이 보이는 동안 라이다 값으로만 판단.
        # OBSTACLE_DIST~TOO_CLOSE_DIST 사이는 정상 도달 → TOO_CLOSE(별도 상태, 자동복귀).
        # OBSTACLE_DIST보다 더 가까우면 장애물로 판단하지만, 상태 전환 없이 FOLLOW 라벨 그대로
        # 두고 속도만 0으로 만든다 (E-STOP도 SEARCH도 아님).
        is_teacher_close = (
            teacher_seen_now
            and teacher_box is not None
            and self.scan_front is not None
            and OBSTACLE_DIST <= self.scan_front < TOO_CLOSE_DIST
        )
        obstacle_blocking = (
            teacher_seen_now
            and teacher_box is not None
            and self.scan_front is not None
            and self.scan_front < OBSTACLE_DIST
        )



        # ── 상태 머신 + cmd_vel ───────────────────────────────────────────────
        twist = Twist()

        if self.e_stop:
            self.state = "E-STOP"

        elif is_teacher_close:
            # 선생님한테 도달해서 정지 — 자동 상태, START 없이도 거리 벌어지면 바로 FOLLOW로 복귀
            self.state = "TOO_CLOSE"
            self.prev_error = 0.0

        elif teacher_seen_now and teacher_box is not None:
            x1, y1, x2, y2 = teacher_box
            error  = (x1 + x2) // 2 - CENTER_X
            self.state = "FOLLOW"

            if obstacle_blocking:
                # 라이다 0.10m 미만 — 장애물로 판단, FOLLOW 라벨 유지한 채 완전 정지
                correction = 0.0
                distance_factor = 0.0
            else:
                if abs(error) >= DEADZONE:
                    d_err = (error - self.prev_error) / dt
                    # 2026-07-15: flip 제거만으로는 안 고쳐져서 부호도 같이 반전(테스트 조합 2/2)
                    correction = -float(np.clip(KP * error + KD * d_err, -MAX_ANGULAR, MAX_ANGULAR))
                else:
                    correction = 0.0

                # 라이다 정면 거리가 LIDAR_SLOW_DIST 이내면 서서히 감속
                # (LIDAR_SLOW_DIST=1.0배속 → TOO_CLOSE_DIST에서 0배속, 그 사이는 선형 보간)
                if self.scan_front is not None and self.scan_front < LIDAR_SLOW_DIST:
                    distance_factor = max(0.0, (self.scan_front - TOO_CLOSE_DIST) / (LIDAR_SLOW_DIST - TOO_CLOSE_DIST))
                else:
                    distance_factor = 1.0

            centering = 1.0 - min(abs(error) / CENTER_X, 1.0)
            twist.linear.x  = float(np.clip(self.motor_base * centering * distance_factor, 0.0, MOTOR_MAX))
            twist.angular.z = correction
            self.prev_error = float(error)

        else:
            # 현재 프레임에서 얼굴이 안 보이면 즉시 제자리 SEARCH를 시작한다.
            # SEEN_GRACE 0.5초는 객체가 사라진 방향으로 보는 첫 단계이며,
            # 이후 중심 → 반대편 → 중심 순서로 범위를 점점 넓힌다.
            elapsed = t_now - self.last_seen

            if elapsed < LOST_TIMEOUT:
                self.state = "SEARCH"
                twist.linear.x = 0.0

                phase = get_search_phase(elapsed)
                if phase is not None:
                    direction_scale, _ = phase
                    # FOLLOW 보정은 화면 오차와 반대 부호이므로, 실제로 객체를
                    # 향해 회전하던 방향은 -last_error_dir이다.
                    outward_direction = -self.last_error_dir
                    direction = outward_direction * direction_scale
                    twist.angular.z = float(direction * SEARCH_ANGULAR)

            elif elapsed < LOST_ESTOP_SEC:
                self.state = "LOST"

                twist.linear.x = 0.0
                outward_direction = -self.last_error_dir
                twist.angular.z = float(outward_direction * SEARCH_ANGULAR * 0.5)

            else:
                # LOST: 느린 속도로 한 방향 계속 회전하며 탐색
                self.state = "E-STOP"

                twist.linear.x  = 0.0
                twist.angular.z = 0.0

                self._set_e_stop(True, "LOST")

            self.prev_error = 0.0

        self.pub.publish(twist)
        self._sync_led()

        # ── 시각화 ───────────────────────────────────────────────────────────
        vis = frame.copy()

        for fr in face_results:
            fx1, fy1, fx2, fy2 = fr.bbox
            is_teacher = (self.teacher is not None and fr.name == self.teacher)
            color     = (0, 220, 0) if is_teacher else (100, 100, 200)
            thickness = 3 if is_teacher else 1
            cv2.rectangle(vis, (fx1, fy1), (fx2, fy2), color, thickness)
            label = f"{fr.name} {fr.similarity:.2f}" if fr.name else "???"
            cv2.putText(vis, label, (fx1, fy1 - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # 상태/속도/라이다 — 텍스트마다 불투명 배경 박스(카메라 배경과 무관하게 항상 잘 보이도록)
        STATE_COLORS = {
            "FOLLOW":    (0, 150, 0),     # 초록
            "TOO_CLOSE": (200, 40, 0),    # 파랑
            "SEARCH":    (0, 110, 230),   # 주황
            "LOST":      (0, 110, 230),   # 주황
            "E-STOP":    (0, 0, 210),     # 빨강
        }
        WHITE, YELLOW, BLACK = (255, 255, 255), (0, 255, 255), (0, 0, 0)

        def draw_chip(text, x, y, scale, text_color, bg_color, thickness=2):
            (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)
            cv2.rectangle(vis, (x, y), (x + tw + 16, y + th + base + 12), bg_color, -1)
            cv2.putText(vis, text, (x + 8, y + th + 6),
                        cv2.FONT_HERSHEY_SIMPLEX, scale, text_color, thickness, cv2.LINE_AA)
            return y + th + base + 12 + 8   # 다음 줄 y 시작점

        y = 10
        y = draw_chip(self.state, 10, y, 1.0, STATE_COLORS.get(self.state, BLACK), WHITE)

        speed_txt = f"SPEED {self.motor_base:.0f}  (real {twist.linear.x:.0f})"
        y = draw_chip(speed_txt, 10, y, 0.65, BLACK, WHITE)

        sf = f"{self.scan_front:.2f}" if self.scan_front is not None else "--"
        sl = f"{self.scan_left:.2f}"  if self.scan_left  is not None else "--"
        sr = f"{self.scan_right:.2f}" if self.scan_right is not None else "--"
        lidar_txt = f"F {sf}   L {sl}   R {sr}"
        y = draw_chip(lidar_txt, 10, y, 0.65, BLACK, YELLOW)

        cv2.imshow("WASAB MC", vis)

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

        if self._console_ctx is not None:
            self._console_node.destroy_node()
            rclpy.shutdown(context=self._console_ctx)

        cv2.destroyAllWindows()

        super().destroy_node()


def main(args=None):
    import argparse
    import sys
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser()
    parser.add_argument("--teacher", default=None,
                        help="시작 시 추종 대상 고정 지정(생략하면 follow_cmd 명령 대기)")
    parser.add_argument("--interactive", action="store_true",
                        help="터미널에서 번호로 선택(웹앱 없이 단독 테스트용, 기존 방식)")
    parser.add_argument("--robot-id", type=int, default=None,
                        help="robots.yaml 상의 이 로봇 id — 지정 시 콘솔 도메인 identify_cmd를 "
                             "agent_node.py 없이 직접 구독(생략하면 이 브리지 비활성화)")
    parser.add_argument("--console-domain", type=int, default=50,
                        help="웹앱(user_gui) 백엔드가 떠 있는 콘솔 도메인")
    ns, _ = parser.parse_known_args(remove_ros_args(args=sys.argv)[1:] if args is None else args)

    teacher = ns.teacher
    if ns.interactive and teacher is None:
        teacher = select_teacher(FACE_DB_DIR)

    rclpy.init(args=args)
    node = AiNode(teacher, robot_id=ns.robot_id, console_domain=ns.console_domain)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

