#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
perception_node — 얼굴(insightface) + 손목(MediaPipe Hands) 통합 인지 (face-recog .venv).

한 프레임에서 동시에 추출해 발행한다:
  /wasab/k3/face    (Float64[cx,cy,conf]) : 등록 선생님 얼굴 중심(없으면 conf 0)
  /wasab/k3/wrist   (Float64[x,y,conf])   : 손목 정규화 좌표 + conf
  /wasab/k3/gesture (String)              : 손가락 제스처 명령(디바운스 확정 시)
같은 프레임이라 얼굴/손 딜레이가 없다. 추종 대상은 '선생님 얼굴', 손목은 트리거.

레이턴시 대책: 별도 스레드가 스트림을 계속 읽어 '최신 프레임'만 보관한다.

--show 시 얼굴 bbox(이름/Unknown) + 손목 점을 한 창에 오버레이.
게이트(SEARCH/HOLD/TRACK)는 RPi search_node 가 담당.

실행 (face-recog .venv, ROS 환경 source 후):
    ~/face-recog/.venv/bin/python perception_node.py \
        --source http://192.168.0.86:8090/stream --show
"""
from __future__ import annotations

import argparse
import os
import threading
from pathlib import Path

import cv2
import yaml

import mediapipe as mp

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String

from wasab_k3_mimic.gesture import (
    CircleDetector,
    GestureDebouncer,
    classify_gesture,
    count_extended_fingers,
)

from face_recognizer import FaceRecognizer


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def best_known_result(results):
    """known 결과 중 best similarity 1개(FaceResult), 없으면 None."""
    known = [r for r in results if r.is_known]
    if not known:
        return None
    return max(known, key=lambda r: r.similarity)


class LatestFrame:
    """스트림을 백그라운드 스레드로 계속 읽어 최신 프레임만 보관(지연 제거)."""

    def __init__(self, cap):
        self._cap = cap
        self._frame = None
        self._ok = False
        self._lock = threading.Lock()
        self._run = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._run:
            ok, frame = self._cap.read()
            with self._lock:
                self._ok, self._frame = ok, frame

    def read(self):
        """가장 최근 프레임 (ok, frame_copy)."""
        with self._lock:
            if not self._ok or self._frame is None:
                return False, None
            return True, self._frame.copy()

    def release(self):
        self._run = False
        self._thread.join(timeout=1.0)
        self._cap.release()


PINKY_DOMAIN_ID = 51
PINKY_GESTURE_TOPIC = "/wasab/gesture_cmd"
PINKY_COMMANDS = frozenset({"PAUSE", "START"})   # HOME/GREET/MOVE_OBJECT는 아직 PinkyPro로 안 보냄


class PerceptionNode(Node):
    """선생님 얼굴 중심 + 손목을 같은 프레임에서 추출해 발행."""

    def __init__(self, recognizer, hands, source, rate_hz, mirror, show,
                 face_topic, wrist_topic, gesture_topic, gesture_frames):
        super().__init__("perception")
        self._rec = recognizer
        self._hands = hands
        self._source = source
        self._mirror = mirror
        self._show = show
        self._face_pub = self.create_publisher(
            Float64MultiArray, face_topic, 10)
        self._wrist_pub = self.create_publisher(
            Float64MultiArray, wrist_topic, 10)
        self._gesture_pub = self.create_publisher(String, gesture_topic, 10)
        self._gesture_debounce = GestureDebouncer(gesture_frames)

        # PinkyPro(domain 51) 브리지 — 이 프로세스는 ROS_DOMAIN_ID=69로 떠있으므로
        # 별도 Context를 열어야 51짜리 노드를 같이 띄울 수 있다(wasab_robot_agent/agent_node.py와
        # 같은 2-Context 패턴). 구독 없이 발행만 하므로 spin/스레드 없이 publish()만으로 충분하다.
        self._pinky_ctx = rclpy.Context()
        rclpy.init(context=self._pinky_ctx, domain_id=PINKY_DOMAIN_ID)
        self._pinky_node = Node("k3_gesture_bridge", context=self._pinky_ctx)
        self._pinky_pub = self._pinky_node.create_publisher(
            String, PINKY_GESTURE_TOPIC, 10)
        self._circle = CircleDetector()   # 검지 원-모션 → MOVE_OBJECT
        self._last_command = None   # 마지막 발행 명령(오버레이용)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _publish(self, pub, data):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in data]
        pub.publish(msg)

    def _tick(self):
        ok, frame = self._source.read()
        if not ok:
            self.get_logger().warning(
                "camera read failed", throttle_duration_sec=5.0)
            return
        if self._mirror:
            frame = cv2.flip(frame, 1)
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 손 (MediaPipe Hands) — 손목 트리거 + 제스처 명령
        wrist_px = None
        raw_command = None
        hres = self._hands.process(rgb)
        if hres.multi_hand_landmarks:
            hand = hres.multi_hand_landmarks[0]
            lm = hand.landmark[0]                           # 0 = WRIST
            conf = 1.0
            if hres.multi_handedness:
                conf = hres.multi_handedness[0].classification[0].score
            self._publish(self._wrist_pub, [lm.x, lm.y, conf])
            wrist_px = (int(lm.x * w), int(lm.y * h))
            pts = [(p.x, p.y) for p in hand.landmark]
            if count_extended_fingers(pts) == 1:
                # 검지 1개: 정지=START / 원=MOVE_OBJECT (모션으로 구분)
                state = self._circle.update(pts[8])   # 8 = 검지끝
                if state == "CIRCLE":
                    self._emit_command("MOVE_OBJECT")   # 즉시(디바운스 우회)
                elif state == "STILL":
                    raw_command = "START"
                # MOVING 이면 raw_command=None (START 억제)
            else:
                self._circle.update(None)
                raw_command = classify_gesture(pts)
        else:
            self._publish(self._wrist_pub, [0.0, 0.0, 0.0])
            self._circle.update(None)

        # 정적 제스처: 디바운스 확정 시 발행
        confirmed = self._gesture_debounce.update(raw_command)
        if confirmed is not None:
            self._emit_command(confirmed)

        # 얼굴 (insightface) — 추종 대상(선생님 중심)
        results = []
        if self._rec.is_ready:
            try:
                results = self._rec.identify(frame)
            except Exception as e:   # noqa: BLE001 - 인식 예외는 conf 0으로
                self.get_logger().warning(
                    f"identify failed: {e}", throttle_duration_sec=5.0)
        teacher = best_known_result(results)
        self.get_logger().info(
            "faces=%d known=%d teacher=%s ready=%s" % (
                len(results),
                sum(1 for r in results if r.is_known),
                teacher.name if teacher else None,
                self._rec.is_ready),
            throttle_duration_sec=1.0)
        if teacher is not None:
            x1, y1, x2, y2 = teacher.bbox
            cx = ((x1 + x2) / 2.0) / w
            cy = ((y1 + y2) / 2.0) / h
            self._publish(self._face_pub, [cx, cy, teacher.similarity])
        # 미검출 시 발행 안 함 — search_node 의 face_timeout 으로 간헐 미검출 흡수

        if self._show:
            self._draw(frame, results, wrist_px)

    def _emit_command(self, command):
        """제스처 명령 발행 + 오버레이 갱신 + 로그."""
        self._gesture_pub.publish(String(data=command))
        if command in PINKY_COMMANDS:
            self._pinky_pub.publish(String(data=command))
        self._last_command = command
        self.get_logger().info("gesture command: %s" % command)

    def _draw(self, frame, results, wrist_px):
        """얼굴 bbox(이름/Unknown) + 손목 점 + 우상단 제스처 명령 오버레이."""
        if self._last_command:
            text = "GESTURE: %s" % self._last_command
            cv2.putText(frame, text, (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        for r in results:
            x1, y1, x2, y2 = r.bbox
            known = r.is_known
            color = (0, 230, 0) if known else (0, 80, 220)
            label = (f"{r.name} {r.similarity:.2f}" if known
                     else f"Unknown {r.similarity:.2f}")
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, max(y1 - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        if wrist_px is not None:
            cv2.circle(frame, wrist_px, 10, (255, 200, 0), 2)
            cv2.putText(frame, "wrist", (wrist_px[0] + 12, wrist_px[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        cv2.imshow("K3 perception (face+hand)", frame)
        cv2.waitKey(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="선생님 얼굴+손목 통합 인지")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--source", help="MJPEG URL 또는 카메라 index")
    parser.add_argument("--face-topic", default="/wasab/k3/face")
    parser.add_argument("--wrist-topic", default="/wasab/k3/wrist")
    parser.add_argument("--gesture-topic", default="/wasab/k3/gesture")
    parser.add_argument("--gesture-frames", type=int, default=3,
                        help="제스처 확정에 필요한 연속 프레임 수")
    parser.add_argument("--rate", type=float, default=8.0, help="처리 주기(Hz)")
    parser.add_argument("--no-mirror", action="store_true")
    parser.add_argument("--show", action="store_true", help="오버레이 창")
    try:
        import argcomplete
        argcomplete.autocomplete(parser)
    except ImportError:
        pass
    args = parser.parse_args()

    os.chdir(Path(__file__).parent)
    cfg = _load_config(args.config)
    src = args.source if args.source is not None else cfg["camera"]["index"]
    source = int(src) if isinstance(src, str) and src.isdigit() else src

    recognizer = FaceRecognizer(
        face_db_dir=cfg["face_db"]["dir"],
        tolerance=cfg["recognition"]["tolerance"],
        min_face_size=cfg["recognition"]["min_face_size"],
        model_name=cfg["model"]["name"],
        det_size=cfg["model"]["det_size"],
        providers=cfg["model"]["providers"],
    )
    if not recognizer.is_ready:
        print("[perception] ❌ 인식기 미준비 (등록 얼굴 없음?)")
        return

    hands = mp.solutions.hands.Hands(max_num_hands=1)

    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])
    if not cap.isOpened():
        print(f"[perception] ❌ 소스 열기 실패: {source}")
        return
    latest = LatestFrame(cap)
    print(f"[perception] 입력: {source} → /face, /wrist, /gesture "
          f"({args.rate}Hz)")

    rclpy.init()
    node = PerceptionNode(
        recognizer, hands, latest, args.rate,
        not args.no_mirror, args.show,
        args.face_topic, args.wrist_topic,
        args.gesture_topic, args.gesture_frames)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        latest.release()
        cv2.destroyAllWindows()
        node._pinky_node.destroy_node()
        rclpy.shutdown(context=node._pinky_ctx)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
