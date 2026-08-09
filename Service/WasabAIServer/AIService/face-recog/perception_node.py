#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
perception_node — 얼굴(insightface) + 손목(MediaPipe Hands) 통합 인지 (face-recog .venv).

한 프레임에서 동시에 추출해 발행한다:
  /wasab/k3/face    (Float64[cx,cy,conf]) : 등록 선생님 얼굴 중심(없으면 conf 0)
  /wasab/k3/wrist   (Float64[x,y,conf])   : 손목 정규화 좌표 + conf
  /wasab/k3/gesture (String)              : 손가락 제스처 명령(디바운스 확정 시)
  /wasab/k3/unknown_face
      (Float64[cx,cy,conf,distance_m])     : 가장 큰 미등록 얼굴 추종 좌표
  /wasab/k3/unknown_present (Bool)         : 정상 처리 프레임의 미등록 얼굴 존재 여부
같은 프레임이라 얼굴/손 딜레이가 없다. 추종 대상은 '선생님 얼굴', 손목은 트리거.

레이턴시 대책: 별도 스레드가 스트림을 계속 읽어 '최신 프레임'만 보관한다.

--show 시 얼굴 bbox(이름/Unknown) + 손목 점을 한 창에 오버레이.
게이트(SEARCH/HOLD/TRACK)는 RPi search_node 가 담당.

실행 (face-recog .venv, ROS 환경 source 후):
    ~/face-recog/.venv/bin/python perception_node.py \
        --udp-port 8090 --show
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import cv2
import yaml

import mediapipe as mp

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from wasab_k3_mimic.gesture import (
    GestureDebouncer,
    classify_gesture,
)

from face_recognizer import FaceRecognizer
from udp_stream import UDPFrameReceiver
from unknown_face_tracker_node import (
    largest_unknown_result,
    unknown_confidence,
)


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def best_known_result(results):
    """known 결과 중 best similarity 1개(FaceResult), 없으면 None."""
    known = [r for r in results if r.is_known]
    if not known:
        return None
    return max(known, key=lambda r: r.similarity)


PINKY_DOMAIN_ID = 51
PINKY_GESTURE_TOPIC = "/wasab/gesture_cmd"
PINKY_COMMANDS = frozenset({"PAUSE", "START"})   # HOME/GREET/MOVE_OBJECT는 아직 PinkyPro로 안 보냄


class PerceptionNode(Node):
    """선생님 얼굴 중심 + 손목을 같은 프레임에서 추출해 발행."""

    def __init__(self, recognizer, hands, source, rate_hz, mirror, show,
                 face_topic, wrist_topic, gesture_topic, gesture_frames,
                 unknown_face_topic, unknown_present_topic,
                 fx_px, assumed_face_width_m,
                 unknown_confidence_threshold):
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
        self._unknown_face_pub = self.create_publisher(
            Float64MultiArray, unknown_face_topic, 10)
        self._unknown_present_pub = self.create_publisher(
            Bool, unknown_present_topic, 10)
        self._fx_px = float(fx_px)
        self._assumed_face_width_m = float(assumed_face_width_m)
        self._unknown_confidence_threshold = float(
            unknown_confidence_threshold)
        self._gesture_debounce = GestureDebouncer(gesture_frames)

        # PinkyPro(domain 51) 브리지 — 이 프로세스는 ROS_DOMAIN_ID=69로 떠있으므로
        # 별도 Context를 열어야 51짜리 노드를 같이 띄울 수 있다(wasab_robot_agent/agent_node.py와
        # 같은 2-Context 패턴). 구독 없이 발행만 하므로 spin/스레드 없이 publish()만으로 충분하다.
        self._pinky_ctx = rclpy.Context()
        rclpy.init(context=self._pinky_ctx, domain_id=PINKY_DOMAIN_ID)
        self._pinky_node = Node("k3_gesture_bridge", context=self._pinky_ctx)
        self._pinky_pub = self._pinky_node.create_publisher(
            String, PINKY_GESTURE_TOPIC, 10)
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
            raw_command = classify_gesture(pts)
        else:
            self._publish(self._wrist_pub, [0.0, 0.0, 0.0])

        # 정적 제스처: 디바운스 확정 시 발행
        confirmed = self._gesture_debounce.update(raw_command)
        if confirmed is not None:
            self._emit_command(confirmed)

        # 얼굴 (insightface) — 추종 대상(선생님 중심)
        results = []
        recognition_ok = False
        if self._rec.is_ready:
            try:
                results = self._rec.identify(frame)
                recognition_ok = True
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
            print(
                f"VISION_TARGET cx={cx:.6f} cy={cy:.6f} "
                f"conf={teacher.similarity:.6f}",
                flush=True,
            )
        # 미검출 시 발행 안 함 — search_node 의 face_timeout 으로 간헐 미검출 흡수

        # Unknown은 정상 처리된 프레임에서만 presence를 발행한다. 카메라/추론 실패를
        # 외부인 퇴거로 오인하지 않도록 실패 프레임에서는 False도 발행하지 않는다.
        unknown_present = False
        if recognition_ok:
            unknown = largest_unknown_result(results)
            unknown_conf = (
                unknown_confidence(unknown, self._rec.tolerance)
                if unknown is not None else 0.0)
            unknown_present = (
                unknown is not None
                and unknown_conf >= self._unknown_confidence_threshold)
            self._unknown_present_pub.publish(
                Bool(data=unknown_present))

            if unknown_present:
                x1, y1, x2, y2 = unknown.bbox
                cx = ((x1 + x2) / 2.0) / w
                cy = ((y1 + y2) / 2.0) / h
                face_width_px = max(1, x2 - x1)
                distance_m = (
                    self._fx_px
                    * self._assumed_face_width_m
                    / face_width_px)
                # search_node 계약: [cx, cy, confidence, distance_m]
                self._publish(
                    self._unknown_face_pub,
                    [cx, cy, unknown_conf, distance_m])

        # 서버 GUI가 ROS throttle 로그와 무관하게 매 검출 결과를 받을 수 있도록
        # 등록/미등록 판정을 별도의 기계 판독용 이벤트로 출력한다.
        if teacher is not None:
            print(
                f"FACE_RESULT registered=1 name={teacher.name}",
                flush=True,
            )
        elif unknown_present:
            print("FACE_RESULT registered=0 name=None", flush=True)

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
            font, scale, thick = cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2
            (tw, th), baseline = cv2.getTextSize(text, font, scale, thick)
            pad_x, pad_y = 12, 10
            x0, y0 = 4, 6
            x1, y1 = x0 + tw + 2 * pad_x, y0 + th + baseline + 2 * pad_y
            panel = frame.copy()
            cv2.rectangle(panel, (x0, y0), (x1, y1), (255, 240, 180), -1)
            cv2.addWeighted(panel, 1.0, frame, 0.0, 0, frame)
            cv2.putText(frame, text, (x0 + pad_x, y0 + pad_y + th),
                        font, scale, (40, 20, 0), thick)
        for r in results:
            x1, y1, x2, y2 = r.bbox
            known = r.is_known
            color = (0, 230, 0) if known else (0, 80, 220)
            text_color = (0, 0, 0) if known else (255, 255, 255)
            label = f"{r.name} {r.similarity:.2f}" if known else "Unknown"
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            (lw, lh), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            label_y = max(y1 - 8, 12)
            cv2.rectangle(frame, (x1, label_y - lh - 6), (x1 + lw + 8, label_y + 6), color, -1)
            cv2.putText(frame, label, (x1 + 4, label_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, text_color, 2)
        if wrist_px is not None:
            cv2.circle(frame, wrist_px, 10, (255, 200, 0), 2)
            cv2.putText(frame, "wrist", (wrist_px[0] + 12, wrist_px[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        cv2.imshow("WasabArm", frame)
        cv2.waitKey(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="선생님 얼굴+손목 통합 인지")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--source",
                        help="기존 카메라 스트림 URL 또는 카메라 index. 지정하면 UDP 대신 사용")
    parser.add_argument("--udp-port", type=int, default=8090,
                        help="cam_server.py 로부터 받을 UDP 포트")
    parser.add_argument("--face-topic", default="/wasab/k3/face")
    parser.add_argument("--wrist-topic", default="/wasab/k3/wrist")
    parser.add_argument("--gesture-topic", default="/wasab/k3/gesture")
    parser.add_argument(
        "--unknown-face-topic", default="/wasab/k3/unknown_face")
    parser.add_argument(
        "--unknown-present-topic", default="/wasab/k3/unknown_present")
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

    if args.source:
        source_value = int(args.source) if args.source.isdigit() else args.source
        latest = cv2.VideoCapture(source_value)
        latest.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not latest.isOpened():
            print(f"[perception] 카메라 소스 열기 실패: {args.source}")
            return
        source_description = args.source
    else:
        latest = UDPFrameReceiver(args.udp_port)
        source_description = f"UDP :{args.udp_port}"
    print(
        f"[perception] {source_description} 수신 → "
        "/face, /wrist, /gesture, /unknown_face, /unknown_present "
        f"({args.rate}Hz)")

    rclpy.init()
    node = PerceptionNode(
        recognizer, hands, latest, args.rate,
        not args.no_mirror, args.show,
        args.face_topic, args.wrist_topic,
        args.gesture_topic, args.gesture_frames,
        args.unknown_face_topic, args.unknown_present_topic,
        cfg["camera"].get("fx_px", 600.0),
        cfg["camera"].get("assumed_face_width_m", 0.1),
        cfg["recognition"].get(
            "unknown_confidence_threshold", 0.3))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception:
        # SIGTERM으로 종료할 때 rclpy의 기본 signal handler가 context를 먼저
        # 무효화할 수 있다. 정상 종료 과정에서 발생한 publish/spin 예외만 삼킨다.
        if rclpy.ok():
            raise
    finally:
        latest.release()
        cv2.destroyAllWindows()
        node._pinky_node.destroy_node()
        if node._pinky_ctx.ok():
            rclpy.shutdown(context=node._pinky_ctx)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
