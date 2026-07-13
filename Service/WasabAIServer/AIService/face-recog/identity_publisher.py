#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
identity_publisher — 등록 선생님 신원을 ROS 토픽으로 발행 (face-recog .venv).

MJPEG 스트림 프레임마다 FaceRecognizer.identify 로 등록자를 인식하고,
known 중 best similarity 1명의 이름(없으면 "")을
/wasab/k3/identity (std_msgs/String) 로 주기 발행한다.
프레임 read 실패·모델 미준비·부재·인식 예외도 ""을 계속 발행해
구독측(search_node) 상태가 예측 가능하게 한다.

--show 시 얼굴 bbox + 이름(known)/Unknown(미등록) + 유사도를 오버레이한다.

실행 (face-recog .venv, ROS 환경 source 후):
    ~/face-recog/.venv/bin/python identity_publisher.py \
        --source http://192.168.0.86:8090/stream --show
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

import cv2
import yaml

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from face_recognizer import FaceRecognizer


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def best_known_name(results) -> str:
    """known 결과 중 best similarity 1명 이름, 없으면 ''."""
    known = [r for r in results if r.is_known]
    if not known:
        return ""
    return max(known, key=lambda r: r.similarity).name


class IdentityPublisher(Node):
    """프레임 인식 결과를 /wasab/k3/identity 로 주기 발행하는 노드."""

    def __init__(self, recognizer, cap, topic, rate_hz, mirror, show):
        super().__init__("identity_publisher")
        self._rec = recognizer
        self._cap = cap
        self._mirror = mirror
        self._show = show
        self._pub = self.create_publisher(String, topic, 10)
        self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        name = ""
        results = []
        ok, frame = self._cap.read()
        if ok and self._rec.is_ready:
            if self._mirror:
                frame = cv2.flip(frame, 1)
            try:
                results = self._rec.identify(frame)
                name = best_known_name(results)
            except Exception as e:   # noqa: BLE001 - 인식 예외는 ""로 흡수
                self.get_logger().warning(
                    f"identify failed: {e}", throttle_duration_sec=5.0)
        msg = String()
        msg.data = name
        self._pub.publish(msg)
        if self._show and ok:
            self._draw(frame, results)

    def _draw(self, frame, results):
        """얼굴 bbox + 이름(known)/Unknown(미등록) + 유사도 오버레이."""
        for r in results:
            x1, y1, x2, y2 = r.bbox
            known = r.is_known
            color = (0, 230, 0) if known else (0, 80, 220)
            label = (f"{r.name} {r.similarity:.2f}" if known
                     else f"Unknown {r.similarity:.2f}")
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, max(y1 - 8, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.imshow("K3 identity (face)", frame)
        cv2.waitKey(1)


def main() -> None:
    parser = argparse.ArgumentParser(description="등록 선생님 신원 ROS 발행")
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--source", help="MJPEG URL 또는 카메라 index")
    parser.add_argument("--topic", default="/wasab/k3/identity")
    parser.add_argument("--rate", type=float, default=5.0, help="발행 주기(Hz)")
    parser.add_argument("--no-mirror", action="store_true",
                        help="좌우반전 끄기(기본 미러)")
    parser.add_argument("--show", action="store_true",
                        help="얼굴 인식 결과 오버레이 창")
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
        print("[identity] ❌ 인식기 미준비 (등록 얼굴 없음? register 먼저)")
        return

    cap = cv2.VideoCapture(source)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # 최신 프레임만(딜레이 누적 방지)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cfg["camera"]["width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cfg["camera"]["height"])
    if not cap.isOpened():
        print(f"[identity] ❌ 소스 열기 실패: {source}")
        return
    print(f"[identity] 입력: {source} → {args.topic} ({args.rate}Hz)")

    rclpy.init()
    node = IdentityPublisher(
        recognizer, cap, args.topic, args.rate,
        not args.no_mirror, args.show)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
