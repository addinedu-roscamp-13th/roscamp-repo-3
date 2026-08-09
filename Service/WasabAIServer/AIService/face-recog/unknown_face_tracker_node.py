#!/usr/bin/env python3
"""JetCobot 카메라에서 미등록(Unknown) 얼굴만 골라 팔 추종 좌표를 발행한다.

입력:
    JetCobot MJPEG 스트림 또는 로컬 카메라

출력:
    /wasab/k3/unknown_face
    (Float64MultiArray: [cx, cy, confidence, distance_m])
    /wasab/k3/unknown_present
    (Bool: 정상 처리된 현재 프레임에 신뢰도 기준을 통과한 Unknown 얼굴이 있는지)

Known으로 판정된 얼굴은 추적하지 않는다. Unknown 얼굴이 여러 명이면
화면에서 가장 크게 보이는 얼굴을 선택한다. 얼굴이 사라지면 메시지를
발행하지 않으며, JetCobot의 arm_search가 face_timeout 후 SEARCH로 전환한다.
presence 토픽은 카메라 프레임과 얼굴 인식이 정상 처리됐을 때만 발행하므로,
토픽 자체가 끊긴 경우를 "외부인 퇴거"로 오인하지 않게 한다.
"""
from __future__ import annotations

import argparse
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray

from face_recognizer import FaceRecognizer, FaceResult


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _face_area(result: FaceResult) -> int:
    x1, y1, x2, y2 = result.bbox
    return max(0, x2 - x1) * max(0, y2 - y1)


def largest_unknown_result(results: list[FaceResult]) -> FaceResult | None:
    """미등록 얼굴 중 화면에서 가장 크게 보이는 얼굴을 반환한다."""
    unknown_faces = [result for result in results if not result.is_known]
    if not unknown_faces:
        return None
    return max(unknown_faces, key=_face_area)

def unknown_confidence(result: FaceResult, tolerance: float) -> float:
    """얼굴 검출 점수와 Known 임계값까지의 여유를 결합한 Unknown 신뢰도."""
    if result.is_known:
        return 0.0
    detection_score = min(1.0, max(0.0, float(result.detection_score)))
    if tolerance <= 0.0:
        return detection_score
    unknown_margin = (float(tolerance) - float(result.similarity)) / float(tolerance)
    return detection_score * min(1.0, max(0.0, unknown_margin))


class UnknownCapableFaceRecognizer(FaceRecognizer):
    """Known DB가 비어 있어도 얼굴을 Unknown으로 반환하는 인식기."""

    @property
    def is_ready(self) -> bool:
        return self._app is not None

    def identify(self, frame_bgr: np.ndarray) -> list[FaceResult]:
        if self._app is None:
            return []

        results: list[FaceResult] = []
        for face in self._app.get(frame_bgr):
            x1, y1, x2, y2 = map(int, face.bbox)
            if (y2 - y1) < self.min_face_size:
                continue

            name = None
            best_similarity = 0.0
            if self._known_mat is not None:
                similarities = self._known_mat @ face.normed_embedding
                best_index = int(np.argmax(similarities))
                best_similarity = float(similarities[best_index])
                if best_similarity >= self.tolerance:
                    name = self.known_names[best_index]

            results.append(
                FaceResult(
                    (x1, y1, x2, y2),
                    name,
                    best_similarity,
                    float(getattr(face, "det_score", 1.0)),
                )
            )
        return results


class LatestFrame:
    """카메라 스트림을 계속 읽고 가장 최근 프레임만 보관한다."""

    def __init__(self, capture):
        self._capture = capture
        self._frame = None
        self._ok = False
        self._lock = threading.Lock()
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self) -> None:
        while self._running:
            ok, frame = self._capture.read()
            with self._lock:
                self._ok = ok
                if ok:
                    self._frame = frame
            if not ok:
                time.sleep(0.01)

    def read(self):
        with self._lock:
            if not self._ok or self._frame is None:
                return False, None
            return True, self._frame.copy()

    def release(self) -> None:
        self._running = False
        self._thread.join(timeout=1.0)
        self._capture.release()


class UnknownFaceTrackerNode(Node):
    """가장 크게 보이는 Unknown 얼굴의 중심 좌표를 발행한다."""

    def __init__(
        self,
        recognizer: UnknownCapableFaceRecognizer,
        source: LatestFrame,
        rate_hz: float,
        mirror: bool,
        show: bool,
        face_topic: str,
        present_topic: str,
        fx_px: float,
        assumed_face_width_m: float,
        unknown_confidence_threshold: float,
        known_only: bool = False,
    ) -> None:
        super().__init__("unknown_face_tracker")
        self._recognizer = recognizer
        self._source = source
        self._mirror = mirror
        self._show = show
        self._fx_px = float(fx_px)
        self._assumed_face_width_m = float(assumed_face_width_m)
        self._unknown_confidence_threshold = float(
            unknown_confidence_threshold
        )
        self._known_only = bool(known_only)
        self._face_publisher = self.create_publisher(
            Float64MultiArray, face_topic, 10
        )
        self._present_publisher = self.create_publisher(
            Bool, present_topic, 10
        )
        self.create_timer(1.0 / rate_hz, self._tick)

    def _publish_target(
        self,
        cx: float,
        cy: float,
        distance_m: float,
        confidence: float,
    ) -> None:
        message = Float64MultiArray()

        # [화면 중심 X, 화면 중심 Y, 검출 신뢰도, 추정 거리(m)]
        message.data = [
            float(cx),
            float(cy),
            float(confidence),
            float(distance_m),
        ]

        self._face_publisher.publish(message)

    def _publish_presence(self, present: bool) -> None:
        message = Bool()
        message.data = bool(present)
        self._present_publisher.publish(message)

    def _tick(self) -> None:
        ok, frame = self._source.read()
        if not ok:
            self.get_logger().warning(
                "camera read failed", throttle_duration_sec=5.0
            )
            return

        if self._mirror:
            frame = cv2.flip(frame, 1)

        height, width = frame.shape[:2]
        try:
            results = self._recognizer.identify(frame)
        except Exception as exc:  # noqa: BLE001 - 다음 프레임에서 계속 시도
            self.get_logger().warning(
                f"identify failed: {exc}", throttle_duration_sec=5.0
            )
            return

        if self._known_only:
            known_faces = [result for result in results if result.is_known]
            target = max(known_faces, key=_face_area) if known_faces else None
            target_confidence = float(target.similarity) if target is not None else 0.0
        else:
            target = largest_unknown_result(results)
            target_confidence = (
                unknown_confidence(target, self._recognizer.tolerance)
                if target is not None
                else 0.0
            )
        target_present = (
            target is not None
            and target_confidence >= self._unknown_confidence_threshold
        )
        self._publish_presence(target_present)

        if target_present:
            x1, y1, x2, y2 = target.bbox

            cx = ((x1 + x2) / 2.0) / width
            cy = ((y1 + y2) / 2.0) / height

            face_width_px = max(1, x2 - x1)

            # 거리 = 초점거리 × 실제 얼굴 폭 / 영상 속 얼굴 폭
            distance_m = (
                self._fx_px
                * self._assumed_face_width_m
                / face_width_px
            )

            self._publish_target(
                cx,
                cy,
                distance_m,
                target_confidence,
            )
            print(
                f"VISION_TARGET cx={cx:.6f} cy={cy:.6f} "
                f"conf={target_confidence:.6f}",
                flush=True,
            )

        self.get_logger().info(
            "faces=%d known=%d unknown=%d target=%s"
            % (
                len(results),
                sum(1 for result in results if result.is_known),
                sum(1 for result in results if not result.is_known),
                (
                    (
                        f"known:{target.name}({target_confidence:.2f})"
                        if self._known_only
                        else f"unknown({target_confidence:.2f})"
                    )
                    if target_present
                    else None
                ),
            ),
            throttle_duration_sec=1.0,
        )

        if self._show:
            self._draw(frame, results, target if target_present else None)

    @staticmethod
    def _draw(
        frame: np.ndarray,
        results: list[FaceResult],
        target: FaceResult | None,
    ) -> None:
        for result in results:
            x1, y1, x2, y2 = result.bbox
            is_target = result is target
            if result.is_known:
                color = (0, 230, 0)
                label = f"KNOWN: {result.name} {result.similarity:.2f}"
            else:
                color = (0, 255, 255) if is_target else (0, 80, 220)
                label = "UNKNOWN TARGET" if is_target else "Unknown"

            thickness = 3 if is_target else 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            cv2.putText(
                frame,
                label,
                (x1, max(y1 - 8, 12)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                color,
                thickness,
            )

        cv2.imshow("K3 unknown face tracker", frame)
        cv2.waitKey(1)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="JetCobot Unknown 얼굴 전용 추적 좌표 발행기"
    )
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--source", help="MJPEG URL 또는 카메라 index")
    parser.add_argument("--face-topic", default="/wasab/k3/unknown_face")
    parser.add_argument(
        "--present-topic",
        default="/wasab/k3/unknown_present",
        help="정상 처리 프레임의 Unknown 존재 여부 Bool 토픽",
    )
    parser.add_argument("--rate", type=float, default=8.0, help="처리 주기(Hz)")
    parser.add_argument("--no-mirror", action="store_true")
    parser.add_argument("--known-only", action="store_true",
                        help="미등록 얼굴 대신 등록 얼굴을 추종 대상으로 사용")
    parser.add_argument("--show", action="store_true", help="얼굴 인식 화면 표시")
    args = parser.parse_args()

    os.chdir(Path(__file__).parent)
    config = _load_config(args.config)
    configured_source = config["camera"]["index"]
    source_value = args.source if args.source is not None else configured_source
    source = (
        int(source_value)
        if isinstance(source_value, str) and source_value.isdigit()
        else source_value
    )

    recognizer = UnknownCapableFaceRecognizer(
        face_db_dir=config["face_db"]["dir"],
        tolerance=config["recognition"]["tolerance"],
        min_face_size=config["recognition"]["min_face_size"],
        model_name=config["model"]["name"],
        det_size=config["model"]["det_size"],
        providers=config["model"]["providers"],
    )
    if not recognizer.is_ready:
        print("[unknown_tracker] 얼굴 인식 모델을 준비하지 못했습니다.")
        return

    capture = cv2.VideoCapture(source)
    capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, config["camera"]["width"])
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, config["camera"]["height"])
    if not capture.isOpened():
        print(f"[unknown_tracker] 카메라 소스 열기 실패: {source}")
        return

    latest_frame = LatestFrame(capture)
    print(
        f"[unknown_tracker] 입력: {source} → {args.face_topic}, "
        f"{args.present_topic} "
        f"({args.rate}Hz, Unknown 전용)"
    )

    rclpy.init()
    node = UnknownFaceTrackerNode(
        recognizer=recognizer,
        source=latest_frame,
        rate_hz=args.rate,
        mirror=not args.no_mirror,
        show=args.show,
        face_topic=args.face_topic,
        present_topic=args.present_topic,
        fx_px=config["camera"].get("fx_px", 600.0),
        assumed_face_width_m=config["camera"].get("assumed_face_width_m", 0.15),
        unknown_confidence_threshold=config["recognition"].get(
            "unknown_confidence_threshold", 0.3
        ),
        known_only=args.known_only,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        latest_frame.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
