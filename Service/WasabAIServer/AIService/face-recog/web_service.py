#!/usr/bin/env python3
# PYTHON_ARGCOMPLETE_OK
"""
web_service — 웹앱용 얼굴·제스처 판정 HTTP 서비스 (face-recog 격리 venv 전용).

웹앱 백엔드는 insightface/mediapipe 를 직접 import 할 수 없다(numpy ABI 충돌로 venv 분리).
그래서 이 프로세스가 판정만 담당하고 웹앱은 HTTP 로 물어본다.

판정 로직은 새로 만들지 않는다 — 기존 검증 자산을 감싸기만 한다:
  얼굴  = FaceRecognizer.identify        (face_recognizer.py, ArcFace 512d 1:N)
  제스처 = classify_gesture + Debouncer   (wasab_k3_mimic/gesture.py, 순수 함수)

엔드포인트:
  GET  /health          → {"ready":bool, "known":[이름...]}
  POST /identify        → body=JPEG bytes, 응답 {"name":str|null, "similarity":float,
                                                "gesture":str|null, "faces":int}
    · name    = 등록자 중 best(임계 미만이면 null)
    · gesture = 디바운스 확정된 순간에만 값, 아니면 null (연속 호출 전제)

실행 (venv + 순수 제스처 모듈 경로):
  PYTHONPATH=<repo>/src/wasab_k3_mimic \
  ~/face-recog/.venv/bin/python web_service.py --face-db <repo>/face-recog/face_db
"""
from __future__ import annotations

import argparse
import json
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import cv2
import numpy as np
import yaml

import mediapipe as mp

from face_recognizer import FaceRecognizer
from wasab_k3_mimic.gesture import GestureDebouncer, classify_gesture

MAX_BODY = 4 * 1024 * 1024          # 프레임 1장 상한(4MB) — 그 이상은 거부
GESTURE_STABLE_FRAMES = 3           # perception_node 기본값과 동일


class Judge:
    """얼굴+제스처 판정기. 한 프레임에서 둘 다 뽑는다(perception_node 와 동일 구성)."""

    def __init__(self, rec: FaceRecognizer, stable_frames: int = GESTURE_STABLE_FRAMES):
        self._rec = rec
        self._hands = mp.solutions.hands.Hands(
            max_num_hands=1, model_complexity=0,
            min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self._debounce = GestureDebouncer(stable_frames)

    def judge(self, frame_bgr) -> dict:
        out = {"name": None, "similarity": 0.0, "gesture": None, "faces": 0}

        # 얼굴 — 등록자 중 유사도 최고 1명
        if self._rec.is_ready:
            results = self._rec.identify(frame_bgr)
            out["faces"] = len(results)
            known = [r for r in results if r.is_known]
            if known:
                best = max(known, key=lambda r: r.similarity)
                out["name"] = best.name
                out["similarity"] = round(float(best.similarity), 4)

        # 제스처 — 손 1개, 확정된 순간에만 값
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        hres = self._hands.process(rgb)
        raw = None
        if hres.multi_hand_landmarks:
            pts = [(p.x, p.y) for p in hres.multi_hand_landmarks[0].landmark]
            raw = classify_gesture(pts)
        out["gesture"] = self._debounce.update(raw)
        return out


def _make_handler(judge: Judge, rec: FaceRecognizer):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _send(self, code: int, payload: dict):
            body = json.dumps(payload).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path != "/health":
                self._send(404, {"error": "not found"})
                return
            # known_names 는 임베딩 행마다 1개(이미지 수만큼 중복) → 사람 단위로 정리
            self._send(200, {"ready": bool(rec.is_ready),
                             "known": sorted(set(rec.known_names))})

        def do_POST(self):
            if self.path != "/identify":
                self._send(404, {"error": "not found"})
                return
            try:
                n = int(self.headers.get("Content-Length", 0))
            except ValueError:
                n = 0
            if n <= 0 or n > MAX_BODY:
                self._send(400, {"error": "bad content-length"})
                return
            data = self.rfile.read(n)
            frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
            if frame is None:
                self._send(400, {"error": "decode failed"})
                return
            try:
                self._send(200, judge.judge(frame))
            except Exception as e:                       # 판정 실패해도 서비스는 유지
                self._send(500, {"error": "%s: %s" % (type(e).__name__, e)})

        def log_message(self, fmt, *args):               # 접근로그 억제(프레임마다 호출됨)
            pass

    return Handler


def main():
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser(description="웹앱용 얼굴·제스처 판정 HTTP 서비스")
    # p.add_argument("--face-db", default=str(here / "face_db"), help="face_db 디렉토리")
    p.add_argument("--face-db", default=str(here.parent.parent / "FaceDB"), help="face_db 디렉토리")
    p.add_argument("--config", default=str(here / "config.yaml"), help="face-recog config.yaml")
    p.add_argument("--host", default="127.0.0.1", help="바인드 주소 (기본 127.0.0.1)")
    p.add_argument("--port", type=int, default=8091, help="포트 (기본 8091)")
    try:
        import argcomplete
        argcomplete.autocomplete(p)
    except ImportError:
        pass
    args = p.parse_args()

    cfg = {}
    if Path(args.config).exists():
        with open(args.config, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f) or {}
    model = cfg.get("model", {})
    reco = cfg.get("recognition", {})

    rec = FaceRecognizer(
        face_db_dir=args.face_db,
        tolerance=reco.get("tolerance", 0.40),
        min_face_size=reco.get("min_face_size", 40),
        model_name=model.get("name", "buffalo_sc"),
        det_size=model.get("det_size", 480),
        providers=model.get("providers"),
    )
    if not rec.is_ready:
        print("[web_service] 경고: 모델/등록자 미준비 — face_db 확인: %s" % args.face_db,
              file=sys.stderr)
    print("[web_service] 등록자 %d명: %s" % (len(rec.known_names), ", ".join(rec.known_names)))

    judge = Judge(rec)
    srv = ThreadingHTTPServer((args.host, args.port), _make_handler(judge, rec))
    print("[web_service] http://%s:%d  (POST /identify, GET /health)" % (args.host, args.port))
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()


if __name__ == "__main__":
    main()
