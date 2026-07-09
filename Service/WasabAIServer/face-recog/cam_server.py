#!/usr/bin/env python3
"""
cam_server.py — JetCobot(RPi5)에서 /dev/video0 를 MJPEG HTTP 로 송출.

노트북 ~/face-recog 의 run.py/register.py 가 --source 로 이 스트림을 받는다.
시스템 cv2 만 사용하므로 RPi5 에 추가 설치 불필요(insightface/onnxruntime 불요).

배치/실행 (JetCobot 측):
    python3 cam_server.py --camera 0 --port 8090
노트북 측:
    python run.py --source http://192.168.0.86:8090/stream
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

_latest = {"frame": None}
_lock = threading.Lock()


def _grab_loop(cam_index: int, w: int, h: int) -> None:
    # 카메라가 준비될 때까지 재시도하고, 끊기면(USB 드롭/재기동 직후 busy) 자동 재오픈한다.
    # 과거: open 실패 시 SystemExit, read 실패 시 무한 대기 → _latest 가 None 으로 굳어 frozen.
    while True:
        cap = cv2.VideoCapture(cam_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not cap.isOpened():
            print(f"[cam_server] 카메라 {cam_index} 열기 실패 — 1.5s 후 재시도", flush=True)
            cap.release()
            time.sleep(1.5)
            continue
        print(f"[cam_server] 카메라 {cam_index} 열림 ({w}x{h})", flush=True)
        fails = 0
        while True:
            ok, frame = cap.read()
            if ok:
                with _lock:
                    _latest["frame"] = frame
                fails = 0
            else:
                fails += 1
                if fails >= 30:   # ~1.5s 연속 실패 → 카메라 재오픈
                    print("[cam_server] 프레임 수신 실패 지속 — 카메라 재오픈", flush=True)
                    break
                time.sleep(0.05)
        cap.release()
        time.sleep(1.0)


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path not in ("/stream", "/", "/stream.mjpg"):
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _lock:
                    frame = _latest["frame"]
                if frame is None:
                    time.sleep(0.02)
                    continue
                ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
                if not ok:
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(0.1)  # ~10fps 상한 (브라우저 버퍼링/지연 감소)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def log_message(self, *_):  # 접속 로그 억제
        pass


def main() -> None:
    p = argparse.ArgumentParser(description="JetCobot MJPEG 카메라 서버")
    p.add_argument("--camera", type=int, default=0, help="V4L2 카메라 인덱스 (기본 0)")
    p.add_argument("--port", type=int, default=8090, help="HTTP 포트 (기본 8090)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    args = p.parse_args()

    threading.Thread(target=_grab_loop, args=(args.camera, args.width, args.height),
                     daemon=True).start()
    srv = ThreadingHTTPServer(("0.0.0.0", args.port), _Handler)
    print(f"[cam_server] MJPEG → http://0.0.0.0:{args.port}/stream  (Ctrl+C 종료)")
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
