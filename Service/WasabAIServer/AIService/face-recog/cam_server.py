#!/usr/bin/env python3
"""
cam_server.py — JetCobot(RPi5)에서 /dev/video0 를 UDP로 AI Service(PC)에 송출.

SW Architecture 다이어그램의 WaSaBArm: Camera→Streamer→UDP→AI Service 경로.
프레임을 JPEG로 인코딩해 청크(각 앞에 4B 헤더: frame_id/chunk_index/total_chunks)로
UDP 유니캐스트 전송한다. 재전송 없음(fire-and-forget) — 청크 유실 시 해당 프레임만 버려짐.
수신측 프로토콜은 face-recog/udp_stream.py 와 반드시 동일해야 한다(양쪽 독립 배포이므로 임포트 공유 안 함).

배치/실행 (JetCobot 측):
    python3 cam_server.py --camera 0 --target 192.168.2.6 --port 8090
PC 측 수신: perception_node.py/register.py/identity_publisher.py --udp-port 8090
"""
from __future__ import annotations

import argparse
import socket
import struct
import time

import cv2

_HEADER = struct.Struct("!HBB")   # frame_id(2B) + chunk_index(1B) + total_chunks(1B)
_MAX_CHUNK = 1400                 # Ethernet MTU(1500) - IP/UDP/헤더 여유


def _send_frame(sock, target, frame_id, jpg_bytes):
    chunks = [jpg_bytes[i:i + _MAX_CHUNK] for i in range(0, len(jpg_bytes), _MAX_CHUNK)]
    total = len(chunks)
    for idx, chunk in enumerate(chunks):
        sock.sendto(_HEADER.pack(frame_id, idx, total) + chunk, target)


def _stream_loop(cam_index: int, w: int, h: int, sock, target) -> None:
    # 카메라가 준비될 때까지 재시도하고, 끊기면(USB 드롭/재기동 직후 busy) 자동 재오픈한다.
    frame_id = 0
    while True:
        cap = cv2.VideoCapture(cam_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        if not cap.isOpened():
            print(f"[cam_server] 카메라 {cam_index} 열기 실패 — 1.5s 후 재시도", flush=True)
            cap.release()
            time.sleep(1.5)
            continue
        print(f"[cam_server] 카메라 {cam_index} 열림 ({w}x{h}) → UDP {target[0]}:{target[1]}", flush=True)
        fails = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                fails += 1
                if fails >= 30:   # ~1.5s 연속 실패 → 카메라 재오픈
                    print("[cam_server] 프레임 수신 실패 지속 — 카메라 재오픈", flush=True)
                    break
                time.sleep(0.05)
                continue
            fails = 0
            ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 50])
            if not ok:
                continue
            _send_frame(sock, target, frame_id, jpg.tobytes())
            frame_id = (frame_id + 1) % 65536
            time.sleep(0.1)  # ~10fps 상한
        cap.release()
        time.sleep(1.0)


def main() -> None:
    p = argparse.ArgumentParser(description="JetCobot UDP 카메라 스트리머")
    p.add_argument("--camera", type=int, default=0, help="V4L2 카메라 인덱스 (기본 0)")
    p.add_argument("--target", default="192.168.2.6", help="수신측(PC, AI Service) IP")
    p.add_argument("--port", type=int, default=8090, help="UDP 포트 (기본 8090)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    target = (args.target, args.port)
    print(f"[cam_server] UDP → {target[0]}:{target[1]} (Ctrl+C 종료)")
    try:
        _stream_loop(args.camera, args.width, args.height, sock, target)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
