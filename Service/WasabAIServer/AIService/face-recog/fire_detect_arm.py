#!/usr/bin/env python3
"""
fire_detect_arm.py — JetCobot에서 카메라를 직접 읽어 화재감지까지 로컬로 끝내는 버전.

기존 방식(cam_server.py가 PC로 영상을 UDP 송출 → PC의 fire_detect_node.py가 그걸 받아서 분석)은
로봇팔에 카메라를 장착해 WiFi로 쏠 때 청크 유실/지연으로 감지가 잘 안 되는 문제가 있었다
(udp_stream.py는 재전송 없이 청크 하나만 빠져도 프레임 전체를 버림). 이 스크립트는 감지 자체를
JetCobot에서 로컬로 수행해 그 구간을 없앤다 — 네트워크로는 판정 결과(ROS 이벤트, 몇 바이트)만 나간다.

화면은 지금까지처럼 PC에서 볼 수 있도록, cam_server.py와 동일한 프로토콜(udp_stream.py 호환)로
계속 UDP 송출한다 — PC 쪽 표시 파이프라인은 바꿀 필요 없음. 카메라 장치는 한 프로세스만 열 수
있으므로 cam_server.py와 동시 실행하지 말 것(같은 --camera 인덱스를 놓고 충돌한다).

감지 알고리즘/FSM은 fire_detect_fsm.py의 detect_fire/FireResponseFSM/FireDetectNode/process_frame을
그대로 재사용한다 — 튜닝값이 두 파일에서 따로 관리되다 어긋나는 걸 막기 위함(2026-07-28).

이 저장소에서는 기존 듀얼암 GUI가 사용하는 PC 카메라용 fire_detect_node.py의 CLI 계약을
유지해야 하므로, 통합본 FSM을 fire_detect_fsm.py라는 이름으로 함께 배치한다.
"""
from __future__ import annotations

import argparse
import socket
import struct
import time

import cv2
import rclpy

import fire_detect_fsm as fdn
from fire_detect_fsm import FireDetectNode, process_frame

_HEADER = struct.Struct("!HBB")   # cam_server.py/udp_stream.py 와 동일 프로토콜(임포트 공유 안 함, 나란히 유지)
_MAX_CHUNK = 1400


def _send_frame(sock, target, frame_id, jpg_bytes):
    chunks = [jpg_bytes[i:i + _MAX_CHUNK] for i in range(0, len(jpg_bytes), _MAX_CHUNK)]
    total = len(chunks)
    for idx, chunk in enumerate(chunks):
        sock.sendto(_HEADER.pack(frame_id, idx, total) + chunk, target)


def _open_camera(cam_index: int, w: int, h: int):
    cap = cv2.VideoCapture(cam_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    return cap


def main() -> None:
    parser = argparse.ArgumentParser(description="JetCobot 로컬 화재감지(카메라 직접 캡처)")
    parser.add_argument("--camera", type=int, default=0, help="V4L2 카메라 인덱스")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--flip", action="store_true", help="좌우 반전(cam_server.py --flip과 동일)")
    parser.add_argument("--target", default=None,
                        help="PC로 화면을 중계할 IP. 생략하면 화면 중계 없이 감지만 수행")
    parser.add_argument("--port", type=int, default=8091, help="화면 중계 UDP 포트")
    parser.add_argument("--fire-topic", default="/wasab/k3/fire")
    parser.add_argument("--arm-still-topic", default="/wasab/k3/arm_still")
    parser.add_argument("--console-domain", type=int, default=50)
    parser.add_argument("--robot-id", default="WaSaBArm")
    parser.add_argument("--zone", default="D")
    parser.add_argument("--rate", type=float, default=8.0, help="처리 주기(Hz)")
    parser.add_argument("--std-min", type=float, default=fdn.STD_MIN)
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()
    fdn.STD_MIN = args.std_min

    stream_sock = None
    target = None
    if args.target:
        stream_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        target = (args.target, args.port)
        print(f"[fire_detect_arm] 화면 중계 → UDP {target[0]}:{target[1]}", flush=True)
    else:
        print("[fire_detect_arm] --target 미지정 — 화면 중계 없이 감지만 수행", flush=True)

    rclpy.init()
    node = FireDetectNode(args.fire_topic, args.console_domain, args.arm_still_topic,
                          robot_id=args.robot_id, zone=args.zone)
    print(f"[fire_detect_arm] 카메라 {args.camera} 로컬 분석 → {args.fire_topic} "
          f"(STD_MIN={fdn.STD_MIN}, console_domain={args.console_domain})", flush=True)

    buf: list = []
    votes: list = []
    areas: list = []
    period = 1.0 / args.rate
    frame_id = 0
    cap = None
    fails = 0
    try:
        while rclpy.ok():
            t0 = time.time()
            if cap is None or not cap.isOpened():
                cap = _open_camera(args.camera, args.width, args.height)
                if not cap.isOpened():
                    print("[fire_detect_arm] 카메라 열기 실패 — 1.5s 후 재시도", flush=True)
                    cap.release()
                    cap = None
                    time.sleep(1.5)
                    continue
                fails = 0
                print(f"[fire_detect_arm] 카메라 {args.camera} 열림 ({args.width}x{args.height})",
                      flush=True)

            ok, frame = cap.read()
            if not ok:
                fails += 1
                if fails >= 30:   # ~1.5s 연속 실패 → 재오픈(cam_server.py와 동일 정책)
                    print("[fire_detect_arm] 프레임 수신 실패 지속 — 카메라 재오픈", flush=True)
                    cap.release()
                    cap = None
                time.sleep(0.05)
                continue
            fails = 0
            if args.flip:
                frame = cv2.flip(frame, 1)

            buf, votes, areas, bbox = process_frame(node, frame, buf, votes, areas,
                                                      debug=args.debug)

            if stream_sock is not None:
                # PC에서 "인식되는지" 눈으로 보게 후보 박스/상태를 화면에 그려서 내보낸다.
                disp = frame if bbox is None else frame.copy()
                if bbox is not None:
                    x, y, w, h = bbox
                    cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 80, 220), 2)
                cv2.putText(disp, f"state={node.fsm.state}", (10, 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 80, 220), 2)
                jok, jpg = cv2.imencode(".jpg", disp, [cv2.IMWRITE_JPEG_QUALITY, 50])
                if jok:
                    _send_frame(stream_sock, target, frame_id, jpg.tobytes())
                    frame_id = (frame_id + 1) % 65536

            rclpy.spin_once(node, timeout_sec=0.0)
            node.spin_console()
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        if cap is not None:
            cap.release()
        node.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
