#!/usr/bin/env python3
"""view_stream.py — cam_server.py/fire_detect_arm.py가 UDP로 보내는 영상을 그냥 화면에 띄우기만 함.

감지가 JetCobot 로컬로 옮겨간 뒤(fire_detect_arm.py), PC 쪽엔 영상을 받아서 "보기만" 하는
프로그램이 없어서 만들었다 — udp_stream.py의 UDPFrameReceiver(재사용, 프로토콜 동일)만 쓴다.
"""
import argparse

import cv2

from udp_stream import UDPFrameReceiver


def main():
    p = argparse.ArgumentParser(description="JetCobot UDP 영상 뷰어(감지 없음, 화면표시 전용)")
    p.add_argument("--port", type=int, default=8091)
    args = p.parse_args()

    recv = UDPFrameReceiver(args.port)
    cv2.namedWindow("JetCobot 화면", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("JetCobot 화면", 960, 720)   # 기본 창이 너무 작게 뜨는 WM 대응, 강제로 크게
    print(f"[view_stream] UDP :{args.port} 수신 대기 — 창에서 q 누르면 종료", flush=True)
    try:
        while True:
            ok, frame = recv.read()
            if ok:
                cv2.imshow("JetCobot 화면", frame)
            if cv2.waitKey(30) & 0xFF == ord('q'):
                break
    finally:
        recv.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
