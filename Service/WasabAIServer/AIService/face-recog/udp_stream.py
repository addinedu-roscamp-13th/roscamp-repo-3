#!/usr/bin/env python3
"""
udp_stream.py — cam_server.py(JetCobot)가 청크 단위로 보내는 UDP JPEG 프레임 수신.

프로토콜(양쪽 고정, cam_server.py 와 반드시 동일해야 함 — 서로 다른 장비에 독립 배포되므로
임포트로 공유하지 않고 두 파일에 나란히 구현되어 있음):
    헤더 4B(네트워크 바이트오더) = frame_id(H,2B) + chunk_index(B,1B) + total_chunks(B,1B)
    이후 payload(JPEG 바이트 조각, 최대 MAX_CHUNK 바이트)
재전송 없음(fire-and-forget) — 청크 하나라도 유실되면 그 프레임은 버려지고 다음 프레임으로 넘어간다.
"""
from __future__ import annotations

import socket
import struct
import threading

import cv2
import numpy as np

HEADER = struct.Struct("!HBB")
MAX_CHUNK = 1400


class UDPFrameReceiver:
    """cv2.VideoCapture 와 동일한 인터페이스(read/release/isOpened)의 UDP 프레임 수신기."""

    def __init__(self, port: int, bind_addr: str = "0.0.0.0"):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind((bind_addr, port))
        self._frame = None
        self._lock = threading.Lock()
        self._run = True
        self._cur_id = None
        self._chunks = {}
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._run:
            try:
                packet, _ = self._sock.recvfrom(65536)
            except OSError:
                break
            if len(packet) < HEADER.size:
                continue
            frame_id, idx, total = HEADER.unpack_from(packet, 0)
            if frame_id != self._cur_id:
                self._cur_id = frame_id
                self._chunks = {}
            self._chunks[idx] = packet[HEADER.size:]
            if len(self._chunks) == total:
                data = b"".join(self._chunks[i] for i in range(total))
                frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                self._chunks = {}
                if frame is not None:
                    with self._lock:
                        self._frame = frame

    def isOpened(self) -> bool:
        return True

    def read(self):
        """가장 최근 완성 프레임 (ok, frame_copy). 아직 수신 전이면 (False, None)."""
        with self._lock:
            if self._frame is None:
                return False, None
            return True, self._frame.copy()

    def release(self):
        self._run = False
        self._sock.close()
        self._thread.join(timeout=1.0)
