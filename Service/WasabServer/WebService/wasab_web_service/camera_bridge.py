"""로컬 UDP 수신 브리지 — 콘솔이 중계한 카메라 프레임을 CameraStore로 흘려넣는다. P6(B).

콘솔 UdpCameraWorker(forward_addr=127.0.0.1:5006)가 보낸 '<ip>|<jpeg>' datagram을
5006에서 받아 store.ingest한다. rclpy와 무관(순수 UDP)하므로 ros 브리지와 분리.
sock_factory 주입으로 소켓 없이 로직 확인 가능(테스트).
"""
import socket
import threading

FORWARD_PORT = 5006


def _default_socket(port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.settimeout(1.0)              # 종료 시 recvfrom 무한 블로킹 방지
    return s


class CameraBridge:
    def __init__(self, store, port=FORWARD_PORT, sock_factory=None):
        self._store = store
        self._port = port
        self._factory = sock_factory or _default_socket
        self._sock = None
        self._thread = None
        self._running = False

    def start(self):
        if self._thread is not None:
            return
        self._sock = self._factory(self._port)
        self._running = True
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while self._running:
            try:
                data, _addr = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception:
                continue
            self._store.ingest(data)

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._sock is not None:
            try:
                self._sock.close()
            except Exception:
                pass
            self._sock = None
