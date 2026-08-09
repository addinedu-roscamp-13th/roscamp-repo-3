"""순수 카메라 프레임 저장/조회 (ROS/소켓 무관). P6 카메라 중계(B).

콘솔 UdpCameraWorker가 "<sender-ip>|<jpeg>" datagram을 로컬 5006으로 중계하면
CameraBridge가 ingest한다. robot-ip별 최신 JPEG만 보관(seq로 신규 판별).
CameraService가 robot_id→ip(robots.yaml) 매핑으로 프레임을 제공한다.

⚠ 스레드 안전: ingest()는 UDP 수신 스레드, frame()은 WS 스레드에서 호출 → Lock 보호.
"""
import threading


class CameraStore:
    def __init__(self):
        self._latest = {}          # ip -> (seq, jpeg_bytes)
        self._seq = 0
        self._lock = threading.Lock()

    def ingest(self, datagram):
        """중계 datagram '<ip>|<jpeg>' 파싱 → 최신 프레임 갱신. 형식 불량은 무시."""
        i = datagram.find(b"|")
        if i <= 0:
            return
        ip = datagram[:i].decode("ascii", "ignore")
        jpeg = datagram[i + 1:]
        if not jpeg:
            return
        with self._lock:
            self._seq += 1
            self._latest[ip] = (self._seq, jpeg)

    def latest(self, ip):
        """(seq, jpeg) 또는 None."""
        with self._lock:
            return self._latest.get(ip)


class CameraService:
    """robot_id → 최신 프레임. WS 핸들러가 create_app(camera=...)로 주입받아 사용."""
    def __init__(self, store, config):
        self._store = store
        self._ip = {rid: r.get("ip") for rid, r in config.items()}

    def frame(self, robot_id):
        """(seq, jpeg) 또는 None. 미지의 robot_id/ip면 None."""
        ip = self._ip.get(robot_id)
        if ip is None:
            return None
        return self._store.latest(ip)
