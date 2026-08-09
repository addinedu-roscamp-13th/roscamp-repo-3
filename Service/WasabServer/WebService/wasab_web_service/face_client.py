"""얼굴·제스처 판정 서비스(face-recog web_service) HTTP 클라이언트.

웹앱 백엔드는 insightface/mediapipe 를 직접 import 할 수 없다(venv 분리).
판정은 face-recog 격리 venv 의 web_service 가 하고, 여기서는 HTTP 로 물어보기만 한다.

표준 urllib 만 사용(새 의존성 0). 서비스가 죽어 있어도 웹앱은 살아야 하므로
실패는 예외 대신 None 으로 돌려주고, 호출측이 "얼굴 인증 불가"로 처리한다.
"""
import json
import urllib.error
import urllib.request

DEFAULT_URL = "http://127.0.0.1:8091"
DEFAULT_TIMEOUT = 5.0


class FaceClient:
    def __init__(self, base_url=DEFAULT_URL, timeout=DEFAULT_TIMEOUT, opener=None):
        self._base = base_url.rstrip("/")
        self._timeout = timeout
        self._opener = opener or urllib.request.urlopen   # 테스트 주입용

    def identify(self, jpeg_bytes):
        """JPEG 한 장 → {"name":str|None, "similarity":float, "gesture":str|None, "faces":int}.
        서비스 미기동·타임아웃·오류면 None."""
        req = urllib.request.Request(
            self._base + "/identify", data=jpeg_bytes,
            headers={"Content-Type": "image/jpeg"}, method="POST")
        try:
            with self._opener(req, timeout=self._timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError):
            return None

    def health(self):
        """{"ready":bool, "known":[...]} 또는 None(미기동)."""
        req = urllib.request.Request(self._base + "/health", method="GET")
        try:
            with self._opener(req, timeout=self._timeout) as r:
                return json.loads(r.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError):
            return None
