"""테스트용 인증 클라이언트 빌더 — 미들웨어(Task5) 적용 후 모든 테스트가 세션·Origin을 갖도록."""
import os
import tempfile

from fastapi.testclient import TestClient

from wasab_web_service import server
from wasab_web_service.auth_store import AuthStore
from wasab_web_service.session_store import SessionStore
from wasab_web_service.login_throttle import LoginThrottle

TEST_ORIGIN = "http://testserver"


class _FakeSource:
    def __init__(self, data=None): self._d = data if data is not None else []
    def fleet(self): return self._d


class _FakeSink:
    def __init__(self): self.published = []
    def publish(self, topic, payload): self.published.append((topic, payload))


def _build(source, sink, camera, patrol, arm=None):
    auth = AuthStore(os.path.join(tempfile.mkdtemp(), "t.json"), iterations=1000)
    auth.add("admin1", "password1", "admin")
    session = SessionStore()
    app = server.create_app(source, sink, camera=camera, patrol=patrol, arm=arm,
                            auth=auth, session=session,
                            throttle=LoginThrottle(sleeper=lambda d: None),
                            allowed_origins=(TEST_ORIGIN,), secure_cookies=False)
    return app, auth, session


def make_app(source=None, sink=None, camera=None, patrol=None, arm=None):
    """admin1(password1) 시드된 앱 인스턴스 반환. 같은 앱에 여러 클라이언트를 붙일 때 사용."""
    app, _, _ = _build(source or _FakeSource(), sink or _FakeSink(), camera, patrol, arm)
    return app


def login_client(app, uid="admin1", pw="password1"):
    c = TestClient(app)
    c.headers.update({"origin": TEST_ORIGIN})
    c.post("/api/login", json={"id": uid, "password": pw})
    return c


def unauth_client(source=None, sink=None, camera=None, patrol=None):
    c = TestClient(make_app(source, sink, camera, patrol))
    c.headers.update({"origin": TEST_ORIGIN})
    return c


def authed_client(source=None, sink=None, camera=None, patrol=None, arm=None):
    return login_client(make_app(source, sink, camera, patrol, arm))
