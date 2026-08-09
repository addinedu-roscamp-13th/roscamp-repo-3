"""WebService 엔트리포인트 (glue). ROS 미연결 시 mock 소스로 데모 기동 가능.

실 연동 시 MockSource/LogSink 자리에 rclpy 브리지(구독/발행)를 주입한다(spec §10.3).
실행: .venv/bin/python -m uvicorn wasab_web_service.main:app --port 8100
"""
import logging
import os

from . import auth_bootstrap, roles, server, state
from .auth_store import AuthStore
from .face_client import FaceClient
from .login_throttle import LoginThrottle
from .agent_runner import AgentRunner
from .arm_client import ArmClient
from .patrol_runner import PatrolRunner
from .session_store import SessionStore

ROBOTS_YAML = os.environ.get(
    "WASAB_ROBOTS_YAML",
    os.path.join(os.path.dirname(__file__), "..", "config", "robots.yaml"),
)
RUN_PATROL_SH = os.path.join(
    os.path.dirname(__file__), "..", "..", "scripts", "run_patrol.sh")
TEACHERS_JSON = os.path.join(os.path.dirname(__file__), "data", "teachers.json")


class MockSource:
    """robots.yaml 실 구성 + 데모 heartbeat → fleet UI 모델. (rclpy 브리지 대체용)"""
    def __init__(self, robots_yaml):
        self._config = roles.load_robots(robots_yaml)

    def fleet(self):
        # 데모 pose는 map11 범위(x∈[-0.191,1.909], y∈[-0.207,1.033]) 안 값으로.
        demo_hb = {
            50: {"id": 50, "mode": "patrol", "battery_v": 7.4,
                 "x": 1.60, "y": 0.05, "yaw": 90.0},
            87: {"id": 87, "mode": "assist", "battery_v": 7.1,
                 "x": 0.40, "y": 0.80, "yaw": 0.0},
            44: {"id": 44, "mode": "idle", "battery_v": 7.3,
                 "x": 0.95, "y": 0.45, "yaw": 45.0},
        }
        return state.fleet_view(self._config, demo_hb)


class LogSink:
    """명령 발행 대체 — 콘솔에 로그만 남긴다."""
    def publish(self, topic, payload):
        print(f"[cmd] {topic}  {payload}", flush=True)


def _setup_audit_log():
    """인증 감사로그(wasab.audit)를 uvicorn 로그레벨과 무관하게 남긴다.
    (--log-level warning 으로 기동하면 INFO 가 막혀 기록이 사라진다.)"""
    lg = logging.getLogger("wasab.audit")
    if lg.handlers:
        return
    h = logging.StreamHandler()
    h.setFormatter(logging.Formatter("%(asctime)s [audit] %(message)s"))
    lg.addHandler(h)
    lg.setLevel(logging.INFO)
    lg.propagate = False


def build_app():
    """WASAB_WEBSERVICE_ROS=1 이면 실 rclpy 브리지, 아니면 mock(데모)."""
    _setup_audit_log()
    patrol = PatrolRunner(roles.load_robots(ROBOTS_YAML), RUN_PATROL_SH)
    agent = AgentRunner(roles.load_robots(ROBOTS_YAML),
                        console_domain=int(os.environ.get("WASAB_CONSOLE_DOMAIN", "50")))
    auth = AuthStore(TEACHERS_JSON)
    auth_bootstrap.bootstrap_admin(auth, os.environ)
    session = SessionStore()
    throttle = LoginThrottle()
    origins = auth_bootstrap.allowed_origins(os.environ)
    secure = os.environ.get("WASAB_WEBAPP_SECURE_COOKIES", "1") != "0"   # HTTP dev만 "0"
    auth_bootstrap.check_cookie_security(secure, origins)                # insecure는 loopback origin만
    # 얼굴 판정은 face-recog 격리 venv 의 web_service(기본 :8091)가 담당 — 미기동이면 얼굴로그인만 503
    face = FaceClient(os.environ.get("WASAB_FACE_URL", "http://127.0.0.1:8091"))
    arm = ArmClient(os.environ.get("WASAB_ARM_API_URL", "http://192.168.2.8:8000"))
    if os.environ.get("WASAB_WEBSERVICE_ROS", "0") == "1":
        from .fleet_store import FleetStore
        from .ros_bridge import RosBridge
        from .camera_store import CameraStore, CameraService
        from .camera_bridge import CameraBridge
        cfg = roles.load_robots(ROBOTS_YAML)
        store = FleetStore(cfg)
        bridge = RosBridge(store, console_domain=int(os.environ.get("WASAB_CONSOLE_DOMAIN", "50")))
        bridge.start()
        cam_store = CameraStore()                       # 콘솔이 5006으로 중계한 도킹영상(B)
        cam_bridge = CameraBridge(cam_store)
        cam_bridge.start()
        cctv_store = CameraStore()                      # 콘솔이 5007로 중계한 천장 CCTV(topview)
        cctv_bridge = CameraBridge(cctv_store, port=5007)
        cctv_bridge.start()
        app = server.create_app(bridge, bridge,
                                camera=CameraService(cam_store, cfg), patrol=patrol, agent=agent,
                                cctv=cctv_store, face=face,
                                arm=arm,
                                auth=auth, session=session, throttle=throttle,
                                allowed_origins=origins, secure_cookies=secure)

        @app.on_event("shutdown")                      # uvicorn 종료 시 정리
        def _shutdown_bridge():
            bridge.stop()
            cam_bridge.stop()
            cctv_bridge.stop()
            patrol.stop_all()

        return app
    app = server.create_app(MockSource(ROBOTS_YAML), LogSink(), patrol=patrol, agent=agent,
                            face=face,
                            arm=arm,
                            auth=auth, session=session, throttle=throttle,
                            allowed_origins=origins, secure_cookies=secure)

    @app.on_event("shutdown")
    def _shutdown_patrol():
        patrol.stop_all()

    return app


app = build_app()

# 프론트 정적 서빙 — /api·/ws 라우트는 create_app에서 먼저 등록되어 우선함
from fastapi.staticfiles import StaticFiles

FRONTEND_DIR = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "..", "..",
    "UI", "mobile", "user_gui", "frontend",
))
app.mount("/", StaticFiles(directory=FRONTEND_DIR, html=True), name="frontend")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8100)
