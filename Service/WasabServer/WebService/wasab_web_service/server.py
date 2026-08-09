"""FastAPI 어댑터 앱 (spec §10.3).

기존 콘솔 계약을 웹으로 노출하는 얇은 어댑터. ROS 접근은 DI로 주입한다:
  - source: `.fleet()` → fleet UI 모델 리스트 (state.fleet_view 결과)
  - sink:   `.publish(topic, payload)` → 콘솔 명령 발행 (rclpy 브리지)
이 분리로 앱 로직은 ROS 없이 pytest 가능(OpService 패턴).
"""
import asyncio
import logging
import threading
from typing import Literal

from fastapi import FastAPI, HTTPException, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from . import commands, event_store
from .arm_client import ArmClientError
from .auth_store import AuthError

_audit = logging.getLogger("wasab.audit")


class LoginIn(BaseModel):
    id: str
    password: str


class ModeCmd(BaseModel):
    robot_id: int
    mode: str


class EventIn(BaseModel):
    t: str
    robot: str = ""
    ip: str = ""
    mode: str = ""
    type: str = ""
    result: str = ""
    status: str = ""
    pos: str = ""


class DockCmd(BaseModel):
    robot_id: int
    tag_id: int


class RelocalizeCmd(BaseModel):
    robot_id: int
    action: str


class PatrolCmd(BaseModel):
    robot_id: int


class AgentCmd(BaseModel):
    robot_id: int
    action: str            # start | stop


class EstopCmd(BaseModel):
    target: int | Literal["all"]        # 그 외 입력(dict/list/null)은 pydantic이 422로 거부
    active: bool

    @property
    def norm_target(self):
        return self.target              # 이미 int 또는 "all"


class TeacherIn(BaseModel):
    id: str
    password: str
    role: str = "teacher"


class ArmCommandIn(BaseModel):
    arm_id: Literal["left", "right", "dual"]
    command: str


class ArmFeatureIn(BaseModel):
    arm_id: Literal["left", "right"]
    feature: Literal["fire-detect", "face-recognition", "tracking"]


class ArmFireResponseIn(BaseModel):
    response: Literal["yes", "no"]


_ARM_COMMANDS = {
    "home", "stop", "pick-place", "pick", "place", "restock", "recycle",
    "help", "pose", "gripper", "gesture-on", "gesture-off",
}
_DUAL_COMMANDS = {"home", "stop", "gift-giving"}


def create_app(source, sink, camera=None, patrol=None, agent=None, cctv=None, face=None, arm=None, *,
               auth=None, session=None, throttle=None,
               allowed_origins=(), secure_cookies=True):
    app = FastAPI(title="WaSaB User Webapp — WebService adapter")
    cookie_name = "__Host-wasab_session" if secure_cookies else "wasab_session"

    def _current(request):
        if session is None:
            return None
        sid = request.cookies.get(cookie_name)
        return session.validate(sid) if sid else None

    def _client_ip(request):
        return request.client.host if request.client else "-"

    _PUBLIC = {"/api/login", "/api/login/face", "/api/session"}
    _STATE_METHODS = ("POST", "PUT", "PATCH", "DELETE")

    def _origin_ok(headers):
        origin = headers.get("origin")
        return origin is not None and origin in allowed_origins   # 비면 거부(fail-closed)

    @app.middleware("http")
    async def _auth_mw(request: Request, call_next):
        path = request.url.path
        # 상태변경은 공개경로(로그인 등) 포함 Origin 검사 우선
        if path.startswith("/api/") and request.method in _STATE_METHODS and not _origin_ok(request.headers):
            return JSONResponse({"detail": "bad origin"}, status_code=403)
        if not path.startswith("/api/") or path in _PUBLIC:
            return await call_next(request)
        ctx = _current(request)
        if ctx is None:
            return JSONResponse({"detail": "unauthorized"}, status_code=401)
        request.state.user = ctx
        return await call_next(request)

    def _ws_authorized(ws: WebSocket):
        if session is None:
            return False
        sid = ws.cookies.get(cookie_name)
        if not (session.validate(sid) if sid else None):
            return False
        return _origin_ok(ws.headers)

    @app.post("/api/login")
    def login(body: LoginIn, request: Request, response: Response):
        ip = _client_ip(request)
        uid = body.id
        # 입력 길이 정규화: 형식 위반·없는 계정 모두 균일 401. 큰 입력이 PBKDF2/스로틀키/로그에 닿기 전 선차단.
        if not (1 <= len(uid) <= 64) or len(body.password) > 128:
            _audit.info("login result=fail ip=%s account=%r reason=badinput", ip, uid[:64])
            raise HTTPException(status_code=401, detail="invalid credentials")
        key = f"{ip}:{uid}"
        throttle.before_attempt(key)
        role = auth.verify(uid, body.password)
        if role is None:
            throttle.record_failure(key)
            _audit.info("login result=fail ip=%s account=%r", ip, uid)   # %r: 개행 등 이스케이프
            raise HTTPException(status_code=401, detail="invalid credentials")
        throttle.record_success(key)
        sid = session.create(uid, role)
        response.set_cookie(cookie_name, sid, httponly=True, secure=secure_cookies,
                            samesite="strict", path="/")
        _audit.info("login result=success ip=%s account=%r", ip, uid)
        return {"id": uid, "role": role}

    _FACE_MAX_BODY = 4 * 1024 * 1024      # 프레임 1장 상한

    @app.post("/api/login/face")
    async def login_face(request: Request, response: Response):
        """얼굴로 로그인(비밀번호 로그인과 OR). body = JPEG bytes.
        판정은 face-recog 서비스가 하고, 세션 발급 경로는 /api/login 과 동일."""
        if face is None or session is None:
            raise HTTPException(status_code=503, detail="face auth unavailable")
        ip = _client_ip(request)
        key = f"{ip}:face"                 # 계정을 모르므로 IP 기준 스로틀
        if throttle is not None:
            throttle.before_attempt(key)
        data = await request.body()
        if not (0 < len(data) <= _FACE_MAX_BODY):
            raise HTTPException(status_code=400, detail="bad image")
        res = face.identify(data)
        if res is None:                    # 서비스 미기동 — 실패 누적 대상 아님(사용자 잘못 아님)
            raise HTTPException(status_code=503, detail="face service unavailable")
        name = res.get("name")
        if not name:
            if throttle is not None:
                throttle.record_failure(key)
            _audit.info("login result=fail ip=%s via=face reason=unrecognized", ip)
            raise HTTPException(status_code=401, detail="face not recognized")
        if throttle is not None:
            throttle.record_success(key)
        role = "teacher"                   # 동명 계정이 있으면 그 권한을 따른다
        if auth is not None:
            for a in auth.list():
                if a["id"] == name:
                    role = a["role"]
                    break
        sid = session.create(name, role, face_verified=True, face_name=name)  # 얼굴로 들어왔으니 인증 완료
        response.set_cookie(cookie_name, sid, httponly=True, secure=secure_cookies,
                            samesite="strict", path="/")
        _audit.info("login result=success ip=%s account=%r via=face", ip, name)
        return {"id": name, "role": role, "via": "face",
                "similarity": res.get("similarity", 0.0)}

    @app.post("/api/face/verify")
    async def face_verify(request: Request):
        """계정(비밀번호) 로그인 세션을 얼굴 인증으로 승격. body = JPEG bytes.
        수업보조 지시는 이 인증을 선행해야 한다(얼굴 인증 AND 모션)."""
        if face is None or session is None:
            raise HTTPException(status_code=503, detail="face service unavailable")
        data = await request.body()
        if not (0 < len(data) <= _FACE_MAX_BODY):
            raise HTTPException(status_code=400, detail="bad image")
        res = face.identify(data)
        if res is None:
            raise HTTPException(status_code=503, detail="face service unavailable")
        name = res.get("name")
        if not name:
            _audit.info("face_verify result=fail ip=%s account=%r",
                        _client_ip(request), request.state.user.get("account_id"))
            raise HTTPException(status_code=401, detail="face not recognized")
        session.mark_face_verified(request.cookies.get(cookie_name), name)
        _audit.info("face_verify result=success ip=%s account=%r face=%r",
                    _client_ip(request), request.state.user.get("account_id"), name)
        return {"face_verified": True, "face_name": name,
                "similarity": res.get("similarity", 0.0)}

    @app.post("/api/face/identify")
    async def face_identify(request: Request):
        """로그인 이후 화면용 판정(얼굴+제스처). body = JPEG bytes.
        세션 필수(공개경로 아님) — 수업보조는 '얼굴 인증 AND 모션'이라 둘을 함께 돌려준다."""
        if face is None:
            raise HTTPException(status_code=503, detail="face service unavailable")
        data = await request.body()
        if not (0 < len(data) <= _FACE_MAX_BODY):
            raise HTTPException(status_code=400, detail="bad image")
        res = face.identify(data)
        if res is None:
            raise HTTPException(status_code=503, detail="face service unavailable")
        return res

    @app.post("/api/logout")
    def logout(request: Request, response: Response):
        sid = request.cookies.get(cookie_name)
        if sid and session is not None:
            session.logout(sid)
        response.delete_cookie(cookie_name, path="/")
        return {"ok": True}

    @app.get("/api/session")
    def get_session(request: Request):
        ctx = _current(request)
        if ctx is None:
            return {"authenticated": False}
        return {"authenticated": True, "id": ctx["account_id"], "role": ctx["role"],
                "face_verified": ctx.get("face_verified", False),
                "face_name": ctx.get("face_name")}

    def _require_admin(request: Request):
        u = getattr(request.state, "user", None)
        if not u or u["role"] != "admin":
            raise HTTPException(status_code=403, detail="admin only")
        return u

    @app.get("/api/teachers")
    def list_teachers(request: Request):
        _require_admin(request)
        return auth.list()

    @app.post("/api/teachers")
    def create_teacher(body: TeacherIn, request: Request):
        admin = _require_admin(request)
        try:
            auth.add(body.id, body.password, body.role)
        except AuthError as e:
            raise HTTPException(status_code=409 if str(e) == "exists" else 400, detail=str(e))
        _audit.info("teacher_create ip=%s admin=%s account=%s role=%s",
                    _client_ip(request), admin["account_id"], body.id, body.role)
        return {"ok": True, "id": body.id, "role": body.role}

    @app.delete("/api/teachers/{tid}")
    def delete_teacher(tid: str, request: Request):
        admin = _require_admin(request)
        try:
            auth.remove(tid)
        except AuthError as e:
            raise HTTPException(status_code=400, detail=str(e))
        session.revoke_account(tid)
        _audit.info("teacher_delete ip=%s admin=%s account=%s",
                    _client_ip(request), admin["account_id"], tid)
        return {"ok": True}

    def _publish(build):
        """빌더 실행 → sink 발행. ValueError는 400으로 변환."""
        try:
            topic, payload = build()
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        sink.publish(topic, payload)
        return {"topic": topic, "payload": payload}

    def _arm_call(call):
        if arm is None:
            raise HTTPException(status_code=503, detail="로봇팔 연동이 설정되지 않았습니다.")
        try:
            return call()
        except ArmClientError as exc:
            raise HTTPException(status_code=exc.status_code, detail=exc.detail) from exc

    @app.get("/api/fleet")
    def get_fleet():
        return source.fleet()

    @app.get("/api/arm/status")
    def get_arm_status(arm_id: Literal["left", "right", "dual"] = "left"):
        return _arm_call(lambda: arm.status(arm_id))

    @app.post("/api/arm/command")
    def post_arm_command(cmd: ArmCommandIn):
        allowed = _DUAL_COMMANDS if cmd.arm_id == "dual" else _ARM_COMMANDS
        if cmd.command not in allowed:
            raise HTTPException(status_code=400, detail="지원하지 않는 로봇팔 명령입니다.")
        if cmd.arm_id == "dual":
            if cmd.command == "home":
                return _arm_call(lambda: {
                    "left": arm.command("left", "home"),
                    "right": arm.command("right", "home"),
                })
            return _arm_call(lambda: arm.dual_command(cmd.command))
        return _arm_call(lambda: arm.command(cmd.arm_id, cmd.command))

    @app.post("/api/arm/feature")
    def post_arm_feature(cmd: ArmFeatureIn):
        return _arm_call(lambda: arm.toggle_feature(cmd.arm_id, cmd.feature))

    @app.get("/api/arm/fire-prompt")
    def get_arm_fire_prompt(arm_id: Literal["left", "right"] = "left"):
        return _arm_call(lambda: arm.fire_prompt(arm_id))

    @app.post("/api/arm/fire-response")
    def post_arm_fire_response(arm_id: Literal["left", "right"], body: ArmFireResponseIn):
        return _arm_call(lambda: arm.fire_response(arm_id, body.response))

    @app.get("/api/arm/face-prompt")
    def get_arm_face_prompt(arm_id: Literal["left", "right"] = "left"):
        return _arm_call(lambda: arm.face_prompt(arm_id))

    @app.post("/api/arm/face-prompt/ack")
    def acknowledge_arm_face_prompt(arm_id: Literal["left", "right"]):
        return _arm_call(lambda: arm.acknowledge_face_prompt(arm_id))

    @app.get("/api/arm/logs")
    def get_arm_logs(after_id: int = 0):
        return _arm_call(lambda: arm.logs(after_id))

    @app.get("/api/arm/camera")
    def get_arm_camera(arm_id: Literal["left", "right"] = "left"):
        data, media_type = _arm_call(lambda: arm.camera(arm_id))
        return Response(content=data, media_type=media_type)

    @app.get("/api/events")
    def get_events(limit: int = 5000):
        return event_store.recent(limit)

    @app.post("/api/events")
    def post_event(ev: EventIn):
        event_store.record({"t": ev.t, "robot": ev.robot, "ip": ev.ip, "mode": ev.mode,
                            "type": ev.type, "result": ev.result, "status": ev.status, "pos": ev.pos})
        return {"ok": True}

    @app.post("/api/cmd/mode")
    def post_cmd_mode(cmd: ModeCmd):
        return _publish(lambda: commands.cmd_mode(cmd.robot_id, cmd.mode))

    @app.post("/api/cmd/estop")
    def post_cmd_estop(cmd: EstopCmd):
        # 심플 긴급정지: /wasab/estop {target,active} 발행. agent가 target 필터·로봇 relay, base가 모터 0.
        result = _publish(lambda: commands.estop_msg(cmd.target, cmd.active))
        if not cmd.active:
            return result

        # 이동로봇 ESTOP 발행을 먼저 보장한 뒤 양쪽 팔에도 STOP을 전달한다.
        # 팔 서버 장애가 이동로봇의 긴급정지 응답까지 실패시키지 않도록 결과를 분리한다.
        arm_stop = {}
        if arm is None:
            arm_stop = {"left": "unavailable", "right": "unavailable"}
        else:
            for arm_id in ("left", "right"):
                try:
                    arm.command(arm_id, "stop")
                    arm_stop[arm_id] = "sent"
                except ArmClientError as exc:
                    arm_stop[arm_id] = f"failed: {exc.detail}"
                    _audit.warning("estop arm_stop=%s result=failed detail=%s", arm_id, exc.detail)
        result["arm_stop"] = arm_stop
        return result

    @app.post("/api/cmd/dock")
    def post_cmd_dock(cmd: DockCmd):
        return _publish(lambda: commands.dock_start(cmd.robot_id, cmd.tag_id))

    @app.post("/api/cmd/relocalize")
    def post_cmd_relocalize(cmd: RelocalizeCmd):
        return _publish(lambda: commands.relocalize(cmd.robot_id, cmd.action))

    @app.post("/api/cmd/agent")
    def post_cmd_agent(cmd: AgentCmd):
        if agent is None:
            raise HTTPException(status_code=503, detail="agent runner unavailable")
        if cmd.action not in ("start", "stop"):
            raise HTTPException(status_code=400, detail="action must be start|stop")
        try:
            res = agent.start(cmd.robot_id) if cmd.action == "start" else agent.stop(cmd.robot_id)
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        if not res.get("ok"):
            raise HTTPException(status_code=502,
                                detail=(res.get("err") or res.get("out") or "agent %s 실패" % cmd.action).strip())
        return {"ok": True, "action": cmd.action, "robot_id": cmd.robot_id}

    @app.post("/api/patrol/start")
    def post_patrol_start(cmd: PatrolCmd):
        if patrol is None:
            raise HTTPException(status_code=503, detail="patrol runner unavailable")
        try:
            patrol.start(cmd.robot_id)                 # 이미 실행 중이면 no-op
        except ValueError as e:
            raise HTTPException(status_code=400, detail=str(e))
        return {"ok": True, "running": True}

    @app.post("/api/patrol/stop")
    def post_patrol_stop(cmd: PatrolCmd):
        if patrol is None:
            raise HTTPException(status_code=503, detail="patrol runner unavailable")
        patrol.stop(cmd.robot_id)
        return {"ok": True, "running": False}

    @app.get("/api/patrol/status")
    def get_patrol_status(robot_id: int):
        # 서버 권위 상태 — 프론트가 선택 로봇의 실제 순찰 여부를 조회(새로고침/로봇전환 동기).
        if patrol is None:
            raise HTTPException(status_code=503, detail="patrol runner unavailable")
        return {"robot_id": robot_id, "running": patrol.is_running(robot_id)}

    @app.websocket("/ws/state")
    async def ws_state(ws: WebSocket):
        if not _ws_authorized(ws):
            await ws.close(code=1008)
            return
        await ws.accept()
        try:
            while True:
                await ws.send_json(source.fleet())     # 접속 즉시 + 1s 주기 push
                await asyncio.sleep(1.0)
        except WebSocketDisconnect:
            return

    @app.websocket("/ws/camera")
    async def ws_camera(ws: WebSocket):
        # 선택 로봇(robot=<id>)의 도킹 전방영상을 binary JPEG로 push. camera 미주입이면 무전송.
        if not _ws_authorized(ws):
            await ws.close(code=1008)
            return
        await ws.accept()
        try:
            rid = int(ws.query_params.get("robot"))
        except (TypeError, ValueError):
            await ws.close()
            return
        last_seq = None
        try:
            while True:
                rec = camera.frame(rid) if camera is not None else None
                if rec is not None and rec[0] != last_seq:
                    await ws.send_bytes(rec[1])         # 새 프레임만 전송
                    last_seq = rec[0]
                await asyncio.sleep(0.066)              # ~15fps 상한
        except WebSocketDisconnect:
            return

    @app.websocket("/ws/cctv")
    async def ws_cctv(ws: WebSocket):
        # 천장 CCTV(topview) binary JPEG push. 콘솔이 5007로 중계한 프레임(cctv 미주입이면 무전송).
        if not _ws_authorized(ws):
            await ws.close(code=1008)
            return
        await ws.accept()
        last_seq = None
        try:
            while True:
                rec = cctv.latest("topview") if cctv is not None else None
                if rec is not None and rec[0] != last_seq:
                    await ws.send_bytes(rec[1])
                    last_seq = rec[0]
                await asyncio.sleep(0.066)
        except WebSocketDisconnect:
            return

    return app
