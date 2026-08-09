"""main 조립용 순수 함수 — environ 주입이라 import 부작용/monkeypatch 타이밍 문제 없음."""

_LOOPBACK = ("http://localhost", "http://127.0.0.1", "http://[::1]")


def bootstrap_admin(auth, environ):
    aid = environ.get("WASAB_WEBAPP_ADMIN")
    apw = environ.get("WASAB_WEBAPP_ADMIN_PW")
    if bool(aid) != bool(apw):
        raise SystemExit("WASAB_WEBAPP_ADMIN 과 WASAB_WEBAPP_ADMIN_PW 는 둘 다 설정해야 합니다")
    if auth.count_admins() > 0:
        return                          # 이미 관리자 존재 → env 무시(같은 env로 재기동 가능)
    if not aid or not apw:
        raise SystemExit("관리자 계정이 없습니다. WASAB_WEBAPP_ADMIN/_PW 로 부트스트랩하세요")
    if auth.exists(aid):
        raise SystemExit(f"부트스트랩 ID가 기존 일반 계정과 충돌합니다: {aid}")
    auth.add(aid, apw, "admin")


def allowed_origins(environ):
    raw = environ.get("WASAB_WEBAPP_ORIGIN", "")
    origins = tuple(o.strip() for o in raw.split(",") if o.strip())
    if not origins:
        raise SystemExit("WASAB_WEBAPP_ORIGIN 필수(예: https://192.168.2.5:8100). CSRF 보호 fail-closed")
    return origins


def check_cookie_security(secure, origins):
    """secure_cookies=False 는 loopback http origin 에서만 허용(런처 우회 시 서버측 방어)."""
    if secure:
        return
    for o in origins:
        if not any(o == p or o.startswith(p + ":") for p in _LOOPBACK):
            raise SystemExit(f"WASAB_WEBAPP_SECURE_COOKIES=0 은 loopback origin 에서만 허용: {o}")
