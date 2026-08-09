"""인메모리 서버 세션. 유휴/절대 만료 + 계정 단위 폐기. 단일 프로세스(workers=1) 전제."""
import secrets
import threading
import time


class SessionStore:
    def __init__(self, idle_ttl=1800, absolute_ttl=43200, clock=time.time):
        self._idle = idle_ttl
        self._abs = absolute_ttl
        self._clock = clock
        self._lock = threading.Lock()
        self._sessions = {}         # sid -> {account_id, role, created, last_seen}

    def create(self, account_id, role, face_verified=False, face_name=None):
        """face_verified: 얼굴로 로그인했으면 True(이미 신원 확인됨).
        계정(비밀번호) 로그인은 False 로 시작하고, 이후 얼굴 인증으로 승격한다."""
        now = self._clock()
        sid = secrets.token_urlsafe(32)
        with self._lock:
            self._sessions[sid] = {"account_id": account_id, "role": role,
                                   "created": now, "last_seen": now,
                                   "face_verified": bool(face_verified),
                                   "face_name": face_name}
        return sid

    def mark_face_verified(self, sid, face_name=None):
        """수업보조 사용 전 얼굴 인증 통과를 세션에 기록. 없는 세션이면 False."""
        with self._lock:
            s = self._sessions.get(sid)
            if s is None:
                return False
            s["face_verified"] = True
            s["face_name"] = face_name
            return True

    def validate(self, sid):
        now = self._clock()
        with self._lock:
            s = self._sessions.get(sid)
            if s is None:
                return None
            if now - s["created"] > self._abs or now - s["last_seen"] > self._idle:
                del self._sessions[sid]
                return None
            s["last_seen"] = now
            return {"account_id": s["account_id"], "role": s["role"],
                    "face_verified": s.get("face_verified", False),
                    "face_name": s.get("face_name")}

    def logout(self, sid):
        with self._lock:
            self._sessions.pop(sid, None)

    def revoke_account(self, account_id):
        with self._lock:
            for sid in [k for k, v in self._sessions.items() if v["account_id"] == account_id]:
                del self._sessions[sid]
