"""교사/관리자 계정 저장소. teachers.json(PBKDF2 해시) 원자적 저장. stdlib만."""
import hashlib
import hmac
import json
import os
import re
import secrets
import tempfile
import threading

PBKDF2_ITERATIONS = 600_000
_ID_RE = re.compile(r"[A-Za-z0-9._-]{1,64}")
_ROLES = ("admin", "teacher")
_DUMMY_SALT = b"\x00" * 16


class AuthError(ValueError):
    pass


class AuthStore:
    def __init__(self, path, iterations=PBKDF2_ITERATIONS):
        self._path = path
        self._iter = iterations
        self._lock = threading.RLock()
        self._data = self._load()

    def _load(self):
        if not os.path.exists(self._path):
            return {}
        with open(self._path, "r", encoding="utf-8") as f:
            return json.load(f)     # 손상 시 JSONDecodeError → 기동 실패

    def _write(self, data):
        d = os.path.dirname(self._path) or "."
        os.makedirs(d, exist_ok=True)
        fd, tmp = tempfile.mkstemp(dir=d)
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(data, f)
            os.chmod(tmp, 0o600)
            os.replace(tmp, self._path)     # 원자적 교체
        except BaseException:
            if os.path.exists(tmp):
                os.unlink(tmp)
            raise

    def _hash(self, pw, salt, iters):
        return hashlib.pbkdf2_hmac("sha256", pw.encode("utf-8"), salt, iters)

    def add(self, account_id, password, role="teacher"):
        if not _ID_RE.fullmatch(account_id):
            raise AuthError("invalid id")
        if role not in _ROLES:
            raise AuthError("invalid role")
        if not (8 <= len(password) <= 128):
            raise AuthError("invalid password length")
        with self._lock:
            if account_id in self._data:
                raise AuthError("exists")
            salt = secrets.token_bytes(16)
            new = dict(self._data)
            new[account_id] = {
                "role": role, "algo": "pbkdf2_sha256", "iterations": self._iter,
                "salt": salt.hex(), "hash": self._hash(password, salt, self._iter).hex(),
            }
            self._write(new)            # 저장 성공 후에만 교체
            self._data = new

    def verify(self, account_id, password):
        with self._lock:
            rec = self._data.get(account_id)
        if rec is None:
            self._hash(password, _DUMMY_SALT, self._iter)   # 타이밍 평준화
            return None
        got = self._hash(password, bytes.fromhex(rec["salt"]), rec["iterations"])
        return rec["role"] if hmac.compare_digest(got, bytes.fromhex(rec["hash"])) else None

    def list(self):
        with self._lock:
            return [{"id": k, "role": v["role"]} for k, v in sorted(self._data.items())]

    def exists(self, account_id):
        with self._lock:
            return account_id in self._data

    def count_admins(self):
        with self._lock:
            return sum(1 for v in self._data.values() if v["role"] == "admin")

    def remove(self, account_id):
        with self._lock:
            if account_id not in self._data:
                raise AuthError("not found")
            if self._data[account_id]["role"] == "admin" and self.count_admins() == 1:
                raise AuthError("cannot remove last admin")
            new = dict(self._data)
            del new[account_id]
            self._write(new)
            self._data = new
