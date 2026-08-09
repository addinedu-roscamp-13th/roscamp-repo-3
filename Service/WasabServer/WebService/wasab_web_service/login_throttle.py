"""로그인 실패 누적 지연(per key=ip:account). TTL·항목수·지연 상한. clock/sleeper 주입."""
import threading
import time


class LoginThrottle:
    def __init__(self, base_delay=0.5, max_delay=8.0, ttl=900, max_entries=4096,
                 clock=time.time, sleeper=time.sleep):
        self._base = base_delay
        self._max = max_delay
        self._ttl = ttl
        self._max_entries = max_entries
        self._clock = clock
        self._sleep = sleeper
        self._lock = threading.Lock()
        self._fails = {}            # key -> {count, last}

    def _cleanup(self, now):
        for k in [k for k, v in self._fails.items() if now - v["last"] > self._ttl]:
            del self._fails[k]
        if len(self._fails) > self._max_entries:
            ordered = sorted(self._fails, key=lambda k: self._fails[k]["last"])
            for k in ordered[: len(self._fails) - self._max_entries]:
                del self._fails[k]

    def before_attempt(self, key):
        now = self._clock()
        with self._lock:
            self._cleanup(now)
            e = self._fails.get(key)
            n = e["count"] if e else 0
        if n:
            exp = min(n - 1, 16)                     # 지수 폭주 방지
            self._sleep(min(self._max, self._base * (2 ** exp)))

    def record_failure(self, key):
        now = self._clock()
        with self._lock:
            e = self._fails.setdefault(key, {"count": 0, "last": now})
            e["count"] += 1
            e["last"] = now
            self._cleanup(now)                       # 추가 후 즉시 상한 적용

    def record_success(self, key):
        with self._lock:
            self._fails.pop(key, None)
