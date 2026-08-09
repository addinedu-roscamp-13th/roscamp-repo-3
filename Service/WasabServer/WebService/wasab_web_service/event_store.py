"""종합 이벤트 영속 저장 — 최소 SQLite 스토어.

프론트가 이미 잡은 이벤트를 **저장/조회만** 한다(파생·tick·dedup 없음).
연결은 호출마다 열고 닫아(이벤트 저빈도) 스레드 안전 걱정 없이 단순 유지.
DB 경로: env WASAB_WEBAPP_DB, 기본 ~/.wasab/webapp_events.db (소스 트리 밖, 누적 영구 보관).
"""
import os
import sqlite3

DB_PATH = os.environ.get("WASAB_WEBAPP_DB", os.path.expanduser("~/.wasab/webapp_events.db"))

_SCHEMA = """
CREATE TABLE IF NOT EXISTS event (
  id     INTEGER PRIMARY KEY,
  ts     TEXT, robot TEXT, ip TEXT, mode TEXT,
  type   TEXT, result TEXT, status TEXT, pos TEXT
)"""

_COLS = ["t", "robot", "ip", "mode", "type", "result", "status", "pos"]


def _conn():
    os.makedirs(os.path.dirname(os.path.abspath(DB_PATH)), exist_ok=True)
    c = sqlite3.connect(DB_PATH)
    c.execute(_SCHEMA)
    return c


def record(e):
    """이벤트 dict(키: t/robot/ip/mode/type/result/status/pos) 1건 append."""
    c = _conn()
    try:
        c.execute(
            "INSERT INTO event (ts,robot,ip,mode,type,result,status,pos)"
            " VALUES (?,?,?,?,?,?,?,?)",
            (e.get("t"), e.get("robot"), e.get("ip"), e.get("mode"),
             e.get("type"), e.get("result"), e.get("status"), e.get("pos")))
        c.commit()
    finally:
        c.close()


def recent(limit=5000):
    """최신순 이벤트 리스트(프론트 표시용 키). limit 상한 20000."""
    limit = max(1, min(int(limit), 20000))
    c = _conn()
    try:
        rows = c.execute(
            "SELECT ts,robot,ip,mode,type,result,status,pos FROM event"
            " ORDER BY ts DESC, id DESC LIMIT ?", (limit,)).fetchall()
    finally:
        c.close()
    return [dict(zip(_COLS, r)) for r in rows]
