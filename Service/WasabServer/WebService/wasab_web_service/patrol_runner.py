"""웹앱 순찰 버튼 → run_patrol.sh subprocess start/stop (robot_id별).

콘솔/agent 계약과 무관 — PC 로컬 런처(Service/WasabServer/scripts/run_patrol.sh)를 띄운다.
domain·peer(ip)는 robots.yaml 구성에서 조회.

정지 주의(실기+통합 실증으로 확인): run_patrol.sh 는 `exec ros2 run` 이라 자식 patrol_node 가
별도 PID 이고, patrol_node 는 SIGINT/SIGTERM 을 흘려버린다. 게다가 리더(ros2 run)가 SIGINT 로
죽어도 자식은 살아남을 수 있다. 따라서
  1) 런처를 새 세션(프로세스그룹 리더=이 pid)으로 띄우고,
  2) 정지는 그룹 전체에 SIGINT → 잠깐 대기 → **그룹이 아직 살아있으면** SIGKILL → 해제한다.
     (SIGKILL 여부는 리더 종료가 아니라 '그룹 생존'으로 판단해야 자식이 새지 않는다.)
spawn·kill·alive 주입 가능(테스트: 리더/그룹 분리 모델링).
"""
import os
import signal
import subprocess

STOP_GRACE_SEC = 5.0             # SIGINT 후 정상 종료 대기(초). 이후 그룹 살아있으면 SIGKILL.


def _default_spawn(argv):
    # start_new_session=True → 자식(ros2 run/patrol_node)이 한 프로세스그룹(리더=이 pid).
    return subprocess.Popen(argv, start_new_session=True)


def _default_kill(proc, sig):
    # 그룹 전체에 신호(리더 pid == 그룹 pgid). 이미 종료됐으면 무시.
    try:
        os.killpg(proc.pid, sig)
    except (ProcessLookupError, PermissionError):
        pass


def _default_alive(proc):
    # 그룹에 아직 살아있는 프로세스가 있나(killpg 0-신호). 자식까지 포함.
    try:
        os.killpg(proc.pid, 0)
        return True
    except (ProcessLookupError, PermissionError):
        return False


class PatrolRunner:
    def __init__(self, robots, script_path,
                 spawn=_default_spawn, kill=_default_kill, alive=_default_alive):
        self._robots = robots            # {id: {"ip":..., "domain":...}}
        self._script = script_path
        self._spawn = spawn
        self._kill = kill
        self._alive = alive
        self._procs = {}                 # robot_id -> process(그룹 리더)

    def start(self, robot_id):
        rid = int(robot_id)
        if self.is_running(rid):
            return False                 # idempotent
        cfg = self._robots.get(rid)
        if not cfg or cfg.get("ip") is None or cfg.get("domain") is None:
            raise ValueError("unknown or unconfigured robot: %r" % (robot_id,))
        argv = [self._script, "--robot", str(rid),
                "--domain", str(cfg["domain"]), "--peer", str(cfg["ip"])]
        self._procs[rid] = self._spawn(argv)
        return True

    def stop(self, robot_id):
        rid = int(robot_id)
        proc = self._procs.get(rid)
        if proc is None:
            return False
        self._kill(proc, signal.SIGINT)          # 정상 종료 시도(그룹)
        self._grace(proc)
        if self._alive(proc):                    # 그룹이 아직 살아있으면(자식 잔존)
            self._kill(proc, signal.SIGKILL)     # 강제(그룹)
            self._grace(proc)
        self._procs.pop(rid, None)               # 신호 전달 후 해제
        return True

    def is_running(self, robot_id):
        proc = self._procs.get(int(robot_id))
        return proc is not None and self._alive(proc)

    def stop_all(self):
        for rid in list(self._procs):
            self.stop(rid)

    def _grace(self, proc):
        """리더 종료를 최대 STOP_GRACE_SEC 대기(좀비 reap 겸 타이머). 초과는 무시."""
        try:
            proc.wait(STOP_GRACE_SEC)
        except subprocess.TimeoutExpired:
            pass
