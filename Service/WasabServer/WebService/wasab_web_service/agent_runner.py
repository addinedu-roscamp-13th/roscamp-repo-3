"""웹앱 로봇 현황 → 각 로봇 agent SSH 기동/종료(토글).

로봇의 ~/wasab/Service/WasabServer/scripts/start_agent.sh / stop_agent.sh 를 SSH로 실행한다.
- 켜기: 로봇 정규 기동 env 를 **명시 세팅**해 실행(비대화형 SSH 라 .bashrc 안 물림).
  로봇 예시(사용자 확인): ROS_DOMAIN_ID=<로봇도메인> WASAB_ROBOT_ID=<id>
  WASAB_CONSOLE_DOMAIN=50 ROS_STATIC_PEERS=192.168.2.5(콘솔 IP).
  도메인·id 는 robots.yaml, console_domain·peer 는 공통.
- 끄기: stop_agent.sh 는 pkill 패턴이라 env 무관.
robots.yaml 구성에서 로봇 IP·domain 조회. (ROS/UDP 계약과 별개 — 프로세스 원격 제어)
"""
import os
import subprocess

SSH_USER = "pinky"
SSH_PW = "1"
CONNECT_TIMEOUT = 6
RUN_TIMEOUT = 30

# 로봇 정규 env + 워크스페이스 오버레이 source → start_agent.sh.
# ★install/setup.bash 를 명시 source 해야 agent_node 의 wasab_docking import 가 됨(SSH 비대화형에서
#   스크립트 내부 source 만으론 안 잡히는 것을 실측 확인 — ROS만/ROS+install 대조).
START_CMD = ("ROS_DOMAIN_ID={domain} WASAB_ROBOT_ID={rid} "
             "WASAB_CONSOLE_DOMAIN={console} ROS_STATIC_PEERS={peer} "
             "bash -c 'source /opt/ros/jazzy/setup.bash; source ~/wasab/install/setup.bash; "
             "~/wasab/Service/WasabServer/scripts/start_agent.sh'")
STOP_CMD = "~/wasab/Service/WasabServer/scripts/stop_agent.sh"


def _default_run(argv):
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=RUN_TIMEOUT)
        return {"ok": r.returncode == 0, "code": r.returncode,
                "out": (r.stdout or "")[-800:], "err": (r.stderr or "")[-800:]}
    except subprocess.TimeoutExpired:
        return {"ok": False, "code": -1, "out": "", "err": "ssh timeout(%ds)" % RUN_TIMEOUT}


class AgentRunner:
    """robot_id → SSH agent start/stop. run 주입 가능(테스트)."""

    def __init__(self, robots, console_domain=50, console_peer=None, run=None):
        self._robots = robots or {}
        self._console_domain = int(console_domain)
        # 로봇이 콘솔 도메인 발견에 쓰는 static peer(콘솔 PC IP). env 로 override 가능.
        self._console_peer = console_peer or os.environ.get("WASAB_CONSOLE_PEER", "192.168.2.5")
        self._run = run or _default_run

    def _cfg(self, robot_id):
        cfg = self._robots.get(int(robot_id)) or {}
        if not cfg.get("ip"):
            raise ValueError("robot %s: robots.yaml 에 ip 없음" % robot_id)
        return cfg

    def _ssh(self, ip, remote_cmd):
        argv = ["sshpass", "-p", SSH_PW, "ssh",
                "-o", "StrictHostKeyChecking=no",
                "-o", "ConnectTimeout=%d" % CONNECT_TIMEOUT,
                "%s@%s" % (SSH_USER, ip), remote_cmd]
        return self._run(argv)

    def start(self, robot_id):
        cfg = self._cfg(robot_id)
        if cfg.get("domain") is None:
            raise ValueError("robot %s: robots.yaml 에 domain 없음(agent 도메인)" % robot_id)
        cmd = START_CMD.format(domain=int(cfg["domain"]), rid=int(robot_id),
                               console=self._console_domain, peer=self._console_peer)
        return self._ssh(cfg["ip"], cmd)

    def stop(self, robot_id):
        return self._ssh(self._cfg(robot_id)["ip"], STOP_CMD)
