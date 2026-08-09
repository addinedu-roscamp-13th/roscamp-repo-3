# wasab_robot_agent/wasab_robot_agent/helpers.py
"""agent 순수 헬퍼 (ROS 무관, pytest)."""
import json
import os
import queue
import signal
import socket
import subprocess

SIGINT = signal.SIGINT
SIGKILL = signal.SIGKILL
GROUP_KILL_GRACE_S = 3.0


def classify_relocalize_cmd(data):
    """relocalize_cmd 문자열을 (action, payload)로 분류.
    'detector:start'/'start' → ('detector_start', None) : agent가 재측위 노드 기동.
    'detector:stop'/'stop'   → ('detector_stop', None)  : agent가 재측위 노드 종료.
    그 외(calibrate:N, rearm) → ('publish', data)        : /wasab/relocalize_cmd 재발행."""
    s = (data or "").strip()
    if s in ("detector:start", "start"):
        return ("detector_start", None)
    if s in ("detector:stop", "stop"):
        return ("detector_stop", None)
    return ("publish", data)


def drain_commands(get_nowait, sinks):
    """큐를 비우며 각 (kind, payload)를 sinks[kind](payload)로 디스패치한다.
    get_nowait: 큐의 get_nowait (소진 시 queue.Empty). sinks: {kind: callable(payload)}.
    cmd_vel가 하나라도 처리되면 True 반환(watchdog 판정용)."""
    drained_cmd_vel = False
    while True:
        try:
            kind, payload = get_nowait()
        except queue.Empty:
            break
        sink = sinks.get(kind)
        if sink is not None:
            sink(payload)
            if kind == "cmd_vel":
                drained_cmd_vel = True
    return drained_cmd_vel


def ip_octet(ip):
    return int(ip.split(".")[-1])


def own_ip():
    """라우팅 기준 자기 IP(외부 연결 없이 소켓 getsockname). 실패 시 127.0.0.1."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"
    finally:
        s.close()


def resolve_id(env, cfg, ip):
    """id 우선순위: env(WASAB_ROBOT_ID) → cfg → IP 끝 옥텟."""
    if env is not None and str(env).strip() != "":
        return int(env)
    if cfg is not None:
        return int(cfg)
    return ip_octet(ip)


def build_heartbeat(ip, rid, pose, battery_v, mode, stamp):
    """heartbeat JSON. pose=(x,y,yaw_deg)|None (None이면 x/y/yaw 생략)."""
    d = {"ip": ip, "id": int(rid), "battery_v": battery_v, "mode": mode, "stamp": stamp}
    if pose is not None:
        d["x"], d["y"], d["yaw"] = float(pose[0]), float(pose[1]), float(pose[2])
    return json.dumps(d)


def mode_for_self(payload, self_id):
    """공유 /wasab/cmd_mode JSON {robot_id,mode} → self_id 일치 시 mode, 아니면 None."""
    try:
        d = json.loads(payload)
    except (ValueError, TypeError):
        return None
    if int(d.get("robot_id", -1)) != int(self_id):
        return None
    return d.get("mode")


def copy_pose_with_cov(src):
    """console_ctx에서 받은 PoseWithCovarianceStamped를 robot_ctx 소유의 새 객체로 복사.
    내용은 동일(dumb pipe)하되 크로스-컨텍스트 안전을 위해 객체를 새로 만든다.
    stamp는 발행 측(콘솔)이 채운 값을 보존한다."""
    from geometry_msgs.msg import PoseWithCovarianceStamped
    dst = PoseWithCovarianceStamped()
    dst.header.frame_id = src.header.frame_id
    dst.header.stamp = src.header.stamp
    sp, dp = src.pose.pose, dst.pose.pose
    dp.position.x, dp.position.y, dp.position.z = sp.position.x, sp.position.y, sp.position.z
    dp.orientation.x = sp.orientation.x
    dp.orientation.y = sp.orientation.y
    dp.orientation.z = sp.orientation.z
    dp.orientation.w = sp.orientation.w
    dst.pose.covariance = list(src.pose.covariance)
    return dst


def tag_status_summary(obs_json, now_sec, robot_id):
    """/wasab/tag_observation JSON → 콘솔 인디케이터용 요약 JSON(최대면적 태그 + robot id)."""
    import math
    d = json.loads(obs_json)
    tags = d.get("tags", [])
    st = d.get("stamp", {"sec": 0, "nanosec": 0})
    stamp = float(st["sec"]) + float(st["nanosec"]) * 1e-9
    age_ms = int(max(0.0, (now_sec - stamp)) * 1000)
    if not tags:
        return json.dumps({"id": int(robot_id), "tag_id": None,
                           "dist_cm": None, "lateral_cm": None, "age_ms": age_ms})
    best = max(tags, key=lambda t: t["area"])
    dist_cm = int(round(math.hypot(best["x"], best["y"]) * 100))
    lateral_cm = int(round(best["y"] * 100))
    return json.dumps({"id": int(robot_id), "tag_id": int(best["tag_id"]), "dist_cm": dist_cm,
                       "lateral_cm": lateral_cm, "age_ms": age_ms})


def relocalize_event_summary(robot_id, event, tag_id):
    """재측위 이벤트 → 콘솔 이벤트 로그용 JSON. event ∈ {'success','timeout'}, tag_id는 int|None."""
    return json.dumps({"id": int(robot_id), "event": event,
                       "tag_id": None if tag_id is None else int(tag_id)})


def dock_event_summary(robot_id, event, tag_id, reason=None):
    """dock 이벤트 → 콘솔 이벤트 로그용 JSON. event ∈ {started,done,failed}; failed면 reason 포함."""
    d = {"id": int(robot_id), "event": event,
         "tag_id": None if tag_id is None else int(tag_id)}
    if reason is not None:
        d["reason"] = reason
    return json.dumps(d)


def kill_process_group(pgid, killpg=os.killpg, wait=None):
    """프로세스 그룹 전체를 SIGINT→(3s)→SIGKILL로 종료.

    그룹 리더(ros2 launch)가 먼저 죽어도 자식(precision_parking/detector)이 고아로
    남아 /cmd_vel을 계속 소유할 수 있다(실기 2026-07-09). 그래서 리더 생사와 무관하게
    pgid로 그룹을 거둔다. 이미 전부 죽었으면 조용히 무시.
    """
    if pgid is None:
        return
    try:
        killpg(pgid, SIGINT)
        try:
            if wait is not None:
                wait(timeout=GROUP_KILL_GRACE_S)
        except (subprocess.TimeoutExpired, TimeoutError):
            killpg(pgid, SIGKILL)
    except ProcessLookupError:
        pass


def reloc_detector_active(detector_proc):
    """재측위 detector가 살아있는지(카메라 점유 중). None/종료면 False."""
    return detector_proc is not None and detector_proc.poll() is None
