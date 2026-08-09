# wasab_robot_agent/wasab_robot_agent/agent_node.py
"""로봇 agent — 2-Context 도메인 브리지.
robot_ctx(로컬 ROS_DOMAIN_ID): map->base pose(/amcl_pose), /battery/voltage, 로컬 /cmd_vel·/wasab/cmd_mode 발행.
console_ctx(WASAB_CONSOLE_DOMAIN): /robots/heartbeat 발행, /robot_<id>/cmd_vel·/wasab/cmd_mode 구독.
"""
import json
import math
import os
import queue
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped

from wasab_docking import dock_approach, tag_map_poses
from wasab_robot_agent import helpers
from wasab_robot_agent.dock_session import DockSession, parse_dock_cmd

HEARTBEAT_TOPIC = "/robots/heartbeat"
CMD_MODE_TOPIC = "/wasab/cmd_mode"
FOLLOW_CMD_TOPIC = "/wasab/follow_cmd"   # 웹앱 "교실이동" → ai_node.py(pinky_yolo) 로컬 구독
ASSIST_STOP_TOPIC = "/wasab/assist_stop_cmd"   # 웹앱 "일시정지"/"수업 동행 정지" → ai_node.py E-STOP
HEARTBEAT_HZ = 2.0
CMD_VEL_TIMEOUT_S = 0.3


def _hb_qos():
    q = QoSProfile(depth=10)
    q.reliability = QoSReliabilityPolicy.BEST_EFFORT
    q.durability = QoSDurabilityPolicy.VOLATILE
    return q


def _reloc_event_qos():
    # 원샷 이벤트 → RELIABLE(드롭 방지). 발행·구독 양쪽 동일해야 신뢰성 성립.
    q = QoSProfile(depth=10)
    q.reliability = QoSReliabilityPolicy.RELIABLE
    q.durability = QoSDurabilityPolicy.VOLATILE
    return q


def _yaw_deg(q):
    return math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                   1.0 - 2.0 * (q.y * q.y + q.z * q.z)))


ROBOT_DRAIN_HZ = 20.0    # robot_ctx 타이머: 큐 drain + watchdog (명령 응답성)
POSE_STALE_S = 5.0       # 이보다 오래된 _pose는 dock nav 판정에서 신뢰 안 함(amcl_pose 저빈도 대비)
DETECTOR_TIMEOUT_S = 30.0  # 재측위 detector 자동종료 백스톱(성공 시 즉시 종료, 태그 미감지 시 이 시간 후 종료)
DETECTOR_STOP_GRACE_S = 1.5  # 재측위 성공 후 relocalizer kill 유예 → /initialpose가 AMCL에 flush될 시간 확보
RELOCALIZE_LAUNCH_CMD = (
    "source /opt/ros/jazzy/setup.bash; source $HOME/wasab/install/setup.bash; "
    "exec ros2 launch wasab_docking relocalization.launch.py")
DOCK_STANDOFF_M = 0.5      # 태그 정면 접근 거리(FOV 바닥 고려, worklog §2.4)
DOCK_TIMEOUT_S = 90.0      # dock 전체 백스톱(Nav2 접근+서보 여유)
DOCK_LAUNCH_CMD = (
    "source /opt/ros/jazzy/setup.bash; source $HOME/wasab/install/setup.bash; "
    "exec ros2 launch wasab_docking dock.launch.py")
TAG_MAP_POSES_PATH = os.path.expanduser("~/.wasab/tag_map_poses.yaml")


class RobotAgent:
    """크로스-컨텍스트 안전 규칙: console_ctx 콜백은 큐에 넣기만 하고,
    robot_ctx 타이머가 큐를 drain해 robot 도메인 publisher로 발행한다.
    각 publisher는 자기 Context executor 스레드에서만 publish된다."""
    def __init__(self):
        self.console_domain = int(os.environ.get("WASAB_CONSOLE_DOMAIN", "100"))
        self.ip = helpers.own_ip()
        self.id = helpers.resolve_id(os.environ.get("WASAB_ROBOT_ID"), None, self.ip)
        self._lock = threading.Lock()
        self._battery_v = None
        self._mode = None
        self._pose = None                 # (x,y,yaw_deg)|None — robot_ctx가 갱신
        self._pose_stamp = 0.0            # _pose 갱신 시각(monotonic) — dock nav 판정 신선도용
        self._last_cmd_t = 0.0
        self._to_robot = queue.Queue()    # console_ctx → robot_ctx 명령: ("cmd_vel",Twist)/("mode",str)
        self._last_tag_obs = None      # robot_ctx가 스냅샷, console_ctx가 발행(heartbeat 패턴)
        self._detector_proc = None     # 재측위 detector 서브프로세스(콘솔 "재측위 시작"으로 기동)
        self._detector_deadline = None # 자동종료 백스톱 시각(monotonic)
        self._detector_stop_at = None  # 재측위 성공 후 유예 종료 예약 시각(monotonic)
        self._reloc_events = queue.Queue()  # robot_ctx put → console_ctx _tick_heartbeat drain 발행(크로스-컨텍스트)
        self._dock_proc = None
        self._dock_pgid = None          # launch 프로세스그룹(리더 사망 후에도 자식 회수용)
        self._dock_deadline = None      # monotonic 백스톱
        self._dock = DockSession()
        self._dock_events = queue.Queue()   # robot_ctx put → console_ctx가 /robots/dock_event 발행

    def start(self):
        # robot_ctx = 기본 도메인(로봇 ROS_DOMAIN_ID 환경변수 그대로)
        self._robot_ctx = rclpy.Context()
        rclpy.init(context=self._robot_ctx)
        self._console_ctx = rclpy.Context()
        rclpy.init(context=self._console_ctx, domain_id=self.console_domain)

        # 노드 이름에 로봇 id를 붙여 유일화 — 다중로봇 시 도메인 내 동일 노드명 충돌 방지
        # (특히 console_ctx: 여러 로봇 agent가 공유 콘솔 도메인에 등록하므로 필수).
        rn = Node(f"wasab_agent_robot_{self.id}", context=self._robot_ctx)
        cn = Node(f"wasab_agent_console_{self.id}", context=self._console_ctx)
        self._rn, self._cn = rn, cn

        # robot_ctx: pose(/amcl_pose + /initialpose 반영), battery, 로컬 재발행 publisher, drain 타이머
        rn.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, 10)
        self._local_cmd_vel = rn.create_publisher(Twist, "/cmd_vel", 10)
        self._local_mode = rn.create_publisher(String, CMD_MODE_TOPIC, 10)
        self._local_initialpose = rn.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self._local_relocalize_cmd = rn.create_publisher(String, "/wasab/relocalize_cmd", 10)
        self._local_follow = rn.create_publisher(String, FOLLOW_CMD_TOPIC, 10)
        self._local_assist_stop = rn.create_publisher(String, ASSIST_STOP_TOPIC, 10)
        rn.create_subscription(String, "/wasab/tag_observation", self._on_tag_obs, 10)
        rn.create_subscription(Float32, "/battery/voltage", self._on_battery, 10)
        # 재측위 성공(/initialpose 발행) 감지 → detector 즉시 종료(CPU). 세션 중일 때만 반응.
        rn.create_subscription(PoseWithCovarianceStamped, "/initialpose", self._on_initialpose_seen, 10)
        rn.create_subscription(String, "/wasab/docking_state", self._on_docking_state, 10)
        rn.create_timer(1.0 / ROBOT_DRAIN_HZ, self._tick_drain)     # 20Hz: 큐 drain + watchdog

        # console_ctx: heartbeat pub(스냅샷 읽기만), cmd_vel/cmd_mode sub(enqueue만)
        self._hb_pub = cn.create_publisher(String, HEARTBEAT_TOPIC, _hb_qos())
        self._tag_status_pub = cn.create_publisher(String, "/robots/tag_status", _hb_qos())
        self._reloc_event_pub = cn.create_publisher(String, "/robots/relocalize_event", _reloc_event_qos())
        self._dock_event_pub = cn.create_publisher(String, "/robots/dock_event", _reloc_event_qos())
        cn.create_subscription(String, f"/robot_{self.id}/relocalize_cmd", self._on_relocalize_cmd, 10)
        cn.create_subscription(String, f"/robot_{self.id}/dock_cmd", self._on_dock_cmd, 10)
        cn.create_subscription(String, f"/robot_{self.id}/follow_cmd", self._on_follow_cmd, 10)
        cn.create_subscription(String, f"/robot_{self.id}/assist_stop_cmd", self._on_assist_stop_cmd, 10)
        cn.create_subscription(Twist, f"/robot_{self.id}/cmd_vel", self._on_cmd_vel, 10)
        cn.create_subscription(String, CMD_MODE_TOPIC, self._on_cmd_mode, 10)
        cn.create_subscription(PoseWithCovarianceStamped,
                               f"/robot_{self.id}/set_initialpose",
                               self._on_set_initialpose, 10)
        cn.create_timer(1.0 / HEARTBEAT_HZ, self._tick_heartbeat)

        self._stop = threading.Event()
        self._threads = [
            threading.Thread(target=self._spin, args=(self._robot_ctx, rn), daemon=True),
            threading.Thread(target=self._spin, args=(self._console_ctx, cn), daemon=True),
        ]
        for t in self._threads:
            t.start()

    def _spin(self, ctx, node):
        ex = SingleThreadedExecutor(context=ctx)
        ex.add_node(node)
        try:
            while not self._stop.is_set() and ctx.ok():
                ex.spin_once(timeout_sec=0.1)
        finally:
            ex.shutdown()

    # ---- robot_ctx 콜백/타이머 (robot 도메인 publish는 여기서만) ----
    def _on_battery(self, msg):
        with self._lock:
            self._battery_v = float(msg.data)

    def _set_pose_from(self, p):
        # geometry_msgs Pose → (x,y,yaw_deg) 스냅샷 + 타임스탬프(dock nav 판정 신선도용)
        with self._lock:
            self._pose = (p.position.x, p.position.y, _yaw_deg(p.orientation))
            self._pose_stamp = time.monotonic()

    def _on_amcl_pose(self, msg):
        # amcl가 map 프레임에서 발행하는 로봇 pose. TF 5Hz 폴링 대체(35Hz /tf 소비 제거).
        self._set_pose_from(msg.pose.pose)

    def _on_tag_obs(self, msg):
        with self._lock:                       # 스냅샷만(발행 안 함) — heartbeat pose와 동일
            self._last_tag_obs = msg.data

    def _tick_drain(self):
        # ROBOT_DRAIN_HZ(20Hz): console→robot 큐 drain + watchdog.
        def _sink_cmd_vel(payload):
            # 큐로 넘어온 Twist를 robot 도메인 소유의 새 객체로 복사해 발행(크로스-컨텍스트 명시적 안전)
            tw = Twist()
            tw.linear.x, tw.linear.y, tw.linear.z = payload.linear.x, payload.linear.y, payload.linear.z
            tw.angular.x, tw.angular.y, tw.angular.z = payload.angular.x, payload.angular.y, payload.angular.z
            self._local_cmd_vel.publish(tw)
            self._last_cmd_t = time.monotonic()

        sinks = {
            "cmd_vel": _sink_cmd_vel,
            "mode": lambda p: self._local_mode.publish(String(data=p)),
            "initialpose": lambda p: self._local_initialpose.publish(helpers.copy_pose_with_cov(p)),
            "relocalize_cmd": self._sink_relocalize,
            "dock": self._handle_dock,
            "follow": lambda p: self._local_follow.publish(String(data=p)),
            "assist_stop": lambda p: self._local_assist_stop.publish(String(data=p)),
        }
        drained_cmd_vel = helpers.drain_commands(self._to_robot.get_nowait, sinks)
        # watchdog: 마지막 cmd 후 timeout이면 로컬 zero
        if (not drained_cmd_vel and self._last_cmd_t
                and time.monotonic() - self._last_cmd_t > CMD_VEL_TIMEOUT_S):
            self._local_cmd_vel.publish(Twist())
            self._last_cmd_t = 0.0
        # detector 자동종료 백스톱: 성공(/initialpose) 없이 deadline 지나면 종료(CPU 안전)
        if (self._detector_deadline is not None
                and time.monotonic() > self._detector_deadline):
            self._rn.get_logger().warning("재측위 detector 타임아웃 → 자동 종료")
            self._reloc_events.put(("timeout", None))
            self._stop_detector()
        # 재측위 성공 유예: /initialpose가 AMCL에 전달될 시간 준 뒤 relocalizer 종료
        if (self._detector_stop_at is not None
                and time.monotonic() > self._detector_stop_at):
            self._rn.get_logger().info("재측위 성공 유예 경과 → detector 종료")
            self._stop_detector()
        # dock: 자식 조기종료 감지(Popen 성공 ≠ launch 성공) → 즉시 종결
        # process_exited는 진짜 조기죽음에만 발생: precision_parking/detector는 DONE/FAILED에서
        # self-exit 안 하고 SIGINT까지 spin하므로, 프로세스 종료 시 terminal docking_state가
        # in-flight일 수 없다. (이 계약이 깨지면=launch가 DONE 후 self-exit하면, in-flight DONE
        # 선점 방지를 위해 종료감지에 grace 필요.)
        if self._dock.active and self._dock_proc is not None and self._dock_proc.poll() is not None:
            ev = self._dock.on_process_exited()
            # launch만 죽고 자식(precision_parking/detector)이 살아남는다 → 그룹째 회수
            helpers.kill_process_group(self._dock_pgid)
            self._dock_proc = None
            self._dock_pgid = None
            self._dock_deadline = None
            if ev:
                self._emit_dock_event(ev["event"], ev["tag_id"], ev["reason"])
        # dock: 전체 timeout 백스톱
        if self._dock.active and self._dock_deadline is not None and time.monotonic() > self._dock_deadline:
            self._stop_dock()
            ev = self._dock.on_timeout()
            if ev:
                self._emit_dock_event(ev["event"], ev["tag_id"], ev["reason"])

    # ---- 재측위 detector on-demand (콘솔 "재측위 시작/종료") ----
    def _sink_relocalize(self, payload):
        action, data = helpers.classify_relocalize_cmd(payload)
        if action == "detector_start":
            self._start_detector()
        elif action == "detector_stop":
            self._stop_detector()
        else:  # calibrate:N / rearm → relocalizer로 재발행
            self._local_relocalize_cmd.publish(String(data=data))

    def _start_detector(self):
        if self._detector_proc is not None and self._detector_proc.poll() is None:
            return  # 이미 실행 중(중복 무시)
        try:
            self._detector_proc = subprocess.Popen(
                ["bash", "-c", RELOCALIZE_LAUNCH_CMD],
                preexec_fn=os.setsid,   # 프로세스그룹 → 전체 kill 가능
                stdout=open(os.path.expanduser("~/agent_detector.log"), "wb"),
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
            self._detector_deadline = time.monotonic() + DETECTOR_TIMEOUT_S
            self._detector_stop_at = None
            self._rn.get_logger().info(f"재측위 detector 기동(pid {self._detector_proc.pid})")
        except Exception as e:   # noqa: BLE001
            self._rn.get_logger().error(f"재측위 detector 기동 실패: {e}")
            self._detector_proc = None
            self._detector_deadline = None

    def _stop_detector(self):
        self._detector_deadline = None
        self._detector_stop_at = None
        proc = self._detector_proc
        self._detector_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            self._rn.get_logger().info("재측위 detector 종료(CPU 회수)")
        except Exception as e:   # noqa: BLE001
            self._rn.get_logger().warning(f"detector 종료 중 예외: {e}")

    def _on_initialpose_seen(self, msg):
        # 재측위/수동 initialpose를 pose로 즉시 반영 — 재측위 직후 dock의 nav 판정이 신선한 pose를 쓰게.
        self._set_pose_from(msg.pose.pose)
        # 재측위 성공 = relocalizer가 /initialpose 발행. detector는 유예 후 종료 —
        # 즉시 kill 시 relocalizer context가 파괴돼 /initialpose가 AMCL에 flush되기 전 유실됨
        # (→ yaw 미반영 버그, 2026-07-08). 성공 이벤트는 즉시 발행하고 kill만 미룬다.
        if self._detector_proc is not None and self._detector_stop_at is None:
            self._rn.get_logger().info(
                f"재측위 성공(/initialpose) → detector {DETECTOR_STOP_GRACE_S}s 유예 후 종료")
            self._reloc_events.put(("success", self._current_tag_id()))
            self._detector_stop_at = time.monotonic() + DETECTOR_STOP_GRACE_S

    def _current_tag_id(self):
        """_last_tag_obs 스냅샷의 최대면적 태그 id(없으면 None). best-effort."""
        with self._lock:
            obs = self._last_tag_obs
        if not obs:
            return None
        try:
            tags = json.loads(obs).get("tags", [])
            return int(max(tags, key=lambda t: t["area"])["tag_id"]) if tags else None
        except (ValueError, KeyError, TypeError):
            return None

    # ---- dock on-demand (콘솔 "dock_cmd": start:<tag_id>/stop) ----
    def _emit_dock_event(self, event, tag_id, reason=None):
        self._dock_events.put((event, tag_id, reason))

    def _handle_dock(self, data):
        # robot_ctx에서만 호출(_tick_drain sink)
        parsed = parse_dock_cmd(data)
        if parsed is None:
            return
        kind, tag_id = parsed
        if kind == "stop":
            self._stop_dock()
            ev = self._dock.on_stop()
            if ev:
                self._emit_dock_event(ev["event"], ev["tag_id"], ev["reason"])
            return
        # start:N
        if self._dock.active or (self._dock_proc is not None and self._dock_proc.poll() is None):
            self._rn.get_logger().warn("dock 진행 중 → start 무시")
            return
        # 재측위 detector가 카메라 점유 중이면 dock 거부(늦은 camera_busy 실패 방지, spec §7)
        if helpers.reloc_detector_active(self._detector_proc):
            self._rn.get_logger().warn("재측위 detector 활성 → dock 거부")
            self._emit_dock_event("failed", tag_id, "reloc_detector_active")
            return
        try:
            cfg = tag_map_poses.load(TAG_MAP_POSES_PATH)
        except Exception:
            cfg = {}
        ap = dock_approach.approach_pose(cfg, tag_id, DOCK_STANDOFF_M)
        if ap is None:
            self._emit_dock_event("failed", tag_id, "unregistered_tag")
            return
        pose = self._pose
        fresh = pose is not None and (time.monotonic() - self._pose_stamp) < POSE_STALE_S
        robot_xy = (pose[0], pose[1]) if fresh else None   # stale/미상이면 None → needs_nav=True(안전기본값)
        use_nav = dock_approach.needs_nav(cfg, tag_id, robot_xy, DOCK_STANDOFF_M)
        try:
            self._dock_proc = subprocess.Popen(
                ["bash", "-lc", DOCK_LAUNCH_CMD
                 + f" tag_id:={tag_id} approach_pose_x:={ap[0]} approach_pose_y:={ap[1]}"
                   f" approach_pose_yaw:={ap[2]} nav_enabled:={str(use_nav).lower()}"
                   f" cmd_vel_enabled:=true"],
                preexec_fn=os.setsid,
                stdout=open(os.path.expanduser("~/agent_dock.log"), "wb"),
                stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL)
        except Exception as e:
            self._rn.get_logger().error(f"dock launch 실패: {e!r}")
            self._dock_proc = None
            self._emit_dock_event("failed", tag_id, "launch_failed")
            return
        # 리더(launch)가 먼저 죽어도 자식을 거둘 수 있도록 pgid 보관
        self._dock_pgid = os.getpgid(self._dock_proc.pid)
        self._dock_deadline = time.monotonic() + DOCK_TIMEOUT_S
        self._dock.start(tag_id)
        self._emit_dock_event("started", tag_id)
        self._rn.get_logger().info(
            f"dock 기동 tag={tag_id} nav={use_nav} approach={ap} pid={self._dock_proc.pid}")

    def _stop_dock(self):
        # 리더가 이미 죽었어도 그룹은 거둔다(고아 precision_parking이 /cmd_vel 소유).
        self._dock_deadline = None
        proc = self._dock_proc
        pgid = self._dock_pgid
        self._dock_proc = None
        self._dock_pgid = None
        if pgid is None:
            return
        try:
            wait = proc.wait if proc is not None and proc.poll() is None else None
            helpers.kill_process_group(pgid, wait=wait)
            self._rn.get_logger().info("dock 종료")
        except Exception as e:   # noqa: BLE001
            self._rn.get_logger().warning(f"dock 종료 중 예외: {e}")

    def _on_docking_state(self, msg):
        # robot_ctx 콜백 → 세션 종결 판정(active일 때만)
        try:
            state = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        ev = self._dock.on_docking_state(state)
        if ev:
            self._stop_dock()
            self._emit_dock_event(ev["event"], ev["tag_id"], ev["reason"])

    # ---- console_ctx 콜백/타이머 (enqueue·스냅샷 읽기만) ----
    def _on_cmd_mode(self, msg):
        mode = helpers.mode_for_self(msg.data, self.id)
        if mode is None:
            return
        with self._lock:
            self._mode = mode
        self._to_robot.put(("mode", msg.data))     # robot_ctx가 로컬 재발행

    def _on_cmd_vel(self, msg):
        self._to_robot.put(("cmd_vel", msg))       # robot_ctx가 로컬 relay + watchdog

    def _on_set_initialpose(self, msg):
        self._to_robot.put(("initialpose", msg))   # robot_ctx가 복사 후 로컬 재발행

    def _on_relocalize_cmd(self, msg):
        self._to_robot.put(("relocalize_cmd", msg.data))   # robot_ctx가 로컬 재발행

    def _on_dock_cmd(self, msg):
        self._to_robot.put(("dock", msg.data))   # robot_ctx가 세션 판정+launch 관리

    def _on_follow_cmd(self, msg):
        self._to_robot.put(("follow", msg.data))   # robot_ctx가 로컬 재발행(ai_node.py 구독)

    def _on_assist_stop_cmd(self, msg):
        self._to_robot.put(("assist_stop", msg.data))   # robot_ctx가 로컬 재발행(ai_node.py E-STOP)

    def _tick_heartbeat(self):
        with self._lock:
            bv, mode, pose = self._battery_v, self._mode, self._pose
        s = helpers.build_heartbeat(self.ip, self.id, pose, bv, mode, time.time())
        out = String(); out.data = s
        self._hb_pub.publish(out)
        with self._lock:
            obs = self._last_tag_obs
        if obs is not None:
            try:
                st = String(); st.data = helpers.tag_status_summary(obs, time.time(), self.id)
                self._tag_status_pub.publish(st)
            except Exception as e:                       # tag_status 실패가 heartbeat를 죽이면 안 됨
                self._cn.get_logger().warn(f"tag_status 발행 실패(무시): {e}")
        try:                                             # 재측위 이벤트 drain(robot_ctx가 put) → 발행
            while True:
                event, tag_id = self._reloc_events.get_nowait()
                ev = String(); ev.data = helpers.relocalize_event_summary(self.id, event, tag_id)
                self._reloc_event_pub.publish(ev)
        except queue.Empty:
            pass
        except Exception as e:
            self._cn.get_logger().warn(f"relocalize_event 발행 실패(무시): {e}")
        try:                                             # dock 이벤트 drain(robot_ctx가 put) → 발행
            while True:
                event, tag_id, reason = self._dock_events.get_nowait()
                ev = String(); ev.data = helpers.dock_event_summary(self.id, event, tag_id, reason)
                self._dock_event_pub.publish(ev)
        except queue.Empty:
            pass
        except Exception as e:
            self._cn.get_logger().warn(f"dock_event 발행 실패(무시): {e}")

    def stop(self):
        self._stop.set()
        self._stop_detector()          # agent 종료 시 재측위 detector도 정리(고아 방지)
        self._stop_dock()              # agent 종료 시 dock launch도 정리(고아 방지)
        for t in self._threads:
            t.join(timeout=1.0)
        self._rn.destroy_node(); self._cn.destroy_node()
        rclpy.shutdown(context=self._robot_ctx)
        rclpy.shutdown(context=self._console_ctx)


def main():
    agent = RobotAgent()
    agent.start()
    # SIGTERM(stop_agent.sh 의 kill -TERM)도 SIGINT 와 같이 정리 경로로 —
    # agent.stop() 이 dock/detector 자식 그룹까지 회수(고아 방지). 핸들러 없으면 SIGTERM 은
    # 즉시 종료라 진행 중 자식이 고아로 남았음.
    def _on_term(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_term)
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        agent.stop()


if __name__ == "__main__":
    main()
