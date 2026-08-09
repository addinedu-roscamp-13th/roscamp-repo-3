"""보안관 순찰 dual-context 노드 (rclpy 지연 import).

로봇 ctx: 로컬 Nav2 NavigateToPose + 자기 /amcl_pose + collision_monitor용 yield PointCloud 발행.
콘솔 ctx(50): /robots/heartbeat 구독 → 타 로봇 pose(BEST_EFFORT+VOLATILE) + rx시각(TTL).
5Hz 타이머: stale 필터 → PatrolPlanner.step → PATROL(goto, 목표변화 시 send_goal) /
YIELD(하드정지 PointCloud + goal cancel). Nav2 goal 상태추적(reject/abort→retry backoff),
YIELD가 N초 초과하면 pause+운영자 알림(fail-safe). /tf 구독 금지(CPU).
wasab_gui 미의존 — heartbeat 파서 내장(로봇 런타임에 GUI 미설치 대비).
"""
import json
import math
import os
import threading
import time

import yaml

from wasab_patrol.patrol_planner import PatrolPlanner

HEARTBEAT_TTL_S = 1.5        # 이보다 오래된 타 로봇 pose 제외(꺼진 로봇 잔상→영구 YIELD 방지)
YIELD_TIMEOUT_S = 20.0       # YIELD 이 이상 지속 시 pause+알림(SC4 fail-safe)
GOAL_RETRY_BACKOFF_S = 2.0   # goal reject/abort 후 재시도 간격(SC5)


def parse_heartbeat_xy(data, self_id):
    """heartbeat JSON → (id, x, y). 자기/무pose/파싱실패는 None. (wasab_gui 미의존, 순수·테스트 가능)"""
    try:
        d = json.loads(data)
    except Exception:
        return None
    try:
        rid = int(d.get("id"))              # 문자열 id("87")도 정규화(자기 오인 방지)
    except (TypeError, ValueError):
        return None
    if rid == int(self_id):
        return None
    if "x" not in d or "y" not in d:
        return None
    try:
        return (rid, float(d["x"]), float(d["y"]))
    except (TypeError, ValueError):
        return None


class PatrolNode:
    def __init__(self, robot_id, config_path, console_domain=50, clock=time.monotonic):
        self._robot_id = int(robot_id)
        if self._robot_id <= 0:               # PATROL_ROBOT_ID 누락 시 자기 heartbeat를 타 로봇으로 오인→영구 YIELD 방지
            raise ValueError("robot_id must be > 0 (set PATROL_ROBOT_ID); got %r" % (robot_id,))
        self._console_domain = int(console_domain)
        self._clock = clock                 # 주입 가능(테스트). rx/TTL/타임아웃 공통 시계
        cfg = yaml.safe_load(open(config_path))
        self._planner = PatrolPlanner(
            [tuple(w) for w in cfg["waypoints"]],
            yield_radius=cfg.get("yield_radius", 0.6),
            clear_radius=cfg.get("clear_radius", 0.9),
            reach_tol=cfg.get("reach_tol", 0.15),
        )
        self._own_xy = None
        self._others = {}          # id -> (x, y, rx)  (콘솔 스레드 갱신; rx=수신 monotonic)
        self._lock = threading.Lock()
        self._cur_goal = None
        self._goal_handle = None
        self._goal_pending = False # send_goal_async 진행 중(중복 발행 방지)
        self._retry_after = 0.0    # 이 시각 이후에만 재시도(backoff)
        self._yield_since = None   # YIELD 진입 시각(타임아웃)
        self._paused = False       # SC4 fail-safe

    def _fresh_others(self, now):     # stale(>TTL) 제외 (순수·테스트 가능)
        return [(x, y) for (x, y, rx) in self._others.values() if now - rx <= HEARTBEAT_TTL_S]

    # ---- 로봇 도메인 ----
    def start_robot_ctx(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.action import ActionClient
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        from nav2_msgs.action import NavigateToPose
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import String
        self._NavigateToPose = NavigateToPose
        self._rctx = rclpy.Context(); rclpy.init(context=self._rctx)
        self._rnode = Node("wasab_patrol_robot", context=self._rctx)
        # amcl은 /amcl_pose를 RELIABLE+TRANSIENT_LOCAL(latched)로 발행하고 정지 중엔 재발행하지 않음.
        # 구독 durability를 맞춰야 재측위 시 latch된 pose를 받아 _own_xy가 채워짐(안 맞으면 goal 미발행).
        amcl_qos = QoSProfile(depth=10)
        amcl_qos.reliability = QoSReliabilityPolicy.RELIABLE
        amcl_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self._rnode.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl_pose, amcl_qos)
        self._nav = ActionClient(self._rnode, NavigateToPose, "navigate_to_pose")
        self._yield_pub = self._rnode.create_publisher(PointCloud2, "/patrol/yield_obstacle", 10)
        self._notify_pub = self._rnode.create_publisher(String, "/patrol/notify", 10)
        self._rnode.create_timer(0.2, self._tick)     # 5Hz
        ex = rclpy.executors.SingleThreadedExecutor(context=self._rctx)
        ex.add_node(self._rnode)
        threading.Thread(target=ex.spin, daemon=True).start()

    def _on_amcl_pose(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self._own_xy = (p.x, p.y)

    def _at_goal(self):
        if self._cur_goal is None or self._own_xy is None:
            return False
        return math.hypot(self._own_xy[0] - self._cur_goal[0],
                          self._own_xy[1] - self._cur_goal[1]) <= self._planner._reach_tol

    def _tick(self):
        now = self._clock()
        with self._lock:
            own = self._own_xy
            others = self._fresh_others(now)     # TTL 필터
        if own is None:
            self._publish_yield(False)       # amcl_pose 대기 중에도 빈 cloud 발행(source timeout 방지)
            return
        r = self._planner.step(own, others, at_goal=self._at_goal())
        if r["state"] == "YIELD":
            self._publish_yield(True)            # collision_monitor 하드정지
            self._cancel()                       # 경로 시도 중단(보조)
            if self._yield_since is None:
                self._yield_since = now
            elif now - self._yield_since > YIELD_TIMEOUT_S and not self._paused:
                self._paused = True              # SC4
                self._notify("yield_timeout: patrol paused (blocked >%.0fs)" % YIELD_TIMEOUT_S)
            return
        # PATROL
        self._publish_yield(False)
        self._yield_since = None
        if self._paused:
            self._paused = False
            self._notify("patrol resumed")
        if self._goal_pending or now < self._retry_after:   # backoff/진행중 대기
            return
        if r["waypoint"] != self._cur_goal or self._goal_handle is None:
            self._send_goal(r["waypoint"])

    def _send_goal(self, wp):
        from geometry_msgs.msg import PoseStamped
        goal = self._NavigateToPose.Goal()
        ps = PoseStamped(); ps.header.frame_id = "map"
        ps.pose.position.x, ps.pose.position.y = float(wp[0]), float(wp[1])
        if len(wp) >= 3:                       # [x, y, yaw] → z축 회전 쿼터니언(2요소 항목은 identity 유지)
            yaw = float(wp[2])
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            ps.pose.orientation.w = 1.0
        goal.pose = ps
        if not self._nav.wait_for_server(timeout_sec=1.0):
            self._retry_after = self._clock() + GOAL_RETRY_BACKOFF_S
            return
        self._goal_pending = True
        self._cur_goal = wp
        self._nav.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):            # accepted/rejected
        self._goal_pending = False
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._goal_handle = None; self._cur_goal = None
            self._retry_after = self._clock() + GOAL_RETRY_BACKOFF_S
            return
        self._goal_handle = gh
        gh.get_result_async().add_done_callback(lambda fut, h=gh: self._on_goal_result(fut, h))

    def _on_goal_result(self, fut, gh):          # SUCCEEDED(4)/CANCELED(5)/ABORTED(6)
        if gh is not self._goal_handle:          # 취소된 옛 goal 결과는 현재(새) 핸들을 건드리지 않음
            return
        status = getattr(fut.result(), "status", 0)
        self._goal_handle = None
        if status == 6:                          # ABORTED → 같은 웨이포인트 재시도
            self._retry_after = self._clock() + GOAL_RETRY_BACKOFF_S
        # SUCCEEDED면 _at_goal→다음 웨이포인트로 진행. cur_goal 유지(도달 판정)

    def _cancel(self):
        if self._goal_handle is not None:
            try:
                self._goal_handle.cancel_goal_async()
            except Exception:
                pass
            self._goal_handle = None
        self._cur_goal = None

    def _publish_yield(self, on):
        # YIELD 시 정지존 안 점 클러스터, 아니면 빈 클라우드. base_footprint frame.
        from sensor_msgs.msg import PointCloud2, PointField
        import struct
        pts = [(0.22, 0.0), (0.22, 0.06), (0.22, -0.06), (0.18, 0.0)] if on else []
        msg = PointCloud2()
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = self._rnode.get_clock().now().to_msg()
        msg.height, msg.width = 1, len(pts)
        msg.fields = [PointField(name=n, offset=o, datatype=PointField.FLOAT32, count=1)
                      for n, o in (("x", 0), ("y", 4), ("z", 8))]
        msg.is_bigendian = False; msg.point_step = 12
        msg.row_step = 12 * len(pts); msg.is_dense = True
        msg.data = b"".join(struct.pack("<fff", x, y, 0.0) for (x, y) in pts)
        self._yield_pub.publish(msg)

    def _notify(self, text):
        from std_msgs.msg import String
        m = String(); m.data = text
        self._notify_pub.publish(m)

    # ---- 콘솔 도메인 ----
    def start_console_ctx(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        from std_msgs.msg import String
        self._cctx = rclpy.Context(); rclpy.init(context=self._cctx, domain_id=self._console_domain)
        self._cnode = Node("wasab_patrol_console", context=self._cctx)
        q = QoSProfile(depth=10)
        q.reliability = QoSReliabilityPolicy.BEST_EFFORT
        q.durability = QoSDurabilityPolicy.VOLATILE
        self._cnode.create_subscription(String, "/robots/heartbeat", self._on_hb, q)
        ex = rclpy.executors.SingleThreadedExecutor(context=self._cctx)
        ex.add_node(self._cnode)
        threading.Thread(target=ex.spin, daemon=True).start()

    def _on_hb(self, msg):
        r = parse_heartbeat_xy(msg.data, self._robot_id)   # 내장 파서(wasab_gui 무의존)
        if r is None:
            return
        rid, x, y = r
        with self._lock:
            self._others[rid] = (x, y, self._clock())       # rx시각 저장


def main():
    import rclpy
    from rclpy.node import Node
    from ament_index_python.packages import get_package_share_directory
    rclpy.init()
    boot = Node("wasab_patrol_boot")
    robot_id = boot.declare_parameter("robot_id", 0).value
    console_domain = boot.declare_parameter("console_domain", 50).value
    default_cfg = os.path.join(get_package_share_directory("wasab_patrol"), "config", "waypoints.yaml")
    config = boot.declare_parameter("config", default_cfg).value
    boot.destroy_node(); rclpy.shutdown()
    node = PatrolNode(robot_id, config, console_domain)
    node.start_console_ctx()
    node.start_robot_ctx()
    while True:
        time.sleep(1.0)
