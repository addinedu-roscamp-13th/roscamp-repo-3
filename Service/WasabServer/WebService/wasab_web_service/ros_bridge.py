"""rclpy 브리지 — 콘솔 도메인에서 상태 구독(→FleetStore)·명령 발행. rclpy 지연 import.

원칙(콘솔 RosControl과 동일): publisher는 ROS spin 스레드에서만 publish. HTTP 스레드는
큐에 넣기만 하고 ROS 타이머가 drain. heartbeat 구독 QoS는 콘솔/agent와 동일한
BEST_EFFORT+VOLATILE(기본 RELIABLE이면 BestEffort publisher와 비호환 → 수신 안 됨).
"""
import queue
import threading

HEARTBEAT_TOPIC = "/robots/heartbeat"
TAG_STATUS_TOPIC = "/robots/tag_status"
RELOCALIZE_EVENT_TOPIC = "/robots/relocalize_event"
DOCK_EVENT_TOPIC = "/robots/dock_event"
FIRE_EVENT_TOPIC = "/wasab/fire_event"      # 천장캠 화재 이벤트(콘솔 발행)
TRAFFIC_GRANT_TOPIC = "/traffic/grant"      # PC측 교통 중재(traffic_node) 판정 결과
_PUB_DRAIN_HZ = 20.0
_TRAFFIC_REASON_KO = {                      # traffic_node QUEUED reason → 운영자용 한글
    "moving": "다른 로봇 이동 중이라 대기",
    "static": "다른 로봇이 정지 점유 중이라 대기",
    "startup": "기동 순서상 대기",
}


class RosBridge:
    def __init__(self, store, console_domain=50):
        self._store = store
        self._console_domain = int(console_domain)
        self._pub_q = queue.Queue()        # (topic, payload) — HTTP 스레드 put, ROS 타이머 drain
        self._ctx = None
        self._node = None
        self._exec = None
        self._String = None
        self._pubs = {}
        self._thread = None
        self._traffic_wait = {}    # rid -> (seq, 대기시작 monotonic 시각) — QUEUED~GRANTED 추적
        self._traffic_last = {}    # rid -> 마지막 처리 (seq,status) — 동일 메시지 재수신 중복기록 방지

    def start(self):
        if self._thread is not None:       # 중복 start 가드
            return
        import rclpy
        from rclpy.node import Node
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
        from std_msgs.msg import String
        self._String = String
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx, domain_id=self._console_domain)
        self._node = Node("wasab_webservice", context=self._ctx)
        hb_qos = QoSProfile(depth=10)                       # 콘솔/agent와 동일
        hb_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        hb_qos.durability = QoSDurabilityPolicy.VOLATILE
        self._node.create_subscription(
            String, HEARTBEAT_TOPIC,
            lambda m: self._store.ingest_heartbeat(m.data), hb_qos)
        event_qos = QoSProfile(depth=10)                    # 원샷 이벤트 → agent와 동일 RELIABLE
        event_qos.reliability = QoSReliabilityPolicy.RELIABLE
        event_qos.durability = QoSDurabilityPolicy.VOLATILE
        self._node.create_subscription(                     # tag_status는 주기적 → heartbeat와 동일 QoS
            String, TAG_STATUS_TOPIC,
            lambda m: self._store.ingest_tag_status(m.data), hb_qos)
        self._node.create_subscription(
            String, RELOCALIZE_EVENT_TOPIC,
            lambda m: self._store.ingest_relocalize_event(m.data), event_qos)
        self._node.create_subscription(
            String, DOCK_EVENT_TOPIC,
            lambda m: self._store.ingest_dock_event(m.data), event_qos)
        self._node.create_subscription(
            String, FIRE_EVENT_TOPIC,
            lambda m: self._ingest_fire(m.data), event_qos)
        self._node.create_subscription(
            String, TRAFFIC_GRANT_TOPIC,
            lambda m: self._ingest_traffic(m.data), event_qos)
        self._node.create_timer(1.0 / _PUB_DRAIN_HZ, self._drain_publish)
        self._exec = SingleThreadedExecutor(context=self._ctx)
        self._exec.add_node(self._node)
        self._thread = threading.Thread(target=self._exec.spin, daemon=True)
        self._thread.start()

    def _ingest_fire(self, data):
        import json
        from . import event_store
        try:
            d = json.loads(data)
        except Exception:
            return
        zone = d.get("zone") or ""
        event_store.record({"t": d.get("t", ""), "robot": "천장CCTV", "ip": "",
                            "mode": "", "type": "fire", "result": zone,
                            "status": "미처리", "pos": zone})

    def _robot_name(self, rid):
        """rid → robots.yaml name(self._store.fleet()에 이미 있는 구성 조회 재사용).

        미등록 로봇·store 미구성 등으로 못 찾으면 rid 문자열로 대체(예외로 콜백을 죽이지 않음).
        """
        try:
            for r in self._store.fleet():
                if r.get("id") == rid:
                    return r.get("name") or str(rid)
        except Exception:
            pass
        return str(rid)

    def _ingest_traffic(self, data):
        """교통 중재(/traffic/grant) 수신 — 실제로 대기가 발생한 경우만 기록.

        QUEUED는 "대기 시작"을 남기고 (rid,seq) 수신시각을 기억, 그 seq의 GRANTED가
        오면 "대기 해제"+대기시간을 기록한다. 대기 없이 바로 GRANTED되거나 REJECTED면
        기록하지 않는다(무대기 통과 폭주 방지). 동일 (seq,status) 재수신은 무시.
        """
        import json
        import time
        from datetime import datetime
        from . import event_store
        try:
            d = json.loads(data)
            rid = int(d["rid"])
            seq = d["seq"]
            status = d["status"]
        except (ValueError, TypeError, KeyError):
            return
        key = (seq, status)
        if self._traffic_last.get(rid) == key:
            return
        self._traffic_last[rid] = key
        robot = self._robot_name(rid)
        now_iso = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")
        blocker = d.get("blocker")
        pos = self._robot_name(int(blocker)) if blocker is not None else "-"
        if status == "QUEUED":
            self._traffic_wait[rid] = (seq, time.monotonic())
            reason = _TRAFFIC_REASON_KO.get(d.get("reason"), d.get("reason") or "")
            event_store.record({"t": now_iso, "robot": robot, "ip": "", "mode": "",
                                "type": "트래픽제어 · 대기 시작", "result": reason,
                                "status": "대기 중", "pos": pos})
        elif status == "GRANTED":
            pending = self._traffic_wait.pop(rid, None)
            if pending is None or pending[0] != seq:
                return                      # 대기 없이 바로 통과 — 기록 대상 아님
            waited_s = time.monotonic() - pending[1]
            event_store.record({"t": now_iso, "robot": robot, "ip": "", "mode": "",
                                "type": "트래픽제어 · 대기 해제", "result": "대기 %.1f초 후 통과" % waited_s,
                                "status": "통과", "pos": pos})
        # REJECTED 등 그 외 상태는 기록하지 않는다

    def _drain_publish(self):
        # ROS spin 스레드에서만 실행 — publisher 생성/발행을 여기서 처리
        while True:
            try:
                topic, payload = self._pub_q.get_nowait()
            except queue.Empty:
                return
            pub = self._pubs.get(topic)
            if pub is None:
                pub = self._node.create_publisher(self._String, topic, 10)
                self._pubs[topic] = pub
            msg = self._String()
            msg.data = payload
            pub.publish(msg)

    def publish(self, topic, payload):
        self._pub_q.put((topic, payload))    # HTTP 스레드: enqueue만

    def fleet(self):
        return self._store.fleet()

    def stop(self):
        import rclpy
        if self._exec is not None:
            self._exec.shutdown()            # spin() 종료 신호
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._ctx is not None:
            rclpy.shutdown(context=self._ctx)
        # 상태 초기화 — 재시작/테스트 반복 대비(start 가드도 _thread=None 기준)
        self._exec = self._thread = self._node = self._ctx = None
        self._pubs = {}
