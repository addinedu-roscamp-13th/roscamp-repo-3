#!/usr/bin/env python3
"""Unknown 감지 승인/퇴거 FSM과 콘솔 이벤트 브리지.

로봇 도메인(프로세스 ROS_DOMAIN_ID):
  구독 /wasab/k3/unknown_alarm   String: IDLE/TRACKING/STOPPED
  구독 /wasab/k3/unknown_present Bool: 정상 처리 프레임의 Unknown 존재 여부

콘솔 도메인(--console-domain, 기본 50):
  발행 /wasab/intruder_event    String JSON
  구독 /wasab/intruder_response String yes/no

Pinky 도메인(--pinky-domain, 기본 51):
  발행 /wasab/k3/intruder_alarm_enable Bool: 승인 후 LED/Buzzer ON/OFF

카메라/인식 메시지가 끊긴 경우에는 퇴거로 판정하지 않는다. EVICTING 상태에서
정상 presence=False가 clear_confirm_sec 동안 연속될 때만 evicted를 발행한다.
"""
from __future__ import annotations

import argparse
import datetime
import json
import queue
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String


class IntruderResponseFSM:
    """외부인 확인 질문, 사용자 응답, 퇴거 완료를 관리하는 순수 상태머신."""

    def __init__(self, response_timeout_sec: float = 15.0, clock=time.monotonic):
        self._clock = clock
        self.response_timeout_sec = float(response_timeout_sec)
        self.state = "IDLE"
        self._awaiting_since = None

    @property
    def alarm_enabled(self) -> bool:
        return self.state == "EVICTING"

    def step(
        self,
        detected: bool,
        response: str | None,
        clear_confirmed: bool,
        now: float | None = None,
    ) -> list[dict[str, str]]:
        now = self._clock() if now is None else float(now)
        events: list[dict[str, str]] = []

        if self.state == "IDLE":
            if detected:
                self.state = "AWAITING"
                self._awaiting_since = now
                events.append({
                    "stage": "prompt",
                    "status": "퇴거를 시작할까요?",
                })

        elif self.state == "AWAITING":
            timed_out = (
                self._awaiting_since is not None
                and now - self._awaiting_since >= self.response_timeout_sec
            )
            if clear_confirmed:
                self.state = "IDLE"
                self._awaiting_since = None
                events.append({
                    "stage": "cancelled",
                    "status": "외부인이 감지 범위에서 사라졌습니다",
                })
            elif response == "yes":
                self.state = "EVICTING"
                events.append({
                    "stage": "evicting",
                    "status": "외부인 퇴거를 시작합니다",
                })
            elif response == "no":
                self.state = "DECLINED"
                events.append({
                    "stage": "declined",
                    "status": "퇴거를 시작하지 않습니다",
                })
            elif timed_out:
                self.state = "DECLINED"
                events.append({
                    "stage": "declined",
                    "status": "응답 시간이 초과되어 퇴거를 시작하지 않습니다",
                })

        elif self.state == "EVICTING":
            if clear_confirmed:
                self.state = "IDLE"
                events.append({
                    "stage": "evicted",
                    "status": "외부인이 퇴거했습니다",
                })

        elif self.state == "DECLINED":
            # 같은 외부인이 계속 보이는 동안 재질문하지 않는다.
            if clear_confirmed:
                self.state = "IDLE"

        return events

    def shutdown(self) -> list[dict[str, str]]:
        """프로세스 종료 시 활성 경보를 닫고 콘솔에 종료 상태를 남긴다."""
        was_active = self.state in ("AWAITING", "EVICTING")
        self.state = "IDLE"
        self._awaiting_since = None
        if not was_active:
            return []
        return [{
            "stage": "cancelled",
            "status": "외부인 감지 프로세스가 종료되어 알림을 해제합니다",
        }]


class ConsoleBridge:
    """콘솔 도메인의 publisher/subscriber를 전용 Executor에서 spin한다."""

    def __init__(
        self,
        console_domain: int,
        event_topic: str,
        response_topic: str,
    ) -> None:
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx, domain_id=int(console_domain))
        self._node = Node("intruder_console_bridge", context=self._ctx)
        self._event_pub = self._node.create_publisher(String, event_topic, 10)
        self._responses: queue.Queue[str] = queue.Queue()
        self._events: queue.Queue[str] = queue.Queue()
        self._node.create_subscription(
            String,
            response_topic,
            lambda msg: self._responses.put(msg.data),
            10,
        )
        self._node.create_timer(0.05, self._drain_events)
        self._executor = SingleThreadedExecutor(context=self._ctx)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(
            target=self._spin,
            name="intruder-console-domain",
            daemon=True,
        )
        self._thread.start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:
            # SIGINT 시 rclpy가 Context를 먼저 닫으면 executor가 RCLError를 낼 수 있다.
            if self._ctx.ok():
                raise

    def _drain_events(self) -> None:
        while True:
            try:
                payload = self._events.get_nowait()
            except queue.Empty:
                return
            self._event_pub.publish(String(data=payload))

    def publish_event(self, payload: str) -> None:
        self._events.put(payload)

    def take_response(self) -> str | None:
        response = None
        while True:
            try:
                response = self._responses.get_nowait()
            except queue.Empty:
                return response

    def flush(self, timeout_sec: float = 0.5) -> None:
        """종료 직전 큐의 마지막 이벤트가 ROS 스레드에서 발행될 시간을 준다."""
        deadline = time.monotonic() + float(timeout_sec)
        while not self._events.empty() and time.monotonic() < deadline:
            time.sleep(0.01)
        # Queue에서 꺼낸 직후 DDS publish가 끝나기 전일 수 있어 한 주기를 더 허용한다.
        time.sleep(0.05)

    def close(self) -> None:
        self.flush()
        try:
            self._executor.shutdown()
        except Exception:
            if self._ctx.ok():
                raise
        self._thread.join(timeout=2.0)
        try:
            self._node.destroy_node()
        except Exception:
            if self._ctx.ok():
                raise
        if self._ctx.ok():
            rclpy.shutdown(context=self._ctx)


class PinkyAlarmBridge:
    """외부인 LED·Buzzer 상태를 Pinky 전용 ROS domain으로 발행한다."""

    def __init__(self, pinky_domain: int, alarm_topic: str) -> None:
        self._ctx = rclpy.Context()
        rclpy.init(context=self._ctx, domain_id=int(pinky_domain))
        self._node = Node("intruder_pinky_bridge", context=self._ctx)
        self._publisher = self._node.create_publisher(
            Bool, alarm_topic, 10
        )

    def publish(self, enabled: bool) -> None:
        self._publisher.publish(Bool(data=bool(enabled)))

    def close(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:
            if self._ctx.ok():
                raise
        if self._ctx.ok():
            rclpy.shutdown(context=self._ctx)


class IntruderEventNode(Node):
    """Unknown 상태를 사용자 승인 기반 퇴거 이벤트와 경보 출력으로 변환한다."""

    def __init__(
        self,
        console_bridge: ConsoleBridge,
        pinky_alarm_bridge: PinkyAlarmBridge,
        unknown_state_topic: str,
        unknown_present_topic: str,
        robot_id: str,
        zone: str,
        response_timeout_sec: float,
        clear_confirm_sec: float,
        presence_timeout_sec: float,
    ) -> None:
        super().__init__("intruder_event")
        self._console_bridge = console_bridge
        self._pinky_alarm_bridge = pinky_alarm_bridge
        self._robot_id = robot_id
        self._zone = zone
        self._clear_confirm_sec = float(clear_confirm_sec)
        self._presence_timeout_sec = float(presence_timeout_sec)
        self._unknown_state = "IDLE"
        self._present: bool | None = None
        self._last_presence_at: float | None = None
        self._absent_since: float | None = None
        self._last_alarm_enabled: bool | None = None
        self.fsm = IntruderResponseFSM(response_timeout_sec)

        self.create_subscription(
            String, unknown_state_topic, self._on_unknown_state, 10
        )
        self.create_subscription(
            Bool, unknown_present_topic, self._on_unknown_present, 10
        )
        self.create_timer(0.1, self._tick)
        self.create_timer(1.0, self._publish_alarm_state)

    def _on_unknown_state(self, msg: String) -> None:
        if msg.data in ("IDLE", "TRACKING", "STOPPED"):
            self._unknown_state = msg.data

    def _on_unknown_present(self, msg: Bool) -> None:
        now = time.monotonic()
        self._present = bool(msg.data)
        self._last_presence_at = now
        if self._present:
            self._absent_since = None
        elif self._absent_since is None:
            self._absent_since = now

    def _presence_is_fresh(self, now: float) -> bool:
        return (
            self._last_presence_at is not None
            and now - self._last_presence_at <= self._presence_timeout_sec
        )

    def _clear_confirmed(self, now: float) -> bool:
        return (
            self._presence_is_fresh(now)
            and self._present is False
            and self._absent_since is not None
            and now - self._absent_since >= self._clear_confirm_sec
        )

    def _detection_confirmed(self, now: float) -> bool:
        return (
            self._unknown_state == "TRACKING"
            and self._presence_is_fresh(now)
            and self._present is True
        )

    def _publish_event(self, event: dict[str, str]) -> None:
        payload = json.dumps({
            "type": "intruder",
            "zone": self._zone,
            "t": datetime.datetime.now().isoformat(timespec="seconds"),
            "robot": self._robot_id,
            "stage": event["stage"],
            "status": event["status"],
        })
        self._console_bridge.publish_event(payload)
        self.get_logger().info(
            f"[intruder_event] {event['stage']}: {event['status']}"
        )

    def _publish_alarm_state(self) -> None:
        enabled = self.fsm.alarm_enabled
        self._pinky_alarm_bridge.publish(enabled)
        if enabled != self._last_alarm_enabled:
            self.get_logger().info(
                f"Intruder LED/Buzzer: {'ON' if enabled else 'OFF'}"
            )
            self._last_alarm_enabled = enabled

    def _tick(self) -> None:
        now = time.monotonic()
        response = self._console_bridge.take_response()
        events = self.fsm.step(
            detected=self._detection_confirmed(now),
            response=response,
            clear_confirmed=self._clear_confirmed(now),
            now=now,
        )
        for event in events:
            self._publish_event(event)
        if events:
            self._publish_alarm_state()


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Unknown 외부인 승인/퇴거 이벤트 및 LED·부저 제어"
    )
    parser.add_argument(
        "--unknown-state-topic", default="/wasab/k3/unknown_alarm"
    )
    parser.add_argument(
        "--unknown-present-topic", default="/wasab/k3/unknown_present"
    )
    parser.add_argument(
        "--alarm-enable-topic",
        default="/wasab/k3/intruder_alarm_enable",
    )
    parser.add_argument("--event-topic", default="/wasab/intruder_event")
    parser.add_argument(
        "--response-topic", default="/wasab/intruder_response"
    )
    parser.add_argument("--console-domain", type=int, default=50)
    parser.add_argument("--pinky-domain", type=int, default=51)
    parser.add_argument("--robot-id", default="WaSaBArm")
    parser.add_argument("--zone", default="C")
    parser.add_argument("--response-timeout", type=float, default=15.0)
    parser.add_argument("--clear-confirm", type=float, default=3.0)
    parser.add_argument("--presence-timeout", type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    bridge = ConsoleBridge(
        args.console_domain,
        args.event_topic,
        args.response_topic,
    )
    pinky_bridge = PinkyAlarmBridge(
        args.pinky_domain,
        args.alarm_enable_topic,
    )
    node = IntruderEventNode(
        console_bridge=bridge,
        pinky_alarm_bridge=pinky_bridge,
        unknown_state_topic=args.unknown_state_topic,
        unknown_present_topic=args.unknown_present_topic,
        robot_id=args.robot_id,
        zone=args.zone,
        response_timeout_sec=args.response_timeout,
        clear_confirm_sec=args.clear_confirm,
        presence_timeout_sec=args.presence_timeout,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            for event in node.fsm.shutdown():
                node._publish_event(event)
            node._publish_alarm_state()
        node.destroy_node()
        bridge.close()
        pinky_bridge.close()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
