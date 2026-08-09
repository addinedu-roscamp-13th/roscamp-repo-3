#!/usr/bin/env python3
"""
fire_detect_node.py — 순찰용 JetCobot 카메라(UDP)에서 화재 감지 → 선생님 UI 알림/응답 처리.

  /wasab/k3/fire        (Float64MultiArray [cx,cy,conf], 도메인69) : 불꽃 중심(정규화 0~1). 미검출 시 미발행.
                         search_node/fire_search_node 가 --face-topic 대신 구독하면 팔이 TRACK/SEARCH.
  /wasab/fire_event     (String JSON, 콘솔 도메인) : user_gui/backend/ros_bridge.py 가 구독 →
                         event_store 기록 → 프론트엔드 알림. 필드: t/robot/stage/status.
  /wasab/fire_response  (String "yes"/"no", 콘솔 도메인, 구독) : user_gui 의 Yes/No 버튼 응답 중계.
  /wasab/k3/fire_suppress (String "close"/"open", 도메인69) : fire_search_node.py 가 구독해 그리퍼 동작.

감지 알고리즘은 wasab_overhead/fire_detect.py(천장캠용, 2026-07-22 확정)와 같은 원리
(시간표준편차=움직임 ∩ 느슨한 불색 HSV ∩ 최소면적)이되, **아레나 마스크(homography)는 제외**했다.
이 카메라는 팔에 부착되어 로봇 이동+팔 서보로 계속 움직이므로 고정 pixel↔meter 대응이 없어
천장캠과 같은 아레나 폴리곤 마스크를 적용할 방법이 없다(2026-07-23 논의로 확정).

2026-07-23 오후 스코프 확정: 이 노드가 순찰 로봇(PinkyPro)을 직접 멈추는 기능은 데모에 불필요
(촬영 시 두 로봇이 물리적으로 분리되어 있어 영상 편집으로 이어붙이는 방식) — 제거함.
대신 "화재 감지 순간부터"가 실제 구현 범위: 감지 → UI 알림(+백그라운드 팔 정렬, 동시 진행) →
"진압을 시작할까요?"(Yes/No) → Yes 시 그리퍼 닫기 → 결과(성공/실패) 알림.

상태 전이 (State):
  IDLE            — 화재 미확정. 계속 감지만 함.
  PENDING         — 화재 확정, ALIGN_DELAY_SEC 동안 팔 정렬 대기 중(이 사이 UI엔 감지 알림+영상만).
  AWAITING        — "진압을 시작할까요?" 표시 중, 응답(Yes/No) 또는 RESPONSE_TIMEOUT_SEC 대기.
  SUPPRESSING     — Yes 수신, 그리퍼 닫음. 이후 SUPPRESS_SUCCESS_SEC 이상 연속으로 불이 안 보이면
                    성공, 그 전까지 계속 보이다가 SUPPRESS_TIMEOUT_SEC 지나면 실패로 판정.
"""
from __future__ import annotations

import argparse
import datetime
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String

from udp_stream import UDPFrameReceiver

# --- 감지 파라미터 (wasab_overhead/fire_detect.py 와 동일 값에서 시작 — 아레나 마스크만 제외) ---
# S_MIN: 2026-07-24 50→140로 상향(USB웹캠 기준). 피부색은 채도가 낮고(옅음) 불꽃은 채도가 매우 높아서(선명함),
# 채도 기준만 올려도 사람 얼굴/손 오탐이 크게 줄어든다(실기에서 사람 얼굴이 불로 잡히는 문제 확인 후 조정).
# 2026-07-27: JetCobot 카메라는 사람 채도가 USB웹캠보다 높게 나와 140에서도 사람이 잡혀서 160으로 재상향.
V_MIN, S_MIN = 90, 160
H_LO_MAX, H_HI_MIN = 25, 160   # 2026-07-28: 35→25. 노란 물체(플라스틱 통 등) 오탐 배제(H 0~35엔 노랑 포함됨)
WIN = 12              # 시간변동을 볼 최근 프레임 창
STD_MIN = 12.0         # 픽셀 시간 표준편차 임계. 정적=노이즈(~2~4)→배제, 반복 변화(불)=큼→통과
MIN_AREA = 300         # 최소 영역 면적(px). 천장캠은 1080p 기준값 — 팔 카메라 해상도가 다르면 재튜닝 필요

# --- 확정 신뢰도(순간 튐 억제) — wasab_gui/vision_worker.py 와 동일 아이디어 ---
# 2026-07-24: 8프레임(약 1초, 8Hz 기준)이 너무 짧아 카메라에 잡히자마자 바로 확정되는 문제로
# 24프레임(약 3초)로 상향. 즉시 반응보다 오탐 억제를 우선.
# FIRE_CONF_MIN도 14(58%)→20(약 83%, 거의 매 프레임)로 상향 — 노이즈는 "떴다 안 떴다" 듬성듬성
# 걸리는데, 진짜 불꽃은 화면에 있는 한 거의 연속으로 감지되어야 한다는 관찰 반영.
FIRE_VOTE_WIN = 24    # 8Hz * 3초
FIRE_CONF_MIN = 20

# --- 면적변화(신규, 2026-07-24) — 사람/LED 오탐 억제 ---
# 불꽃은 테두리가 일렁여 프레임마다 면적이 들쭉날쭉하지만, 사람 실루엣이나 LED는 크기가 거의 일정하다.
# 최근 감지된 면적들의 변동계수(표준편차/평균)가 이 값 이상이어야 "불"로 최종 확정.
AREA_CV_MIN = 0.15

# --- UI 상호작용 타이밍(2026-07-23 잠정값 — 실기/UI 구현 시 조정) ---
ALIGN_DELAY_SEC = 3.0          # 화재감지 → "진압할까요?" 프롬프트까지(팔 정렬 대기 시뮬레이션)
RESPONSE_TIMEOUT_SEC = 20.0    # 프롬프트 후 무응답 시 No와 동일 처리까지 대기(N값, 미정 — 임시)
SUPPRESS_SUCCESS_SEC = 3.0     # 그리퍼 닫은 후 이만큼 연속으로 불이 안 보이면 진압 성공
SUPPRESS_TIMEOUT_SEC = 25.0    # 이 시간 지나도 성공 조건(연속 미검출) 못 채우면 진압 실패

FIRE_EVENT_TOPIC = "/wasab/fire_event"
FIRE_RESPONSE_TOPIC = "/wasab/fire_response"
FIRE_SUPPRESS_TOPIC = "/wasab/k3/fire_suppress"


def fire_color_mask(hsv):
    """불꽃색 픽셀 마스크(주황~빨강, 느슨하게)."""
    lo1 = np.array([0, S_MIN, V_MIN], np.uint8)
    hi1 = np.array([H_LO_MAX, 255, 255], np.uint8)
    lo2 = np.array([H_HI_MIN, S_MIN, V_MIN], np.uint8)
    hi2 = np.array([180, 255, 255], np.uint8)
    return cv2.bitwise_or(cv2.inRange(hsv, lo1, hi1), cv2.inRange(hsv, lo2, hi2))


def detect_fire(frame, buf, still=True, debug=False):
    """프레임에서 화재 검출(아레나 마스크 없음). → (bbox|None, centroid|None, buf).

    buf = 최근 gray 프레임 리스트(최대 WIN). 창이 차면 픽셀별 시간 std로 움직이는 영역을 찾고,
    느슨한 불꽃색과 겹치는 가장 큰 덩어리를 불로 판정.

    still=False(로봇팔이 방금 움직여 화면이 통째로 바뀐 구간)면 버퍼를 비우고 판정을 보류한다.
    안 그러면 팔 이동 자체가 "전체 화면 변화"로 잡혀 오탐(예: 빨간색이면 다 불로 판정)이 난다.
    """
    if not still:
        return None, None, []
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    color = fire_color_mask(hsv)

    buf = list(buf) if buf else []
    if buf and buf[-1].shape != gray.shape:
        buf = []
    buf.append(gray)
    if len(buf) > WIN:
        buf = buf[-WIN:]
    if len(buf) < WIN:
        return None, None, buf                       # 창 채우는 중

    std = np.stack(buf, axis=0).astype(np.float32).std(axis=0)
    active = ((std >= STD_MIN) * 255).astype(np.uint8)       # 움직이는 영역
    # 2026-07-24: 팽창 커널 11x11→5x5, 15x15→7x7로 축소(천장캠 1080p 기준값이라 팔 카메라엔 너무 큼).
    # 작은 노이즈 씨앗이 실제보다 훨씬 큰 덩어리로 부풀려져 평균 색이 희석되던 문제 완화 목적.
    active = cv2.dilate(active, np.ones((5, 5), np.uint8))
    mask = cv2.bitwise_and(color, active)                    # 불색 ∩ 움직임
    mask = cv2.dilate(mask, np.ones((7, 7), np.uint8))        # 불꽃 조각 병합
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, buf
    big = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(big) < MIN_AREA:
        return None, None, buf
    x, y, w, h = cv2.boundingRect(big)

    # --- 디버그(2026-07-24, USB캠 튜닝용) — 후보 영역의 실제 평균 HSV 확인, --debug일 때만 출력 ---
    if debug:
        region_mask = np.zeros(gray.shape, np.uint8)
        cv2.drawContours(region_mask, [big], -1, 255, -1)
        mean_h, mean_s, mean_v = cv2.mean(hsv, mask=region_mask)[:3]
        print(f"[fire_detect][debug] 후보 HSV 평균: H={mean_h:.0f} S={mean_s:.0f} V={mean_v:.0f} "
              f"area={cv2.contourArea(big):.0f} (S_MIN={S_MIN} V_MIN={V_MIN})")

    return (x, y, w, h), (x + w / 2.0, y + h / 2.0), buf


def _area_varies(areas: list) -> bool:
    """최근 감지 면적들이 충분히 들쭉날쭉한지(변동계수 >= AREA_CV_MIN).

    불꽃은 테두리가 일렁여 면적이 계속 변하지만, 사람 실루엣이나 LED는 크기가 거의 일정하다.
    감지된(0 아닌) 면적이 FIRE_CONF_MIN개 미만이면 아직 판단할 근거가 부족하므로 False.
    """
    detected = [a for a in areas if a > 0]
    if len(detected) < FIRE_CONF_MIN:
        return False
    mean_a = sum(detected) / len(detected)
    if mean_a <= 0:
        return False
    var_a = sum((a - mean_a) ** 2 for a in detected) / len(detected)
    return (var_a ** 0.5) / mean_a >= AREA_CV_MIN


class FireResponseFSM:
    """UI 알림/응답 상태머신 (순수 로직, rclpy 무관 — 단위테스트 가능).

    step(confirmed, response, now) 을 매 tick 호출. 상태 변화 시 발행할 이벤트 리스트를 반환.
    events 의 각 항목: {"stage": ..., "status": ...} (fire_event 페이로드에 그대로 실림).
    suppress 필드가 있으면 그 값("close")을 fire_suppress 토픽으로 발행해야 함을 의미.
    """

    def __init__(self, clock=time.time):
        self._clock = clock
        self.state = "IDLE"
        self._t_pending = None
        self._t_awaiting = None
        self._t_suppress = None
        self._t_last_seen = None   # SUPPRESSING 중 raw 검출이 마지막으로 있었던 시각

    def step(self, confirmed, response, now=None, seen_now=None):
        # seen_now: SUPPRESSING 판정 전용 raw 신호("지금 이 프레임에 불이 보이는가", votes 윈도우 아님).
        # None이면 confirmed로 대체(하위호환) — SUPPRESSING이 아닌 상태에선 안 쓰임.
        now = self._clock() if now is None else now
        events = []
        suppress = None

        if self.state == "IDLE":
            if confirmed:
                self.state = "PENDING"
                self._t_pending = now
                events.append({"stage": "detected", "status": "화재가 감지되었습니다"})

        elif self.state == "PENDING":
            if now - self._t_pending >= ALIGN_DELAY_SEC:
                self.state = "AWAITING"
                self._t_awaiting = now
                events.append({"stage": "prompt", "status": "진압을 시작할까요?"})

        elif self.state == "AWAITING":
            timed_out = (now - self._t_awaiting) >= RESPONSE_TIMEOUT_SEC
            if response == "yes":
                self.state = "SUPPRESSING"
                self._t_suppress = now
                self._t_last_seen = now   # 진압 시작 시점엔 당연히 불이 있었다고 간주
                suppress = "close"
                events.append({"stage": "suppressing", "status": "화재를 진압합니다"})
            elif response == "no" or timed_out:
                # response=="yes"를 먼저 체크 — 응답이 타임아웃 경계와 겹쳐 도착해도 "예"가 이기게.
                # (2026-07-28: 예전엔 timed_out을 먼저 봐서 이 경우 declined로 새버렸음)
                self.state = "IDLE"
                events.append({"stage": "declined", "status": "순찰을 재개합니다"})

        elif self.state == "SUPPRESSING":
            seen = confirmed if seen_now is None else seen_now
            if seen:
                self._t_last_seen = now
            if now - self._t_last_seen >= SUPPRESS_SUCCESS_SEC:
                # 불이 SUPPRESS_SUCCESS_SEC 이상 연속으로 안 보임 → 진압 성공
                self.state = "IDLE"
                suppress = "open"   # 진압 끝났으니 그리퍼 다시 개방
                events.append({"stage": "success", "status": "진압 성공"})
            elif now - self._t_suppress >= SUPPRESS_TIMEOUT_SEC:
                # SUPPRESS_TIMEOUT_SEC 동안 성공 조건(연속 미검출)을 못 채움 → 진압 실패
                self.state = "IDLE"
                suppress = "open"   # 실패해도 그리퍼는 계속 닫아둘 이유 없음
                events.append({"stage": "fail", "status": "진압 실패"})

        return events, suppress


class FireDetectNode(Node):
    def __init__(self, fire_topic: str, console_domain: int, arm_still_topic: str,
                 robot_id: str = "WaSaBArm", zone: str = "D"):
        super().__init__("fire_detect")
        self._robot_id = robot_id
        self._zone = zone
        self._fire_pub = self.create_publisher(Float64MultiArray, fire_topic, 10)
        self._suppress_pub = self.create_publisher(String, FIRE_SUPPRESS_TOPIC, 10)
        # fire_search_node.py(JetCobot)가 발행 — 팔이 방금 움직여 화면이 통째로 바뀐 구간인지.
        # 기본 True(구독 전엔 판정 막지 않음 — search 노드 없이 단독 테스트할 때도 동작하도록).
        self._arm_still = True
        self.create_subscription(Bool, arm_still_topic, self._on_arm_still, 10)

        # 선생님 UI 브리지 — 이 프로세스는 도메인69(JetCobot 화재)로 떠있으므로
        # 콘솔 도메인에 별도 Context를 하나 더 열어야 user_gui(콘솔 도메인)에 닿는다.
        # perception_node.py의 PinkyPro 제스처 브리지(_pinky_ctx)와 같은 2-Context 패턴.
        self._console_ctx = rclpy.Context()
        rclpy.init(context=self._console_ctx, domain_id=console_domain)
        self._console_node = Node("k3_fire_ui_bridge", context=self._console_ctx)
        self._event_pub = self._console_node.create_publisher(String, FIRE_EVENT_TOPIC, 10)
        self._response = None
        self._console_node.create_subscription(
            String, FIRE_RESPONSE_TOPIC, self._on_response, 10)
        # rclpy.spin_once(node)의 전역 executor는 기본 context(도메인69)에 고정돼있어서
        # 별도 context(도메인50)인 _console_node를 넣으면 콜백이 절대 안 불림 — 전용 executor 필수
        # (ros_bridge.py의 SingleThreadedExecutor(context=...) 패턴과 동일).
        from rclpy.executors import SingleThreadedExecutor
        self._console_exec = SingleThreadedExecutor(context=self._console_ctx)
        self._console_exec.add_node(self._console_node)

        self.fsm = FireResponseFSM()

    def _on_response(self, msg):
        self._response = msg.data   # "yes" / "no" — 다음 tick 에서 소비 후 리셋

    def _on_arm_still(self, msg):
        self._arm_still = msg.data

    @property
    def arm_still(self) -> bool:
        return self._arm_still

    def take_response(self):
        r, self._response = self._response, None
        return r

    def spin_console(self) -> None:
        # /wasab/fire_response(예/아니오)는 self._console_node(콘솔 도메인, 별도 context)가 구독한다.
        # 메인 루프는 기본 context의 node만 spin_once 하므로, 여기를 안 부르면 콜백이 영원히 안 불려
        # 응답이 절대 반영되지 않고 AWAITING이 매번 RESPONSE_TIMEOUT_SEC으로만 끝난다.
        self._console_exec.spin_once(timeout_sec=0.0)

    def publish_fire(self, cx: float, cy: float, conf: float) -> None:
        msg = Float64MultiArray()
        msg.data = [cx, cy, conf]
        self._fire_pub.publish(msg)

    def publish_events(self, events: list) -> None:
        # 통신 규약(protocol-fire-event-to-comprehensive-events.md) 준수:
        # type/zone/t(ISO8601)/robot 필수 필드 + 기존 stage/status(로봇팔 소스 전용, 백엔드가 mode 컬럼에 실음).
        for ev in events:
            payload = json.dumps({
                "type": "fire",
                "zone": self._zone,
                "t": datetime.datetime.now().isoformat(timespec="seconds"),
                "robot": self._robot_id,
                "stage": ev["stage"], "status": ev["status"],
            })
            self._event_pub.publish(String(data=payload))
            self.get_logger().info(f"[fire_event] {ev['status']}")

    def publish_suppress(self, action: str) -> None:
        self._suppress_pub.publish(String(data=action))

    def destroy(self) -> None:
        self._console_node.destroy_node()
        rclpy.shutdown(context=self._console_ctx)


def process_frame(node: "FireDetectNode", frame, buf: list, votes: list, areas: list,
                   debug: bool = False, ignore_arm_still: bool = False):
    """한 프레임을 감지→투표/면적버퍼→FSM까지 처리하고 필요한 발행을 모두 수행한다.

    카메라 소스가 다른 두 실행 경로(PC가 UDP로 받는 fire_detect_node.main / JetCobot이
    로컬 카메라를 직접 읽는 fire_detect_arm.py)가 이 함수를 공유해서, 튜닝값/FSM 로직이
    두 곳에서 따로 관리되다 어긋나는 걸 막는다.

    반환: (buf, votes, areas, bbox|None) — 호출측은 buf/votes/areas를 다음 호출에 그대로 넘기면 됨.
    """
    still = True if ignore_arm_still else node.arm_still
    bbox, centroid, buf = detect_fire(frame, buf, still=still, debug=debug)

    votes.append(bbox is not None)
    areas.append((bbox[2] * bbox[3]) if bbox is not None else 0)
    if len(votes) > FIRE_VOTE_WIN:
        votes = votes[-FIRE_VOTE_WIN:]
        areas = areas[-FIRE_VOTE_WIN:]
    color_motion_ok = len(votes) >= FIRE_VOTE_WIN and sum(votes) >= FIRE_CONF_MIN
    confirmed = color_motion_ok and _area_varies(areas)

    if bbox is not None:
        fh, fw = frame.shape[:2]
        cx, cy = centroid[0] / fw, centroid[1] / fh
        # conf: 최근 5프레임 중 몇 프레임이나 후보가 잡혔는지 비율(0~1). 2026-07-27 참고.
        recent = votes[-5:]
        track_conf = sum(recent) / len(recent) if recent else 0.0
        node.publish_fire(cx, cy, track_conf)
    # 미검출 시 /wasab/k3/fire 미발행 — search_node의 face_timeout으로 간헐 미검출 흡수

    # SUPPRESSING 판정은 votes 윈도우 기반 confirmed(순간 면적변동 스냅샷) 대신,
    # 매 프레임 raw 검출 여부(bbox 유무)를 그대로 넘겨서 FSM이 "마지막으로 본 시각"을 추적한다.
    events, suppress = node.fsm.step(confirmed, node.take_response(), seen_now=(bbox is not None))
    if events:
        node.publish_events(events)
        # FSM이 IDLE로 돌아가는 시점(재개/성공/실패)마다 votes/areas를 비워서,
        # 오탐 원인이 화면에 계속 남아있어도 다음 알림까지 항상 3초를 처음부터
        # 다시 채우게 한다 — 안 그러면 리셋 직후 바로 재확정되어 즉시 재알림된다.
        if any(ev["stage"] in ("declined", "success", "fail") for ev in events):
            votes = []
            areas = []
    if suppress:
        node.publish_suppress(suppress)

    return buf, votes, areas, bbox


def main() -> None:
    global STD_MIN
    parser = argparse.ArgumentParser(description="순찰 로봇 화재(불꽃) 감지 — 시간표준편차+불색")
    parser.add_argument("--udp-port", type=int, default=8091,
                        help="cam_server.py 로부터 받을 UDP 포트")
    parser.add_argument("--fire-topic", default="/wasab/k3/fire")
    parser.add_argument("--arm-still-topic", default="/wasab/k3/arm_still",
                        help="fire_search_node.py(JetCobot)가 발행하는 '팔 정지' 신호")
    parser.add_argument("--console-domain", type=int, default=50,
                        help="user_gui 백엔드가 구독 중인 콘솔 도메인")
    parser.add_argument("--robot-id", default="WaSaBArm",
                        help="fire_event payload의 robot 식별자(통신 규약 필수 필드, "
                             "프론트 순찰 화재 카드 필터와 일치해야 함)")
    parser.add_argument("--zone", default="D",
                        help="fire_event payload의 zone A~E(통신 규약 필수 필드, 기본 D=정원 데모 구역)")
    parser.add_argument("--rate", type=float, default=8.0, help="처리 주기(Hz)")
    parser.add_argument("--std-min", type=float, default=STD_MIN,
                        help="픽셀 시간표준편차 임계(카메라·압축 잡음 수준에 따라 재튜닝 필요, "
                             "기본값은 코드 상수와 동일)")
    parser.add_argument("--show", action="store_true", help="오버레이 창")
    parser.add_argument("--debug", action="store_true",
                        help="후보 영역 평균 HSV/면적을 매 프레임 출력(튜닝용, 평소엔 끔)")
    parser.add_argument("--ignore-arm-still", action="store_true",
                        help="arm_still 게이팅 무시(항상 still=True). PC 웹캠으로 감지 테스트하면서 "
                             "JetCobot 팔(그리퍼)도 같이 켜둘 때 씀 — 팔 움직임이 PC 카메라와 무관한데도 "
                             "감지 버퍼를 계속 초기화하던 문제 회피용. 평소(팔 장착 카메라)엔 쓰지 말 것.")
    args = parser.parse_args()
    STD_MIN = args.std_min

    latest = UDPFrameReceiver(args.udp_port)
    print(f"[fire_detect] UDP :{args.udp_port} 수신 대기 → {args.fire_topic} "
          f"(WIN={WIN}, STD_MIN={STD_MIN}, console_domain={args.console_domain})")

    rclpy.init()
    node = FireDetectNode(args.fire_topic, args.console_domain, args.arm_still_topic,
                          robot_id=args.robot_id, zone=args.zone)

    buf = []
    votes = []   # 최근 FIRE_VOTE_WIN 프레임의 감지여부(bool) — 순간 튐 억제
    areas = []   # votes와 같은 길이로 병행 — 그 프레임의 bbox 면적(미검출 시 0)
    period = 1.0 / args.rate
    try:
        while rclpy.ok():
            t0 = time.time()
            ok, frame = latest.read()
            if ok:
                buf, votes, areas, bbox = process_frame(node, frame, buf, votes, areas,
                                                          debug=args.debug,
                                                          ignore_arm_still=args.ignore_arm_still)

                if args.show:
                    disp = frame.copy()
                    if bbox is not None:
                        x, y, w, h = bbox
                        cv2.rectangle(disp, (x, y), (x + w, y + h), (0, 80, 220), 2)
                    cv2.putText(disp, f"state={node.fsm.state}", (10, 28),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 80, 220), 2)
                    cv2.imshow("FireDetect", disp)
                    cv2.waitKey(1)

            rclpy.spin_once(node, timeout_sec=0.0)
            node.spin_console()
            dt = time.time() - t0
            if dt < period:
                time.sleep(period - dt)
    except KeyboardInterrupt:
        pass
    finally:
        latest.release()
        node.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
