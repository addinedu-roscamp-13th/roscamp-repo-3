from intruder_event_node import IntruderResponseFSM


def test_yes_enables_alarm_until_clear():
    fsm = IntruderResponseFSM(response_timeout_sec=15.0)

    assert fsm.step(True, None, False, now=0.0) == [
        {"stage": "prompt", "status": "퇴거를 시작할까요?"}
    ]
    assert fsm.state == "AWAITING"
    assert not fsm.alarm_enabled

    assert fsm.step(True, "yes", False, now=1.0) == [
        {"stage": "evicting", "status": "외부인 퇴거를 시작합니다"}
    ]
    assert fsm.alarm_enabled

    assert fsm.step(False, None, True, now=5.0) == [
        {"stage": "evicted", "status": "외부인이 퇴거했습니다"}
    ]
    assert fsm.state == "IDLE"
    assert not fsm.alarm_enabled


def test_no_never_enables_alarm_and_does_not_reprompt_same_person():
    fsm = IntruderResponseFSM()
    fsm.step(True, None, False, now=0.0)

    assert fsm.step(True, "no", False, now=1.0) == [
        {"stage": "declined", "status": "퇴거를 시작하지 않습니다"}
    ]
    assert fsm.state == "DECLINED"
    assert not fsm.alarm_enabled
    assert fsm.step(True, None, False, now=2.0) == []

    assert fsm.step(False, None, True, now=5.0) == []
    assert fsm.state == "IDLE"


def test_timeout_is_declined():
    fsm = IntruderResponseFSM(response_timeout_sec=3.0)
    fsm.step(True, None, False, now=10.0)

    assert fsm.step(True, None, False, now=13.0) == [
        {
            "stage": "declined",
            "status": "응답 시간이 초과되어 퇴거를 시작하지 않습니다",
        }
    ]
    assert not fsm.alarm_enabled


def test_clear_before_response_cancels_prompt():
    fsm = IntruderResponseFSM()
    fsm.step(True, None, False, now=0.0)

    assert fsm.step(False, None, True, now=4.0) == [
        {
            "stage": "cancelled",
            "status": "외부인이 감지 범위에서 사라졌습니다",
        }
    ]
    assert fsm.state == "IDLE"


def test_shutdown_cancels_active_prompt():
    fsm = IntruderResponseFSM()
    fsm.step(True, None, False, now=0.0)

    assert fsm.shutdown() == [
        {
            "stage": "cancelled",
            "status": "외부인 감지 프로세스가 종료되어 알림을 해제합니다",
        }
    ]
    assert fsm.state == "IDLE"
    assert not fsm.alarm_enabled


def test_shutdown_has_no_event_when_already_inactive():
    fsm = IntruderResponseFSM()

    assert fsm.shutdown() == []
    assert fsm.state == "IDLE"


def test_shutdown_cancels_evicting_and_disables_alarm():
    fsm = IntruderResponseFSM()
    fsm.step(True, None, False, now=0.0)
    fsm.step(True, "yes", False, now=1.0)
    assert fsm.alarm_enabled

    events = fsm.shutdown()

    assert events[0]["stage"] == "cancelled"
    assert fsm.state == "IDLE"
    assert not fsm.alarm_enabled
