from .authhelpers import authed_client


class FakeArm:
    def __init__(self):
        self.calls = []

    def status(self, arm_id):
        self.calls.append(("status", arm_id))
        return {"face-recognition": {"running": False}}

    def command(self, arm_id, command):
        self.calls.append(("command", arm_id, command))
        return {"status": "queued"}

    def toggle_feature(self, arm_id, feature):
        self.calls.append(("feature", arm_id, feature))
        return {"running": True}

    def fire_prompt(self, arm_id):
        self.calls.append(("fire-prompt", arm_id))
        return {"prompt": {"id": 1, "message": "진압을 시작할까요?"}}

    def fire_response(self, arm_id, response):
        self.calls.append(("fire-response", arm_id, response))
        return {"status": "accepted"}

    def face_prompt(self, arm_id):
        self.calls.append(("face-prompt", arm_id))
        return {"prompt": {"id": 2, "message": "미등록 사용자입니다."}}

    def acknowledge_face_prompt(self, arm_id):
        self.calls.append(("face-ack", arm_id))
        return {"status": "acknowledged"}

    def dual_command(self, command):
        self.calls.append(("dual", command))
        return {"status": "started"}

    def logs(self, after_id=0):
        self.calls.append(("logs", after_id))
        return {"logs": []}

    def camera(self, arm_id):
        self.calls.append(("camera", arm_id))
        return b"\xff\xd8jpeg", "image/jpeg"


def test_arm_status_and_feature_proxy():
    arm = FakeArm()
    client = authed_client(arm=arm)
    assert client.get("/api/arm/status?arm_id=left").status_code == 200
    response = client.post(
        "/api/arm/feature",
        json={"arm_id": "left", "feature": "face-recognition"},
    )
    assert response.status_code == 200
    assert arm.calls == [
        ("status", "left"),
        ("feature", "left", "face-recognition"),
    ]


def test_arm_single_and_dual_commands():
    arm = FakeArm()
    client = authed_client(arm=arm)
    assert client.post(
        "/api/arm/command", json={"arm_id": "left", "command": "recycle"}
    ).status_code == 200
    assert client.post(
        "/api/arm/command", json={"arm_id": "dual", "command": "gift-giving"}
    ).status_code == 200
    assert arm.calls == [
        ("command", "left", "recycle"),
        ("dual", "gift-giving"),
    ]


def test_arm_rejects_command_not_available_for_dual():
    arm = FakeArm()
    response = authed_client(arm=arm).post(
        "/api/arm/command", json={"arm_id": "dual", "command": "recycle"}
    )
    assert response.status_code == 400
    assert arm.calls == []


def test_arm_camera_proxy():
    arm = FakeArm()
    response = authed_client(arm=arm).get("/api/arm/camera?arm_id=right")
    assert response.status_code == 200
    assert response.headers["content-type"] == "image/jpeg"
    assert response.content == b"\xff\xd8jpeg"


def test_arm_fire_and_face_prompt_proxy():
    arm = FakeArm()
    client = authed_client(arm=arm)
    assert client.get("/api/arm/fire-prompt?arm_id=left").status_code == 200
    assert client.post(
        "/api/arm/fire-response?arm_id=left", json={"response": "yes"}
    ).status_code == 200
    assert client.get("/api/arm/face-prompt?arm_id=right").status_code == 200
    assert client.post("/api/arm/face-prompt/ack?arm_id=right").status_code == 200
    assert arm.calls == [
        ("fire-prompt", "left"),
        ("fire-response", "left", "yes"),
        ("face-prompt", "right"),
        ("face-ack", "right"),
    ]


def test_estop_also_stops_both_arms():
    arm = FakeArm()
    response = authed_client(arm=arm).post(
        "/api/cmd/estop", json={"target": "all", "active": True}
    )
    assert response.status_code == 200
    assert response.json()["arm_stop"] == {"left": "sent", "right": "sent"}
    assert arm.calls == [
        ("command", "left", "stop"),
        ("command", "right", "stop"),
    ]


def test_estop_release_does_not_move_arms():
    arm = FakeArm()
    response = authed_client(arm=arm).post(
        "/api/cmd/estop", json={"target": "all", "active": False}
    )
    assert response.status_code == 200
    assert arm.calls == []
