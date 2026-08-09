# WasabArm

`Device/WasabBot/WasabArm`은 라즈베리파이/Jetcobot 쪽에서 실행되는 Arm 클라이언트입니다.

역할:

- 카메라에서 640x480 프레임 캡처
- 최신 프레임을 노트북 서버 AdminGUI로 스트리밍
- 현재 MyCobot Flange pose 읽기
- 노트북 서버의 `/v1/grasp-plan`에 이미지와 pose 전송
- 서버가 반환한 `flange_command`를 로컬 안전 범위로 검증
- 안전한 경우 MyCobot과 그리퍼 제어

## 실행 준비

```bash
cd /home/ane/dev_ws/wasab-manipulation/Device/WasabBot/WasabArm
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## 네트워크 설정

설정 파일:

```text
config/client_config.ini
```

노트북 서버 IP를 설정합니다.

```ini
[network]
grasp_server_url = http://<노트북_LAN_IP>:8000/v1/grasp-plan
```

라즈베리파이에서 `127.0.0.1`은 노트북이 아니라 라즈베리파이 자신입니다. 반드시 노트북 LAN IP를 사용합니다.

연결 확인:

```bash
python3 check_server_connection.py
```

## 실행

```bash
python3 run_client.py
```

## 키/원격 명령

| 키/명령 | 기능 |
| --- | --- |
| `g` / `pick` | 프레임과 현재 Flange pose를 서버로 보내고 pick 수행 |
| `f` / `place` | home 이동 후 place pose로 이동하고 그리퍼 열기 |
| `p` / `pose` | 현재 Flange pose 출력 |
| `q` / `gripper` | 그리퍼 열기/닫기 토글 |
| `s` / `servo-release` | 모든 servo release |
| `k` / `servo-focus` | 모든 servo focus |
| `a` / `recycle` | 왼팔이 `trash`는 빨간 박스, `water`는 파란 박스로 분류 |
| `help` | 왼팔이 AprilTag ID 0 물체를 픽업한 뒤 기존 Place 동작 실행 |
| `w` / `home` | home 위치로 이동 |
| `space` / `stop` | 현재 동작 즉시 정지 |
| `x` / `exit` | 종료 |

`recycle.dynamic_color_target=true`이면 빨강/파랑 박스의 중심 픽셀을
Hand-Eye 보정으로 Base XY로 변환합니다. 테스트 후 고정 측정 자세로
돌아가려면 `config/client_config.left.ini`에서 이 값을 `false`로 바꿉니다.

## 카메라 스트림

Arm 클라이언트는 최신 프레임을 노트북 서버로 보냅니다.

```ini
[camera_stream]
enabled = true
fps = 4.0
jpeg_quality = 45

[udp_stream]
enabled = true
port = 8001
fallback_http = true
```

노트북 서버에서는 아래 화면에서 확인합니다.

```text
http://<노트북_LAN_IP>:8000/camera-view
```

## 안전 설정

처음 테스트할 때는 실제 모터 명령을 막습니다.

```ini
[safety]
dry_run = true
```

실제 동작 전 확인할 항목:

- `safe_x/y/z_*` 범위
- `home_flange_coords`
- `mycobot_port`
- `camera_id`
- 서버 캘리브레이션 해상도와 카메라 해상도 일치 여부

## 캘리브레이션

수동:

```bash
python3 marker.py
```

자동:

```bash
python3 auto_marker.py
```

주요 결과 파일:

- `camera_intrinsic_charuco.npz`
- `auto_camera_intrinsic_charuco_*.npz`
- `auto_handeye_result_*.json`
- `auto_handeye_result_*.npz`
- `auto_handeye_charuco_samples_*.npz`

노트북 서버가 사용할 파일은 `Service/WasabAIServer/ai_service/calibration/`에 맞게 복사하거나 서버 설정 경로를 수정합니다.
# Arm-specific configuration

Robot settings are maintained separately:

- `config/client_config.ini` — common settings plus `[right.*]` overrides
- `config/arm_identity` — device-local identity containing only `left` or `right`
- `config/client_config.ini` — active/backward-compatible local configuration

Deploy only the matching profile:

```bash
./deploy_arm_config.sh left
./deploy_arm_config.sh right
```

For local validation without replacing `client_config.ini`, select a profile with
Both devices use the same command:

```bash
python3 run_client.py
```

The Left device stores `left` and the Right device stores `right` in
`config/arm_identity`. `WASAB_ARM_ID` remains available only as a temporary
diagnostic override.
