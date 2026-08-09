# README_Manipulation

이 문서는 `roscamp-repo-3` 안에 들어간 WaSaB manipulation 코드 실행 위치와 확인 방법만 정리합니다.
기존 루트 `README.md`는 변경하지 않았습니다.

## 현재 배치 구조

```text
roscamp-repo-3/
├── Device/
│   └── WasabBot/
│       ├── WasabArm/              # Jetcobot Arm 클라이언트
│       ├── WasabLeg/              # 기존 ROS2 leg/navigation 패키지
│       └── WasabHead/
├── Service/
│   ├── WasabAIServer/
│   │   ├── ai_service/            # 노트북 AI 서버: FastAPI + YOLO + AdminGUI
│   │   └── face_db/
│   ├── WasabOpService/            # 2D→3D 파지 좌표 계산 공용 코드
│   └── WasabServer/
│       └── wasab_db/
└── UI/
    ├── pc/admin_gui/
    └── mobile/user_gui/
```

## 노트북 AI 서버 실행

```bash
cd /home/ane/dev_ws/src/roscamp-repo-3/Service/WasabAIServer/ai_service
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
python run_server.py
```

상태 확인:

```bash
curl http://127.0.0.1:8000/health
```

AdminGUI:

```text
http://127.0.0.1:8000/camera-view
```

다른 장치에서는 노트북 LAN IP를 사용합니다.

```text
http://<노트북_LAN_IP>:8000/camera-view
```

## YOLO 모델 위치

서버는 아래 파일을 사용합니다.

```text
Service/WasabAIServer/ai_service/models/best.pt
```

설정 파일:

```text
Service/WasabAIServer/ai_service/config/server_config.ini
```

현재 설정의 기본 모델 경로:

```ini
[model]
model_path = models/best.pt
```

## Jetcobot Arm 클라이언트 실행

라즈베리파이/Jetcobot 제어 장치에서 실행합니다.

```bash
cd /home/ane/dev_ws/src/roscamp-repo-3/Device/WasabBot/WasabArm
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

노트북 LAN IP를 설정합니다.

```text
Device/WasabBot/WasabArm/config/client_config.ini
```

```ini
[network]
grasp_server_url = http://<노트북_LAN_IP>:8000/v1/grasp-plan
```

연결 확인:

```bash
python3 check_server_connection.py
```

클라이언트 실행:

```bash
python3 run_client.py
```

## 주요 명령

| 키/명령 | 기능 |
| --- | --- |
| `g` / `pick` | 현재 카메라 이미지와 Flange pose를 서버로 보내고 pick 수행 |
| `f` / `place` | home 이동 후 place 위치로 이동하고 그리퍼 열기 |
| `p` / `pose` | 현재 Flange pose 출력 |
| `q` / `gripper` | 그리퍼 열기/닫기 토글 |
| `r` / `random` | home 주변 안전 random pose로 이동 |
| `a` / `recycle` | 왼팔이 `trash`는 빨간 박스, `water`는 파란 박스로 분류 |
| `help` | 왼팔이 AprilTag ID 0 물체를 픽업한 뒤 기존 Place 동작 실행 |
| `w` / `home` | home 위치로 이동 |
| `space` / `stop` | 현재 동작 즉시 정지 |
| `x` / `exit` | 클라이언트 종료 |

## 결과 저장 위치

AdminGUI `Capture` 버튼:

```text
Service/WasabAIServer/ai_service/app/components/capture/
```

Detect / grasp-plan 로그:

```text
Service/WasabAIServer/ai_service/laptop_detect_logs/<timestamp>/
```

로그 저장은 서버 설정에서 켭니다.

```ini
[logging]
save_results = true
save_root_dir = laptop_detect_logs
```

## 안전 확인

실제 로봇을 움직이기 전에는 Arm 설정에서 dry-run을 권장합니다.

```ini
[safety]
dry_run = true
```

충분히 검증한 뒤에만 `dry_run = false`로 변경합니다.

## 참고 리포트

컴포넌트 간 통신 흐름 샘플:

```text
wasab_architecture_communication_report.html
```
