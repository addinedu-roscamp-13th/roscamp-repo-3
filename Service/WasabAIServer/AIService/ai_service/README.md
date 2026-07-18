# WasabAIServer AI Service

`Service/WasabAIServer/ai_service`은 노트북에서 실행하는 WaSaB 서버 런타임입니다.

역할:

- FastAPI 서버 실행
- AdminGUI 제공
- Arm 카메라 프리뷰 수신
- YOLO 검출
- WasabOpService를 통한 2D→3D 파지 계획
- `/v1/grasp-plan` 응답 생성

## 실행 준비

```bash
cd /home/ane/dev_ws/wasab-manipulation/Service/WasabAIServer/ai_service
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## 필수 파일

```text
models/best.pt
calibration/camera_intrinsic_charuco.npz
calibration/auto_handeye_result_20260619_162107.json
```

## 설정

```text
config/server_config.ini
```

주요 항목:

```ini
[server]
host = 0.0.0.0
port = 8000

[model]
model_path = models/best.pt
device = cuda:0
default_conf = 0.5
default_imgsz = 640

[calibration]
intrinsic_file = calibration/camera_intrinsic_charuco.npz
handeye_result_json = calibration/auto_handeye_result_20260619_162107.json
euler_order = zyx

[request_validation]
expected_image_width = 640
expected_image_height = 480
```

CPU만 사용할 경우:

```ini
[model]
device = cpu
```

## 서버 실행

```bash
python run_server.py
```

상태 확인:

```bash
curl http://127.0.0.1:8000/health
```

정상 응답 예:

```json
{
  "status": "ok",
  "runtime": "laptop-local",
  "model_path": "models/best.pt",
  "device": "cuda:0"
}
```

## AdminGUI

노트북:

```text
http://127.0.0.1:8000/camera-view
```

같은 LAN의 다른 장치:

```text
http://<노트북_LAN_IP>:8000/camera-view
```

## 주요 API

| API | 방식 | 용도 |
| --- | --- | --- |
| `/health` | `GET` | 서버 상태 확인 |
| `/camera-view` | `GET` | AdminGUI 화면 |
| `/camera-frame` | `POST` | Arm 프리뷰 HTTP fallback 업로드 |
| `/camera-frame/latest.jpg` | `GET` | 최신 프레임 JPEG |
| `/camera-frame/stream.mjpg` | `GET` | MJPEG 프리뷰 스트림 |
| `/camera-frame/detect` | `POST` | 최신 프레임 YOLO 검출 |
| `/camera-frame/capture` | `POST` | 최신 프레임 저장 |
| `/grasp-plan` | `POST` | 파지 계획 요청 |
| `/v1/grasp-plan` | `POST` | 기존 프로토콜 호환 파지 계획 요청 |

## `/v1/grasp-plan` 요청 예

```bash
curl -X POST "http://127.0.0.1:8000/v1/grasp-plan" \
  -F "image=@frame.jpg" \
  -F 'robot_state={"request_id":"test-001","flange_coords":[147.2,52.7,242.2,-177.51,5.1,-94.2]}'
```

응답 예:

```json
{
  "status": "ok",
  "request_id": "test-001",
  "plan": {
    "target_label": "box",
    "flange_command": [152.6, 61.8, 160.0, -177.51, 5.1, -94.2]
  }
}
```

## 결과 저장

AdminGUI Capture:

```text
app/components/capture/
```

Detect / grasp-plan 로그:

```text
laptop_detect_logs/<timestamp>/
├── raw.jpg
├── annotated.jpg
└── result.json
```

로그 저장은 설정에서 켭니다.

```ini
[logging]
save_results = true
save_root_dir = laptop_detect_logs
```
