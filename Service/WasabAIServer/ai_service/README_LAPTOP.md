# 노트북 로컬 WaSaB 서버 실행 가이드

이 문서는 `Service/WasabAIServer/ai_service`을 노트북에서 실행하는 방법을 설명합니다.

현재 서버는 다음 기능을 한 프로세스에서 처리합니다.

1. AdminGUI 제공
2. Arm 카메라 프리뷰 수신
3. YOLO 검출
4. 카메라 intrinsic + Eye-in-Hand hand-eye 보정 기반 2D→3D 파지 계획
5. Arm 클라이언트에 `flange_command` 반환

## 데이터 흐름

```text
WasabArm
  ├─ camera preview ───────→ WasabWebService /camera-view
  └─ image + flange pose ──→ WasabWebService /v1/grasp-plan
                              ├─ YOLO detect
                              ├─ WasabOpService 2D→3D planning
                              └─ flange_command response
```

실제 모터 명령은 노트북이 아니라 `Device/WasabBot/WasabArm` 클라이언트가 수행합니다.

## 실행 준비

```bash
cd /home/ane/dev_ws/wasab-manipulation/Service/WasabAIServer/ai_service
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r requirements.txt
```

## 필수 파일 확인

```bash
ls -lh models/best.pt
ls -lh calibration/camera_intrinsic_charuco.npz
ls -lh calibration/auto_handeye_result_20260619_162107.json
```

## 설정

```text
config/server_config.ini
```

주요 설정:

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

[udp_stream]
enabled = true
host = 0.0.0.0
port = 8001
```

GPU를 쓰지 않는 노트북에서는 `device = cpu`로 변경합니다.

## 서버 실행

```bash
python run_server.py
```

정상 실행 예:

```text
[LAPTOP] YOLO + grasp-plan service
[LAPTOP] listening on http://0.0.0.0:8000
[LAPTOP] Pi endpoint: /v1/grasp-plan (existing protocol preserved)
```

## 상태 확인

```bash
curl http://127.0.0.1:8000/health
```

정상 응답 예:

```json
{
  "status": "ok",
  "runtime": "laptop-local",
  "device": "cuda:0"
}
```

## AdminGUI

노트북에서:

```text
http://127.0.0.1:8000/camera-view
```

같은 LAN의 다른 장치에서:

```text
http://<노트북_LAN_IP>:8000/camera-view
```

## Arm 클라이언트 연결 설정

Arm 쪽 설정 파일:

```text
/home/ane/dev_ws/wasab-manipulation/Device/WasabBot/WasabArm/config/client_config.ini
```

노트북 LAN IP로 설정합니다.

```ini
[network]
grasp_server_url = http://<노트북_LAN_IP>:8000/v1/grasp-plan
```

## 단독 API 확인

이미지 파일이 있을 때:

```bash
curl -X POST "http://127.0.0.1:8000/v1/grasp-plan" \
  -F "image=@frame.jpg" \
  -F 'robot_state={"request_id":"test-001","flange_coords":[147.2,52.7,242.2,-177.51,5.1,-94.2]}'
```

## 결과 저장

Capture 버튼:

```text
app/components/capture/
```

Detect / grasp-plan 로그:

```text
laptop_detect_logs/<timestamp>/
```

로그 저장을 켜려면:

```ini
[logging]
save_results = true
save_root_dir = laptop_detect_logs
```

## 주의사항

- 서버 캘리브레이션 해상도는 640x480 기준입니다.
- Arm 카메라도 `frame_width = 640`, `frame_height = 480`이어야 합니다.
- Pi/Arm에서 `127.0.0.1`은 노트북이 아닙니다. 반드시 노트북 LAN IP를 사용합니다.
- 실제 로봇 테스트 전 Arm 쪽 `[safety] dry_run = true`로 먼저 확인합니다.
