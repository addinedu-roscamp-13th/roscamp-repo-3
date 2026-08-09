# WaSaB Dual Arm 실행 가이드

## 요약 실행 방법

바탕화면의 **WaSaB 통합 실행기**를 더블클릭합니다. 실행기가 다음 항목을
순서대로 자동으로 켜고 브라우저에서 통합 GUI를 엽니다.

1. AI/로봇팔 서버
2. 왼팔 클라이언트
3. 오른팔 클라이언트
4. 통합 WebService GUI (`http://127.0.0.1:8100`)

최초 실행에서만 관리자 아이디와 8자 이상의 비밀번호를 입력합니다. 이미 실행
중인 서버나 팔 클라이언트는 중복 실행하지 않습니다. 종료할 때는 실행기의
**전체 종료** 버튼을 사용합니다.

## 수동 실행 방법

아래 명령은 실행기를 사용할 수 없을 때의 점검용 방법입니다. 실행 순서는
**AI/GUI 서버 → 왼팔 클라이언트 → 오른팔 클라이언트**입니다.

### 1. 노트북 AI/GUI 서버

```bash
cd /home/ane/dev_ws/src/roscamp-repo-3/Service/WasabAIServer/AIService/ai_service
/home/ane/dev_ws/.venv-server/bin/python -u run_server.py
```

### 2. 왼팔 Raspberry Pi

```bash
ssh jetcobot@192.168.2.10
cd /home/jetcobot/wasab/roscamp-repo-3/Device/WasabBot/WasabArmController
/home/jetcobot/venv/wasabarm/bin/python -u run_client.py
```

### 3. 오른팔 Raspberry Pi

```bash
ssh jetcobot@192.168.2.12
cd /home/jetcobot/wasab/roscamp-repo-3/Device/WasabBot/WasabArmController
/home/jetcobot/venv/wasabarm/bin/python -u run_client.py
```

### 4. AdminGUI 접속

웹 브라우저에서 다음 주소를 엽니다.

```text
http://192.168.2.8:8000/camera-view
```

## 상세 실행 방법

### 실행 전 확인

- 로봇팔 주변에 사람과 장애물이 없는지 확인합니다.
- 카메라와 USB 시리얼 케이블이 연결되어 있어야 합니다.
- 노트북 IP는 `192.168.2.8`을 기준으로 설정되어 있습니다.
- 왼팔 주소는 `192.168.2.10`, 오른팔 주소는 `192.168.2.12`입니다.
- 서버의 TCP `8000`, UDP `8001` 포트를 사용할 수 있어야 합니다.

### AI/GUI 서버 실행 확인

서버 실행 후 다음 메시지가 표시되면 준비된 상태입니다.

```text
Uvicorn running on http://192.168.2.8:8000
UDP Streamer receiver: 0.0.0.0:8001
```

상태 API로도 확인할 수 있습니다.

```bash
curl http://192.168.2.8:8000/health
```

### 로봇 클라이언트 실행 확인

각 Raspberry Pi에서 다음 메시지가 표시되어야 합니다.

```text
[NETWORK] laptop server reachable
[CAMERA] calibrated capture size: 640x480
[READY] Robot client is running
```

왼팔과 오른팔은 각각 다음 설정을 사용합니다.

```text
config/client_config.left.ini
config/client_config.right.ini
```

실제 실행 설정인 `config/client_config.ini`와 `config/arm_identity`가 해당 팔에 맞게 배치되어 있어야 합니다.

## AdminGUI 팔 선택

- **Left Arm**: 왼팔 카메라와 왼팔 명령을 사용합니다.
- **Right Arm**: 오른팔 카메라와 오른팔 명령을 사용합니다.
- **Dual Arm**: Home, Dual STOP, Gift Giving 기능을 사용합니다.

STOP은 현재 선택한 팔의 동작을 멈춥니다. Dual STOP은 양쪽 팔을 모두 멈춥니다.

## 주요 기능

### 공통 기능

- **Live**: 실시간 카메라 영상을 표시합니다.
- **Detect**: 최신 영상에서 YOLO 객체 검출을 실행합니다.
- **Capture**: 현재 카메라 영상을 서버에 저장합니다.
- **Pickup**: 검출한 물체를 집습니다.
- **Place**: 들고 있는 물체를 설정된 위치에 놓습니다.
- **Pick & Place**: 픽업과 플레이스를 연속 실행합니다.
- **Home**: 해당 팔을 설정된 홈 자세로 이동합니다.
- **Pose**: 현재 Flange 좌표와 관절각을 로그에 출력합니다.
- **Gripper**: 그리퍼를 열거나 닫습니다.
- **Servo**: 서보 활성화와 해제를 전환합니다.

### 왼팔 기능

- **Restock**: 저장된 보충 위치를 기준으로 물체를 보충합니다.
- **Recycle**: trash는 빨간 상자, water는 파란 상자로 분류합니다.
- **Help**: AprilTag ID 0 물체를 픽업하고 지정된 위치에 놓습니다.
- **Fire Detect**: 화염을 탐색하고 검출 위치를 추종합니다.
- **Face Recognition**: 얼굴을 탐색하고 미등록 얼굴 위치를 추종합니다.
- **Tracking**: 원본 perception 기능으로 등록된 얼굴을 찾아 화면 중앙으로 추종합니다.

### 오른팔 기능

- **Gesture**: 손바닥 제스처 인식을 켜거나 끕니다.
- **Fire Detect**: 오른팔 영상으로 화염 탐색과 추종을 실행합니다.
- **Face Recognition**: 오른팔 영상으로 얼굴 탐색과 추종을 실행합니다.
- **Tracking**: 오른팔 영상에서 등록 얼굴을 탐색하고 추종합니다.

### 듀얼암 기능

- **Gift Giving**: 왼팔 준비 동작 후 오른팔 전달 동작을 실행합니다.
- **Home**: 양쪽 팔에 Home 명령을 전달합니다.

## 얼굴인식과 화염감지

Face Recognition, Fire Detect, Tracking은 같은 팔에서 동시에 실행되지 않습니다.

- Face Recognition을 켜면 Fire Detect가 자동으로 꺼집니다.
- Fire Detect를 켜면 Face Recognition이 자동으로 꺼집니다.
- Tracking을 켜면 Face Recognition과 Fire Detect가 자동으로 꺼집니다.
- 다른 기능을 켜면 기존 Tracking도 자동으로 꺼집니다.
- 기능을 켜면 SEARCH 스위핑을 시작합니다.
- 대상이 검출되면 SEARCH를 멈추고 화면 중앙으로 TRACK합니다.
- 얼굴은 2초, 화염은 1초 동안 놓치면 마지막 추종 위치 중심으로 재탐색합니다.
- Tracking은 등록된 얼굴을 대상으로 하며 2초 동안 놓치면 마지막 위치 중심으로 재탐색합니다.

공통 스위핑 설정은 다음과 같습니다.

```text
home = [0, 0, 0, -15, 0, -135]
search speed = 10
yaw step = 14 degrees
pitch step = 6 degrees
yaw limit = 90 degrees
pitch limit = 50 degrees
search dwell = 1.0 second
```

화재가 확정되면 AdminGUI에 다음 메시지 박스가 나타납니다.

```text
진압을 시작할까요?
```

- **Yes**: 화염 상태머신에 진압 응답을 전달합니다.
- **No**: 진압을 취소하고 순찰 재개 상태로 전환합니다.
- 15초 동안 응답이 없으면 자동으로 취소됩니다.

검출 결과는 AdminGUI의 Operation logs에서 실시간으로 확인할 수 있습니다.

## 종료 방법

각 터미널에서 `Ctrl+C`를 누릅니다. GUI에서는 STOP으로 움직임을 먼저 멈춘 후 Exit을 사용할 수 있습니다.

프로세스를 확인하려면 다음 명령을 사용합니다.

```bash
ps -ef | grep run_server.py
ps -ef | grep run_client.py
```

## 문제 확인

### GUI가 열리지 않는 경우

```bash
curl http://192.168.2.8:8000/health
```

서버가 응답하지 않으면 AI/GUI 서버부터 다시 실행합니다.

### 카메라가 나오지 않는 경우

Raspberry Pi에서 장치를 확인합니다.

```bash
ls -l /dev/video*
ls -l /dev/v4l/by-id/
```

다른 프로세스가 카메라를 점유하지 않았는지도 확인합니다.

```bash
fuser /dev/video0
```

### 팔이 움직이지 않는 경우

- AdminGUI에서 올바른 팔이 선택되었는지 확인합니다.
- 해당 팔 클라이언트에 `[READY]`가 출력되었는지 확인합니다.
- Servo가 `FOCUSED` 상태인지 확인합니다.
- Operation logs에서 명령 전달과 오류 메시지를 확인합니다.

### 얼굴이 모두 unknown으로 나오는 경우

얼굴 데이터는 다음 위치에 있어야 합니다.

```text
Service/WasabAIServer/FaceDB/known/<이름>/*.jpg
Service/WasabAIServer/FaceDB/encodings.pkl
```
# 최신 통합본 화재 감지·얼굴 추종 주행

`wasab_통합`의 2026-07-31 기준 코드를 현재 듀얼암 프로젝트 구조에 병합했다. 기존 듀얼암 GUI의
`Fire Detect`/`Face Recognition` 버튼은 이전과 같은 명령으로 실행되며, 아래 항목은 로봇 실기용
최신 경로다.

## 화재 감지 (JetCobot 카메라 로컬 처리)

JetCobot에 아래 파일과 `wasab_k3_mimic` 패키지를 배포한 후 실행한다.

```bash
cd Service/WasabAIServer/AIService/face-recog
./run_fire_detect_arm.sh <PC_IP>
```

감지는 JetCobot에서 직접 처리하고, `<PC_IP>`에는 오버레이 영상만 UDP로 전송한다. 같은 카메라를
사용하는 `cam_server.py`와 동시에 실행하면 안 된다. PC에서 화면을 볼 때는 같은 폴더에서
`python3 view_stream.py`를 실행한다.

## 얼굴 인식 추종 주행

```bash
cd Service/WasabAIServer/AIService/pinky_yolo
./run_ai.sh
```

`run_ai.sh`는 현재 ROS 워크스페이스를 자동으로 찾아 `pinky_yolo`만 빌드한다. 기본값은
`ROS_DOMAIN_ID=51`, 로봇 ID `50`, 콘솔 도메인 `50`이며 각각 환경변수 `ROS_DOMAIN_ID`,
`WASAB_ROBOT_ID`, `WASAB_CONSOLE_DOMAIN`으로 바꿀 수 있다. PinkyPro에서는 최신
`Device/WasabBot/WasabMoveController/pinky_node.py`를 배포해 실행한다.

PC·JetCobot·PinkyPro의 추종 구성 전체를 한 번에 올릴 때는 저장소 루트에서
`Service/WasabServer/scripts/run_follow_all.sh`를 사용한다. 순찰 주행은
`Service/WasabServer/scripts/run_patrol.sh`를 사용한다.
