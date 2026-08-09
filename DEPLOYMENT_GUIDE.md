# WaSaB 통합 시스템 배포 및 실행 가이드

이 문서는 `WaSaB_Integrated_Deploy_20260803.zip` 배포본을 기준으로 합니다.

## 1. 가장 빠른 실행 방법

1. ZIP을 `/home/ane/dev_ws/src/roscamp-repo-3` 경로에 풉니다.
2. Ubuntu 바탕화면의 **WaSaB 통합 실행기**를 더블클릭합니다.
3. 실행할 항목을 체크합니다.
4. AI Server IP, Left/Right Arm IP와 ROS Domain, Pinky IP와 Domain을 확인합니다.
5. **선택 항목 시작**을 누릅니다.
6. 통합 GUI가 자동으로 열리지 않으면 브라우저에서 `http://127.0.0.1:8100`에 접속합니다.

실행기를 직접 실행할 때는 다음 명령을 사용합니다.

```bash
cd /home/ane/dev_ws/src/roscamp-repo-3
python3 Service/WasabServer/Launcher/wasab_launcher.py
```

> 실행기는 프로젝트 위치를 자동으로 계산하지만, AI 서버 Python은 기본적으로
> `/home/ane/dev_ws/.venv-server/bin/python`을 사용합니다. 다른 PC에서는 실행기 상단의
> `SERVER_PYTHON` 설정 또는 가상환경 경로를 환경에 맞게 수정해야 합니다.

## 2. 배포 전 준비

### 노트북/서버

- Ubuntu 22.04 이상 권장
- Python 3.10 이상
- ROS 2 Humble 환경
- PyQt6
- AI 서버용 가상환경 `/home/ane/dev_ws/.venv-server`
- 로봇과 같은 LAN 연결

Python 의존성 설치 예시:

```bash
python3 -m venv /home/ane/dev_ws/.venv-server
/home/ane/dev_ws/.venv-server/bin/pip install -U pip
/home/ane/dev_ws/.venv-server/bin/pip install -r Service/WasabAIServer/AIService/ai_service/requirements.txt
/home/ane/dev_ws/.venv-server/bin/pip install -r Service/WasabAIServer/AIService/face-recog/requirements.txt
python3 -m pip install PyQt6
```

### 로봇팔 Raspberry Pi

- 왼팔 기본 IP: `192.168.2.10`
- 오른팔 기본 IP: `192.168.2.12`
- 작업 경로: `/home/jetcobot/wasab/roscamp-repo-3/Device/WasabBot/WasabArmController`
- Python: `/home/jetcobot/venv/wasabarm/bin/python`
- 카메라 및 `/dev/ttyJETCOBOT` 연결

팔 클라이언트 의존성:

```bash
/home/jetcobot/venv/wasabarm/bin/pip install -r Device/WasabBot/WasabArmController/requirements.txt
```

실행 전 각 장치에서 `config/client_config.ini`와 `config/arm_identity`를 왼팔 또는
오른팔에 맞게 배치합니다. 기준 파일은 `client_config.left.ini`,
`client_config.right.ini`입니다.

### 기본 네트워크

| 구성요소 | 기본 주소/설정 |
|---|---|
| AI Server | `192.168.2.8:8000` |
| 통합 Web GUI | `127.0.0.1:8100` |
| Left Arm | `192.168.2.10` |
| Right Arm | `192.168.2.12` |
| Console ROS Domain | `50` |
| Pinky-50/87/44/31 | 실행기에서 IP 및 Domain 설정 |

## 3. 통합 실행기 사용

실행 항목은 다음 세 범주로 나뉩니다.

1. **Server & GUI**: AI Server, 통합 GUI, DDS 준비
2. **Robot Arm**: Left Arm, Right Arm
3. **Mobile Robot**: 선택한 Pinky의 bringup/localization/navigation/agent

`ALL SELECT`로 전체를 선택하거나 해제할 수 있습니다. 실행기에서 입력한 IP와 ROS
Domain은 HTTP 프록시, SSH 실행, ROS 통신 설정에 반영됩니다. 로그는
`/tmp/wasab-launcher/`에 저장됩니다.

종료할 때는 로봇을 안전한 상태로 만든 뒤 실행기의 **전체 종료**를 사용합니다.
웹앱의 **비상정지**는 이동로봇과 Left/Right/Dual Arm에 STOP 명령을 전달합니다.

## 4. 수동 실행

### AI/로봇팔 서버

```bash
cd /home/ane/dev_ws/src/roscamp-repo-3/Service/WasabAIServer/AIService/ai_service
/home/ane/dev_ws/.venv-server/bin/python -u run_server.py
```

상태 확인:

```bash
curl http://192.168.2.8:8000/health
```

### 통합 WebService

```bash
cd /home/ane/dev_ws/src/roscamp-repo-3
./Service/WasabServer/scripts/run_webapp.sh
```

접속 주소: `http://127.0.0.1:8100`

### 왼팔/오른팔 클라이언트

각 Raspberry Pi에서:

```bash
cd /home/jetcobot/wasab/roscamp-repo-3/Device/WasabBot/WasabArmController
/home/jetcobot/venv/wasabarm/bin/python -u run_client.py
```

클라이언트 로그에 서버 연결, 카메라 초기화, `[READY]`가 표시되는지 확인합니다.

## 5. 웹앱 주요 기능과 로봇팔 매핑

| 웹 기능 | 실행 대상 | 명령 |
|---|---|---|
| 선물주기 | Dual Arm | `gift-giving` |
| 교보재 올리기 | Left Arm | `help` |
| 분리수거 | Left Arm | `recycle` |
| 비상정지 | 선택 장치 또는 전체 | `stop` |

관리자 메뉴의 **로봇암 현황**에서는 Left/Right/Dual Arm 상태, 카메라, 로그와 기능을
확인할 수 있습니다. 화재감지, 얼굴인식, 추종은 한 팔에서 상호 배타적으로 실행됩니다.

## 6. 폴더 구성

```text
roscamp-repo-3/
├── Device/                     로봇팔·이동로봇 장치 코드
│   └── WasabBot/WasabMoveController/
│       ├── wasab_navigation/   이동로봇 navigation ROS 패키지
│       └── wasab_robot_agent/  Pinky 상태·명령 중계 ROS 패키지
├── Service/
│   ├── WasabAIServer/          객체·얼굴·화재 인식과 로봇팔 API
│   └── WasabServer/            실행기, WebService, 운영 서비스, scripts
├── UI/
│   ├── mobile/user_gui/        통합 웹앱
│   └── pc/                     데스크톱 실행 항목과 Admin GUI
├── DEPLOYMENT_GUIDE.md         이 문서
└── CODE_STRUCTURE.html         코드 구조 및 데이터 흐름 설명
```

## 7. 문제 해결

### GUI가 열리지 않음

```bash
curl http://127.0.0.1:8100/api/session
```

응답이 없으면 `/tmp/wasab-launcher/` 로그와 WebService 실행 상태를 확인합니다.

### 로봇팔 버튼이 실패함

1. AI Server `http://<AI_SERVER_IP>:8000/health` 확인
2. 해당 팔의 `run_client.py` 실행 로그 확인
3. 카메라와 `/dev/ttyJETCOBOT` 연결 확인
4. 웹앱 **로봇암 현황 → 로봇팔 로그** 확인

### 얼굴인식 데이터

등록 얼굴은 다음 위치를 사용합니다.

```text
Service/WasabAIServer/FaceDB/known/<이름>/*.jpg
Service/WasabAIServer/FaceDB/encodings.pkl
```

### 보안 주의

- 배포 전에 실행기의 SSH 계정과 인증 정보를 운영 환경에 맞게 변경합니다.
- `teachers.json`에는 운영 계정 정보가 있으므로 외부 공개 배포 시 초기화합니다.
- 운영 환경에서는 WebService 쿠키 보안 및 허용 Origin을 HTTPS 구성에 맞게 설정합니다.

## 8. 배포본 무결성 확인

ZIP과 함께 제공되는 `.sha256` 파일이 있을 경우:

```bash
sha256sum -c WaSaB_Integrated_Deploy_20260803.zip.sha256
```
