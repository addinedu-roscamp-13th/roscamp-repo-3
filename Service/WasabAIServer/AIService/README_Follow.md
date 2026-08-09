# 추종 로봇 — 실행 방법

PinkyPro가 등록된 얼굴을 인식해서 따라다니고, JetCobot 제스처(주먹/검지)로 시작·정지를 제어하는 기능이다.

## 구성 요소

| 위치 | 실행 기기 | 역할 |
|---|---|---|
| `pinky_yolo/` | PC | 얼굴 인식 + 추종 제어 AI (`run_ai.sh`) |
| `face-recog/` | PC | JetCobot 카메라로 제스처 인식 (`start_laptop.sh`) |
| `../../Device/WasabBot/WasabArmController/wasab_k3_mimic/` + `start_rpi.sh` | JetCobot | 카메라 스트리밍 + 팔 얼굴 추종 |
| `../../Device/WasabBot/WasabMoveController/pinky_node.py` | PinkyPro | 카메라 발행 + 모터/LED 제어 |

JetCobot·PinkyPro용 코드는 각 로봇 자신의 워크스페이스(보통 `~/wasab/`)에 복사해서 실행한다.

## 1. 사전 준비

```bash
source /opt/ros/jazzy/setup.bash
sudo apt install python3-colcon-common-extensions -y

# pinky_yolo 빌드 (레포가 속한 colcon 워크스페이스 루트에서)
colcon build --packages-select pinky_yolo
source install/setup.bash

# 파이썬 패키지 (pinky_yolo용)
pip install insightface onnxruntime ultralytics opencv-python --break-system-packages
```

`face-recog`는 별도 venv를 쓴다:

```bash
python3 -m venv ~/face-recog/.venv
~/face-recog/.venv/bin/pip install -r face-recog/requirements.txt
```

InsightFace 얼굴 모델(`buffalo_sc`)은 최초 실행 시 자동 다운로드된다.

## 2. 본인 ROS_DOMAIN_ID 확인

로봇/PC마다 설정된 값이 다를 수 있으니 먼저 확인하고, 아래 실행 시 그 값을 쓴다.

```bash
echo $ROS_DOMAIN_ID
```

## 3. 얼굴 등록

`pinky_yolo/register.ipynb`를 실행해서 추종 대상 얼굴을 등록한다.

```bash
cd pinky_yolo
jupyter notebook register.ipynb
```

- `SPACE`: 3-2-1 카운트다운 후 촬영 (5장 이상 권장, 각도/조명 다양하게)
- `d`: 마지막 사진 삭제
- `q`: 등록 종료

등록 결과는 `pinky_yolo/face_db/known/<이름>/*.jpg`에 저장된다.

## 4. 실행 순서

4개 프로세스를 각각 별도 터미널에서, 이 순서로 띄운다.

```bash
# ① JetCobot (SSH 접속 후, ~/wasab/에 배포된 상태에서)
./start_rpi.sh
# → 카메라 스트리밍 + 팔 얼굴 추종 시작

# ② PC — 제스처 인식
cd face-recog
./scripts/start.sh --show-view
# → 손 제스처 인식 창이 뜸

# ③ PC — 추종 AI
cd pinky_yolo
bash run_ai.sh
# → 등록된 얼굴 목록에서 추종 대상 번호 선택 → 대기 상태로 시작

# ④ PinkyPro (SSH 접속 후, ~/wasab/에 배포된 상태에서)
python3 pinky_node.py
```

①→②→③→④ 순서를 지킬 것. ③은 시작 직후엔 대기 상태라, 제스처로 START를 보내야 실제로 움직인다.

## 5. 조작

| 제스처 / 키 | 동작 |
|---|---|
| ✊ 주먹 | 추종 정지 (LED 빨강 3초) |
| ☝️ 검지 펴고 정지 | 추종 시작/재개 (LED 초록) |
| `Space` (③번 창) | 제스처와 동일하게 시작/정지 토글 |
| `q` (③번 창) | 종료 |
