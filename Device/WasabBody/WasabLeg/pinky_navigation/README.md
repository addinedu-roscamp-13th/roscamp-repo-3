# 순찰 로봇 — 실행 방법

Nav2를 이용한 자율주행. 별도의 자동 순찰 코드 없이, RViz에서 지점을 클릭하면 Nav2가 알아서 경로계획+주행한다.

이 폴더의 `params/nav2_params1.yaml`, `map/wasab_map6.yaml`(+`.pgm`)이 실제로 튜닝/사용 중인 파일이다. (`nav2_params.yaml`은 팀 기본 템플릿이라 건드리지 않음)

## 실행 순서 (PinkyPro, 3개 터미널)

```bash
# ① 로봇 기본 구동 (모터/센서 등)
ros2 launch pinky_bringup bringup_robot.launch.xml
```

```bash
# ② 위치 추정 (AMCL) — map/params 경로는 본인 배포 위치에 맞게 수정
ros2 launch pinky_navigation localization_launch.xml \
  map:=<repo경로>/pinky_navigation/map/wasab_map6.yaml \
  params_file:=<repo경로>/pinky_navigation/params/nav2_params1.yaml
```

```bash
# ③ 주행 (Nav2 스택)
ros2 launch pinky_navigation navigation_launch.xml \
  params_file:=<repo경로>/pinky_navigation/params/nav2_params1.yaml
```

## 이동시키기

PC에서 RViz를 띄운 뒤, 상단 툴바의 **"Nav2 Goal"** 버튼으로 지도 위 원하는 지점을 클릭하면 그 위치까지 자동으로 이동한다. 별도 코드 작성 불필요.
