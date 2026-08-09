# wasab_localization — odom+IMU EKF 융합 측위

천정 카메라 없는 로봇 자체 측위 스택에서, 휠 오도메트리에 공분산을 주입하고
`robot_localization` EKF로 인코더 vx + IMU yaw/yaw_rate를 융합한다. EKF가
`odom → base_footprint`(상대·평활)를 소유하고, AMCL이 `map → odom`(절대)을 담당한다.

전체 기술 검토: `docs/robot-localization-without-overhead-2026-06-29.md`.
설계: `docs/superpowers/specs/2026-06-29-ekf-odom-imu-localization-design.md`.

## 구조
```
/odom (공분산 0) → odom_cov_relay → /odom_cov ─┐
                                                ├→ ekf_filter_node → odom→base_footprint
imu_raw (yaw+yaw_rate) ────────────────────────┘   (two_d_mode)
/scan → AMCL → map→odom
bringup: launch에서 /tf 억제 → odom→base는 EKF가 소유 (upstream 무수정)
```

## 의존성
- 로봇에 `ros-jazzy-robot-localization` 설치 필요.
- `upstream_pinky_pro`는 **수정·복사하지 않고** launch에서 참조만 한다.

## 빌드 & 실행 (로봇)
```bash
colcon build --packages-select wasab_localization && source install/setup.bash
ros2 launch wasab_localization localization.launch.py map:=<map.yaml>
# WaSaB AMCL 튜닝값 사용 시:
#   ... amcl_params_file:=<wasab_nav2/params/nav2_params.yaml 경로>
# 휠 캘리브 결과 반영 시:
#   ... wheel_radius:=<r> wheel_separation:=<sep>
```

## 검증 (문서 §4.2 staged)
0. `ros2 run tf2_ros tf2_echo base_footprint rplidar_link` → static laser TF 존재 확인.
0. `ros2 topic hz /imu_raw` → IMU 발행 확인 (AMCL 전 선행조건).
1. `ros2 topic echo /odom_cov --once` → twist/pose covariance ≠ 0.
2. `ros2 run tf2_tools view_frames` → `odom→base_footprint`를 `ekf_filter_node`만 발행(bringup TF 억제), `/odometry/filtered` 발행.
3. 전체 스택 → `map→odom→base_footprint` 끊김 없음, amcl_pose 안정.

> 정량 정확도 평가는 휠 캘리브(`wasab_sensorpose`) 적용 + 맵 재작성 이후라야 의미.
> IMU 절대 yaw가 실내 교란되면 `config/ekf.yaml`의 `imu0_config`를 yaw_rate-only로 축소(폴백).

## 단위테스트 (HW 불필요)
```bash
cd wasab_navigation/wasab_localization && python3 -m pytest test -q
```
