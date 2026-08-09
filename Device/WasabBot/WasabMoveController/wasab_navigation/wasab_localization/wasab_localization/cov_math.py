"""오도메트리 공분산 배열 구성 — 순수 함수 (ROS/HW 무관)."""


def covariance_matrix(diag6):
    """길이 6 대각값 → 6×6 row-major 36요소 공분산 (대각만, 나머지 0)."""
    if len(diag6) != 6:
        raise ValueError("diag6는 길이 6이어야 합니다")
    m = [0.0] * 36
    for i, v in enumerate(diag6):
        m[i * 6 + i] = float(v)
    return m


def twist_covariance(vx, vy, vz, vroll, vpitch, vyaw):
    return covariance_matrix([vx, vy, vz, vroll, vpitch, vyaw])


def pose_covariance(x, y, z, roll, pitch, yaw):
    return covariance_matrix([x, y, z, roll, pitch, yaw])
