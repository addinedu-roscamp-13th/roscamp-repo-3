# wasab_docking/geometry.py
"""정밀 주차 순수 기하 헬퍼 (ROS 무관, pytest). 각도·쿼터니언·축변환·오차."""
import math


def normalize_angle(rad):
    """각도를 [-pi, pi]로 정규화."""
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_from_quat(x, y, z, w):
    """평면 yaw만 추출(z축 회전)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw):
    """yaw(z축 회전)만 갖는 단위 쿼터니언 (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def yaw_about_vertical_from_R(R):
    """solvePnP 회전행렬 R에서 카메라 수직축(광학 Y, down) 기준 tag 회전각.

    R은 tag→camera 회전. 순수 Y축 회전 R_y(θ)=[[c,0,s],[0,1,0],[-s,0,c]]에서
    θ=atan2(-R[2][0], R[0][0]). fronto-parallel일 때 0.
    (asin(-R[2][0]) 형태는 pitch/elevation이므로 쓰지 않는다.)
    """
    return math.atan2(-R[2][0], R[0][0])


def camera_optical_to_base(cam_xyz, cam_yaw, extrinsic):
    """카메라 광학 프레임 tag pose → base_footprint 기준 (x, y, yaw).

    광학 프레임(OpenCV): x=right, y=down, z=forward.
    base_footprint(REP-103): x=forward, y=left, z=up.
    base-Z(up) = -camera-Y(down)이므로 yaw 부호가 뒤집힌다.
    extrinsic 회전은 yaw만 반영(roll/pitch=0 전제, Phase 1).
    """
    cx, cy, cz = cam_xyz
    base_x = cz + extrinsic["x"]      # forward
    base_y = -cx + extrinsic["y"]     # left
    base_yaw = normalize_angle(-cam_yaw + extrinsic.get("yaw", 0.0))
    return (base_x, base_y, base_yaw)


def compute_errors(tag_base, tag_goal):
    """base 기준 tag pose(x, y, yaw)와 목표 pose의 오차. error = 현재 - 목표."""
    tx, ty, tyaw = tag_base
    return {
        "x": tx - tag_goal["x"],
        "y": ty - tag_goal["y"],
        "yaw": normalize_angle(tyaw - tag_goal["yaw"]),
    }


def se2_compose(a, b):
    """SE2 pose 합성 a⊙b. a,b = (x,y,yaw). frame a에서 표현된 b를 a의 상위 frame으로."""
    ax, ay, ayaw = a
    bx, by, byaw = b
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (ax + bx * c - by * s,
            ay + bx * s + by * c,
            normalize_angle(ayaw + byaw))


def se2_inverse(a):
    """SE2 pose 역변환."""
    ax, ay, ayaw = a
    c, s = math.cos(ayaw), math.sin(ayaw)
    return (-(ax * c + ay * s),
            ax * s - ay * c,
            normalize_angle(-ayaw))
