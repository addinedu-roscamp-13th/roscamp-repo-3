# wasab_docking/pid.py
"""정밀 주차 순수 servo 제어 (ROS 무관, pytest).

diff-drive를 태그 정면 goal(tag_goal_x, 0, 0)로 몰기 위한 pose regulation.
목표 주차 자세 G(base 프레임) = se2_compose(tag_base, inv(tag_goal)) 를 극좌표(ρ,α,β)로
추종한다. '제자리 회전만으로는 좌우 offset을 못 고치는' 단순 P-제어의 구조적 한계를 해결한다
(호를 그리며 태그 법선 위로 올라타 ey→0, eyaw→0, ex→0 = 정면 tag_goal_x 로 수렴).
"""
import math

from wasab_docking import geometry as g


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def within_tolerance(errors, tols):
    """x/y/yaw 오차가 모두 tolerance 이내면 True."""
    return (
        abs(errors["x"]) <= tols["x"]
        and abs(errors["y"]) <= tols["y"]
        and abs(errors["yaw"]) <= tols["yaw"]
    )


def target_pose(errors, tag_goal):
    """현재 base 프레임에서 '도킹 완료 시 로봇이 있어야 할 자세' G = (gx, gy, gθ).

    tag_base = errors + tag_goal(성분별), G = tag_base ⊙ inv(tag_goal).
    """
    tag_base = (
        errors["x"] + tag_goal["x"],
        errors["y"] + tag_goal["y"],
        g.normalize_angle(errors["yaw"] + tag_goal["yaw"]),
    )
    goal = (tag_goal["x"], tag_goal["y"], tag_goal["yaw"])
    return g.se2_compose(tag_base, g.se2_inverse(goal))


def servo_cmd(errors, tag_goal, gains, limits, tols):
    """오차 → (vx, wz). pose regulation(ρ,α,β) + deadband + clamp.

    로봇은 원점·heading 0. G가 뒤에 있으면(overshoot) 후진으로 재접근한다.
    필요 gains: k_rho(>0), k_alpha(>k_rho), k_beta(<0), kyaw(최종 정렬).
    """
    if within_tolerance(errors, tols):
        return {"vx": 0.0, "wz": 0.0}

    gx, gy, gth = target_pose(errors, tag_goal)
    rho = math.hypot(gx, gy)
    max_wz = limits["max_wz"]

    if rho < tols["x"]:                       # 위치는 도달 → 최종 yaw 정렬만
        return {"vx": 0.0, "wz": clamp(gains["kyaw"] * gth, -max_wz, max_wz)}

    heading = math.atan2(gy, gx)              # 로봇→G 방향(robot 프레임)
    reverse = abs(heading) > math.pi / 2.0    # G가 뒤쪽 → 후진 접근
    alpha = g.normalize_angle(heading - math.pi) if reverse else heading
    beta = g.normalize_angle(gth - alpha)     # -θ - α + θ_goal (θ=0)

    v = gains["k_rho"] * rho
    if reverse:
        v = -v
    wz = gains["k_alpha"] * alpha + gains["k_beta"] * beta

    return {
        "vx": clamp(v, -limits["max_vx_back"], limits["max_vx"]),
        "wz": clamp(wz, -max_wz, max_wz),
    }
