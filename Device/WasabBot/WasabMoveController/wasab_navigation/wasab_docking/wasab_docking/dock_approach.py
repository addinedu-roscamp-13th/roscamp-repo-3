"""등록 태그 pose + standoff → Nav2 approach pose (순수, rclpy 무관, pytest).

servo 규약: docked 시 tag_in_base=(tag_goal_x,0,0). standoff 지점의 목표 관측은
tag_in_base=(standoff,0,0)이므로 approach = tag_in_map ⊙ inverse((standoff,0,0)).
결과 = 태그 정면 standoff, 로봇이 태그를 향함(yaw=태그 yaw).
"""
import math

from wasab_docking import geometry as g
from wasab_docking import tag_map_poses as tmp


def approach_pose(tag_cfg, tag_id, standoff_m):
    tag = tmp.get(tag_cfg, tag_id)          # (x,y,yaw) | None
    if tag is None:
        return None
    return g.se2_compose(tag, g.se2_inverse((float(standoff_m), 0.0, 0.0)))


def needs_nav(tag_cfg, tag_id, robot_xy, standoff_m):
    """Nav2 접근이 필요한가 = 로봇이 standoff 바깥에 있는가.

    이미 standoff 안쪽이면 approach pose가 로봇 뒤에 놓여, 좁은 공간에서
    후진/제자리회전 기동이 발생한다(RPP allow_reversing:false → 180도 회전 2회).
    가까우면 Nav2를 건너뛰고 PID 서보만으로 붙는다(카메라 closed-loop이라 odom 오차 무관).
    판단 불가(pose 미상/미등록 태그)면 기존 동작 유지.
    """
    tag = tmp.get(tag_cfg, tag_id)
    if tag is None or robot_xy is None:
        return True
    return math.hypot(robot_xy[0] - tag[0], robot_xy[1] - tag[1]) > float(standoff_m)
