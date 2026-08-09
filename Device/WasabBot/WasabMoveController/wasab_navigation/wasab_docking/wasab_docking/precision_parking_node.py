#!/usr/bin/env python3
# wasab_docking/precision_parking_node.py
"""/wasab/tag_pose 구독 → DockingStateMachine → /cmd_vel + /wasab/docking_state.

Phase 2: nav_enabled면 Nav2 NavigateToPose 접근 후 정밀 정차. Nav2 접근 중엔
/cmd_vel을 발행하지 않는다(소유권 분리). shutdown/cancel 시 항상 zero(견고화).
"""
import json
import os
import subprocess

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String

from wasab_docking import geometry as g
from wasab_docking.state_machine import DockingStateMachine
from wasab_docking.pose_filter import PoseFilter, YawFilter
from wasab_docking.nav_client import NavClientAdapter
from wasab_docking import estop as es

_NO_PUBLISH = ("NAV_TO_APPROACH", "NAV_CANCELING")   # Nav2 소유 → precision 발행 금지


class PrecisionParkingNode(Node):
    def __init__(self):
        super().__init__("precision_parking")
        p = self.declare_parameter
        cfg = {
            "gains": {"kx": p("kx", 0.35).value, "ky": p("ky", 0.8).value, "kyaw": p("kyaw", 0.9).value,
                      "k_rho": p("k_rho", 0.5).value, "k_alpha": p("k_alpha", 1.2).value,
                      "k_beta": p("k_beta", -0.4).value},
            "limits": {"max_vx": p("max_vx", 0.02).value, "max_vx_back": p("max_vx_back", 0.01).value,
                       "max_wz": p("max_wz", 0.20).value},
            "tols": {"x": p("tol_x", 0.015).value, "y": p("tol_y", 0.010).value, "yaw": p("tol_yaw", 0.04).value},
            "tag_goal": {"x": p("tag_goal_x", 0.25).value, "y": p("tag_goal_y", 0.0).value,
                         "yaw": p("tag_goal_yaw", 0.0).value},
            "tag_lost_timeout_s": p("tag_lost_timeout_s", 0.5).value,
            "settle_time_s": p("settle_time_s", 0.4).value,
            "settle_min_frames": p("settle_min_frames", 5).value,
            "overall_timeout_s": p("overall_timeout_s", 45.0).value,
            "search_timeout_s": p("search_timeout_s", 10.0).value,
            "nav_enabled": p("nav_enabled", False).value,
            "approach_pose_set": p("approach_pose_set", False).value,
            "nav_result_timeout_s": p("nav_result_timeout_s", 60.0).value,
            "nav_cancel_wait_s": p("nav_cancel_wait_s", 3.0).value,
        }
        self.tag_goal = cfg["tag_goal"]
        self.rate_hz = p("control_rate_hz", 20.0).value
        self.cmd_vel_enabled = p("cmd_vel_enabled", False).value
        self.goal_id = p("goal_id", "dock_a").value
        self.tag_id = p("tag_id", 7).value
        self.approach_pose = (p("approach_pose_x", 0.0).value,
                              p("approach_pose_y", 0.0).value,
                              p("approach_pose_yaw", 0.0).value)
        self.filt = PoseFilter(p("max_pose_jump_m", 0.15).value, p("max_yaw_jump_rad", 0.5).value)
        self.yawfilt = YawFilter(p("yaw_filter_window", 6).value)   # 평면 마커 yaw ±flip 안정화
        self.sm = DockingStateMachine(cfg)
        self.nav = NavClientAdapter(self, "/navigate_to_pose", p("nav_server_wait_s", 5.0).value) \
            if cfg["nav_enabled"] else None

        self._latest_errors = None
        self._last_msg_time = None
        self._prev_state = None
        self._prev_nav_status = None              # nav_status 변화 로그용(진단)
        self._estop = False                       # E-STOP 래치(한번 켜지면 유지) — 안전 최우선
        self._detector_proc = None       # 도킹 detector 서브프로세스(SEARCH_TAG에서 기동)
        self._fatal_reason = None         # 노드레벨 치명오류(detector_spawn_failed/detector_exited)

        self.pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pub_state = self.create_publisher(String, "/wasab/docking_state", 10)
        self.create_subscription(PoseStamped, "/wasab/tag_pose", self._on_tag, 10)
        self.create_subscription(String, "/wasab/estop", self._on_estop, 10)   # 콘솔 E-STOP → 하드정지

        self.filt.reset()                         # 새 goal 시작 → 필터 리셋(리뷰 2차 #1)
        self.yawfilt.reset()
        self.sm.start(self._now())
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"precision_parking 시작: nav_enabled={cfg['nav_enabled']} "
            f"approach_pose_set={cfg['approach_pose_set']} approach_pose={self.approach_pose} "
            f"cmd_vel_enabled={self.cmd_vel_enabled}")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_tag(self, msg):
        yaw = g.yaw_from_quat(msg.pose.orientation.x, msg.pose.orientation.y,
                              msg.pose.orientation.z, msg.pose.orientation.w)
        pose = (msg.pose.position.x, msg.pose.position.y, yaw)
        if not self.filt.accept(pose):            # outlier → 이번 pose 폐기(리뷰 #4)
            return
        yaw_f = self.yawfilt.update(yaw)          # yaw ±flip을 circular mean으로 안정화
        self._latest_errors = g.compute_errors((pose[0], pose[1], yaw_f), self.tag_goal)
        self._last_msg_time = self._now()

    def _on_estop(self, msg):
        # E-STOP 수신 → 래치(유지). 상태머신·cmd_vel_enabled 무관하게 _tick이 zero 강제.
        if es.parse_estop(msg.data) and not self._estop:
            self._estop = True
            self.get_logger().warn("E-STOP 수신 → 하드정지(zero cmd_vel 래치). 재개는 노드 재시작.")

    def _tick(self):
        # ★안전 최우선: E-STOP 래치면 상태/게이트 전부 무시하고 무조건 zero cmd_vel.
        if self._estop:
            self._publish_cmd(0.0, 0.0)
            self._safe_publish(self.pub_state, String(data=json.dumps(
                {"goal_id": self.goal_id, "state": "ESTOP", "estop": True})))
            return
        # 도킹 detector 치명오류 → FSM 안 돌리고 FAILED 발행(매 tick, agent 수신까지)
        if self._fatal_reason is not None:
            self._publish_fatal()
            return
        now = self._now()
        errors = self._latest_errors
        if self._last_msg_time is None or (now - self._last_msg_time) > self.sm.cfg["tag_lost_timeout_s"]:
            errors = None
        nav_status = self.nav.status if self.nav is not None else None
        out = self.sm.update(now, errors, nav_status)

        # 도킹 detector 지연기동 + 생존 감시
        self._maybe_spawn_detector(out["state"])
        if out["state"] == "SEARCH_TAG" and errors is None:
            self._check_detector_alive()
            if self._fatal_reason is not None:
                self._publish_fatal()
                return
        if out["state"] in ("DONE", "FAILED"):
            self._kill_detector()

        # nav 부수효과
        if out["nav_cmd"] == "send" and self.nav is not None:
            self.nav.send_goal(self.approach_pose)
        elif out["nav_cmd"] == "cancel" and self.nav is not None:
            self.nav.cancel()

        # 상태 전이 시 필터 리셋 + 진단 로그(라이브 관측 대체)
        if out["state"] != self._prev_state:
            self.get_logger().info(
                f"docking 상태전이: {self._prev_state} → {out['state']} "
                f"(nav_status={nav_status}, tag={errors is not None}, fail={out['fail_reason']})")
            if out["state"] in ("SEARCH_TAG", "FAILED", "DONE"):
                self.filt.reset()
                self.yawfilt.reset()
            self._prev_state = out["state"]

        # nav_status 변화 로그: 'active'에 머무는지/succeeded 도달인지/failed인지 = 이번 증상 핵심
        if nav_status != self._prev_nav_status:
            self.get_logger().info(f"nav_status: {self._prev_nav_status} → {nav_status}")
            self._prev_nav_status = nav_status

        # /cmd_vel 소유권: NAV/CANCELING 중 발행 금지(핵심 안전)
        if out["state"] not in _NO_PUBLISH:
            self._publish_cmd(out["vx"], out["wz"]) if self.cmd_vel_enabled else self._publish_cmd(0.0, 0.0)

        # 강제 취소 시 Nav2가 아직 active면 warning(리뷰 2차 #2)
        if out["state"] == "FAILED" and out["fail_reason"] == "nav_timeout" \
                and nav_status == "active":
            self.get_logger().warn("nav_cancel_wait 초과: Nav2 status가 여전히 active (짧은 경합 가능)")

        self._publish_state(out, errors, nav_status)

    def _publish_cmd(self, vx, wz):
        t = Twist()
        t.linear.x = float(vx)
        t.angular.z = float(wz)
        self._safe_publish(self.pub_cmd, t)

    def _publish_state(self, out, errors, nav_status):
        d = {"goal_id": self.goal_id, "state": out["state"],
             "nav_active": nav_status in ("pending", "active", "waiting_server") if nav_status else False,
             "nav_status": nav_status, "tag_detected": errors is not None,
             "error_x": round(errors["x"], 4) if errors else None,
             "error_y": round(errors["y"], 4) if errors else None,
             "error_yaw": round(errors["yaw"], 4) if errors else None,
             "vx": round(out["vx"], 4), "wz": round(out["wz"], 4),
             "cmd_vel_enabled": self.cmd_vel_enabled, "settled": out["settled"],
             "settle_count": out["settle_count"], "fail_reason": out["fail_reason"]}
        de = out["done_error"]
        d["done_error_x"] = round(de["x"], 4) if de else None
        d["done_error_y"] = round(de["y"], 4) if de else None
        d["done_error_yaw"] = round(de["yaw"], 4) if de else None
        self._safe_publish(self.pub_state, String(data=json.dumps(d)))

    def _detector_config_path(self):
        return os.path.join(get_package_share_directory("wasab_docking"),
                            "config", "precision_parking.yaml")

    def _spawn_detector(self):
        try:
            cmd = ["ros2", "run", "wasab_docking", "apriltag_detector",
                   "--ros-args", "--params-file", self._detector_config_path(),
                   "-p", f"tag_id:={self.tag_id}"]
            # start_new_session 미지정 → dock 프로세스그룹 유지(agent group-kill 백스톱)
            self._detector_proc = subprocess.Popen(cmd)
            self.get_logger().info(f"도킹 detector 기동(pid {self._detector_proc.pid})")
        except Exception as e:                                # noqa: BLE001
            self.get_logger().error(f"도킹 detector spawn 실패: {e!r}")
            self._detector_proc = None
            self._fatal_reason = "detector_spawn_failed"

    def _maybe_spawn_detector(self, state):
        if state == "SEARCH_TAG" and self._detector_proc is None and self._fatal_reason is None:
            self._spawn_detector()

    def _check_detector_alive(self):
        # SEARCH_TAG 중 tag 검출 전 detector가 죽으면 즉시 FAILED(카메라 busy 등)
        if self._detector_proc is not None and self._detector_proc.poll() is not None:
            self.get_logger().error("도킹 detector 조기 종료 → FAILED(detector_exited)")
            self._detector_proc = None
            self._fatal_reason = "detector_exited"

    def _kill_detector(self):
        proc = self._detector_proc
        self._detector_proc = None
        if proc is None or proc.poll() is not None:
            return
        try:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()                                   # 안 죽으면 강제(stop/shutdown 경로 orphan 방지, 리뷰 F3)
                proc.wait()               # SIGKILL한 자식 회수(zombie 방지)
        except Exception:                                     # noqa: BLE001
            pass

    def _publish_fatal(self):
        out = {"state": "FAILED", "vx": 0.0, "wz": 0.0, "settled": False,
               "nav_cmd": None, "fail_reason": self._fatal_reason,
               "settle_count": 0, "done_error": None}
        self._publish_cmd(0.0, 0.0)
        self._publish_state(out, None, None)

    def _safe_publish(self, pub, msg):
        # 종료 중 context invalid 방지(리뷰 #5, 필수 #6)
        if not rclpy.ok():
            return
        try:
            pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"publish 실패(무시): {e!r}")

    def stop(self):
        try:
            self.timer.cancel()
        except Exception:
            pass
        if self.nav is not None:
            try:
                self.nav.cancel()
            except Exception:
                pass
        self._kill_detector()
        self._publish_cmd(0.0, 0.0)               # zero 1회(rclpy.ok 확인은 _safe_publish 내부)


def main():
    rclpy.init()
    node = PrecisionParkingNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
