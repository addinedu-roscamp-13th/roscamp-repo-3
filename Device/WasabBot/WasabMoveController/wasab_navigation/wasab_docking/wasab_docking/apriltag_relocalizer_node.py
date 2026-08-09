"""AprilTag 글로벌 재측위 노드.

/wasab/tag_observation(후보) + /odom(정지·주행) → 정지-게이트 원샷으로 /initialpose 발행.
/wasab/relocalize_cmd "calibrate:<id>"|"rearm" 처리. 발행/파일쓰기는 이 노드 스레드에서만.
"""
import math
import os

import rclpy
from rclpy.node import Node
import tf2_ros
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

from wasab_docking import geometry as g
from wasab_docking import tag_map_poses as tmp
from wasab_docking import relocalize_logic as rl


class Relocalizer(Node):
    def __init__(self):
        super().__init__("apriltag_relocalizer")
        p = self.declare_parameter
        self.cfg = {
            "v_thresh": p("v_thresh", 0.01).value,
            "w_thresh": p("w_thresh", 0.03).value,
            "stationary_frames": int(p("stationary_frames", 5).value),
            "min_rearm_travel_m": p("min_rearm_travel_m", 0.3).value,
            "cooldown_s": p("cooldown_s", 5.0).value,
            "auto_rearm_enabled": p("auto_rearm_enabled", True).value,
        }
        self.max_tag_age_s = p("max_tag_age_s", 0.2).value
        self.max_odom_age_s = p("max_odom_age_s", 0.5).value
        self.cov_xy = p("cov_xy", 0.0025).value
        self.cov_yaw = p("cov_yaw", 0.0012).value
        self.map_name = p("map_name", "wasab_map5").value
        self.auto_enabled = p("auto_relocalize_enabled", True).value
        self.map_frame = p("map_frame", "map").value
        self.base_frame = p("base_frame", "base_footprint").value
        # 저장/로드 = writable HOME 경로(install/share는 재빌드 때 덮이고 권한 문제 → 금지).
        default_cfg = os.path.expanduser("~/.wasab/tag_map_poses.yaml")
        self.config_path = p("tag_map_poses_path", default_cfg).value

        self.state = rl.RelocalizeState(self.cfg)
        self.tagcfg = tmp.load(self.config_path) if self.config_path and os.path.exists(self.config_path) else {"map": self.map_name, "tags": {}}
        self.tagcfg.setdefault("map", self.map_name)   # 로드 config에 map 없으면 현재 맵으로
        self._last_obs = None       # (stamp_sec, tags)
        self._last_odom_t = None
        self._prev_xy = None

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(String, "/wasab/tag_observation", self._on_obs, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(String, "/wasab/relocalize_cmd", self._on_cmd, 10)
        self.get_logger().info(f"relocalizer: map={self.map_name} auto={self.auto_enabled} config={self.config_path}")

    def _now(self):
        t = self.get_clock().now().to_msg()
        return t.sec + t.nanosec * 1e-9

    def _on_obs(self, msg):
        try:
            self._last_obs = rl.parse_observation(msg.data)
        except Exception as e:
            self.get_logger().warn(f"obs parse 실패: {e}")
            return
        if not self.auto_enabled:
            return
        if self.map_name != self.tagcfg.get("map"):
            self.get_logger().warn(                       # 현장 디버깅용(throttle)
                f"map 불일치: node map_name={self.map_name} != config map={self.tagcfg.get('map')} → 재측위 skip",
                throttle_duration_sec=10.0)
            return
        self._try_auto()

    def _on_odom(self, msg):
        now = self._now()
        self._last_odom_t = now
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        dd = 0.0 if self._prev_xy is None else math.hypot(x - self._prev_xy[0], y - self._prev_xy[1])
        self._prev_xy = (x, y)
        self.state.on_odom(v, w, dd)

    def _try_auto(self):
        now = self._now()
        if self._last_obs is None or self._last_odom_t is None:
            return
        stamp, tags = self._last_obs
        if not rl.is_fresh(stamp, now, self.max_tag_age_s):
            return
        if not rl.is_fresh(self._last_odom_t, now, self.max_odom_age_s):
            return
        sel = rl.select_registered_max_area(tags, tmp.registered_ids(self.tagcfg))
        if sel is None:
            return
        tag_id, tag_in_base = sel
        if not self.state.try_relocalize(tag_id, now):
            return
        tag_in_map = tmp.get(self.tagcfg, tag_id)
        base_in_map = g.se2_compose(tag_in_map, g.se2_inverse(tag_in_base))
        self._publish_initialpose(base_in_map)
        self.get_logger().info(f"재측위 tag={tag_id} pose={base_in_map}")

    def _on_cmd(self, msg):
        kind, tag_id = rl.parse_cmd(msg.data)
        if kind == "rearm":
            self.state.on_rearm_cmd()
            self.get_logger().info("수동 재무장")
        elif kind == "calibrate":
            self._calibrate(tag_id)

    def _calibrate(self, tag_id):
        if self.map_name != self.tagcfg.get("map"):
            self.get_logger().warn(
                f"calibrate: map 불일치 {self.map_name} != {self.tagcfg.get('map')} → skip",
                throttle_duration_sec=10.0)
            return
        if self._last_obs is None:
            self.get_logger().warn("calibrate: observation 없음")
            return
        stamp, tags = self._last_obs
        if not rl.is_fresh(stamp, self._now(), self.max_tag_age_s):
            self.get_logger().warn("calibrate: observation stale")
            return
        tag_in_base = rl.find_tag(tags, tag_id)
        if tag_in_base is None:
            self.get_logger().warn(f"calibrate: tag {tag_id} 미탐지")
            return
        try:
            tr = self._tf_buf.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception as e:
            self.get_logger().warn(f"calibrate: TF 없음 {e}")
            return
        t = tr.transform.translation
        yaw = g.yaw_from_quat(tr.transform.rotation.x, tr.transform.rotation.y,
                              tr.transform.rotation.z, tr.transform.rotation.w)
        base_in_map = (t.x, t.y, yaw)
        tag_in_map = g.se2_compose(base_in_map, tag_in_base)
        tmp.upsert(self.tagcfg, tag_id, tag_in_map)
        if self.config_path:
            os.makedirs(os.path.dirname(self.config_path), exist_ok=True)  # writable HOME 보장
            tmp.save(self.tagcfg, self.config_path)
        self.get_logger().info(f"태그 등록 tag={tag_id} map_pose={tag_in_map} → {self.config_path}")

    def _publish_initialpose(self, pose):
        x, y, yaw = pose
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.map_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.pose.pose.position.x = x
        m.pose.pose.position.y = y
        _, _, qz, qw = g.quat_from_yaw(yaw)
        m.pose.pose.orientation.z = qz
        m.pose.pose.orientation.w = qw
        cov = [0.0] * 36
        cov[0] = self.cov_xy
        cov[7] = self.cov_xy
        cov[35] = self.cov_yaw
        m.pose.covariance = cov
        self.pub_init.publish(m)


def main():
    rclpy.init()
    node = Relocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
