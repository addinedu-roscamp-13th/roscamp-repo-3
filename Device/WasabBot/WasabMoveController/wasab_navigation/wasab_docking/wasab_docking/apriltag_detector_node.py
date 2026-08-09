#!/usr/bin/env python3
# wasab_docking/apriltag_detector_node.py
"""picamera2 프레임 → cv2.aruco(tag36h11) 검출 → solvePnP → /wasab/tag_pose.

Pinky 함대는 RPi CSI(ov5647)라 ROS image topic이 없어 picamera2로 직접 프레임을
읽는다(scripts/picamera_mjpeg_server.py와 동일 캡처 패턴). 검출한 tag의 광학 프레임
pose를 base_footprint 기준으로 변환해 PoseStamped로 발행한다.

디버그: annotated 프레임을 UDP(JPEG)로 PC에 직접 송신한다(udp_host:udp_port).
picamera2는 단독 점유라 별도 스트림 노드를 못 띄우므로 detector가 직접 보낸다.
"""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import cv2
import socket
from picamera2 import Picamera2

from wasab_docking import geometry as g
from wasab_docking import relocalize_logic as rl


class AprilTagDetectorNode(Node):
    def __init__(self):
        super().__init__("apriltag_detector")
        p = self.declare_parameter
        self.tag_id = p("tag_id", 7).value
        self.tag_ids = [int(x) for x in p("tag_ids", [7, 8, 9, 10]).value]
        self.tag_size = p("tag_size_m", 0.06).value
        self.base_frame = p("base_frame", "base_footprint").value
        self.rate_hz = p("publish_rate_hz", 15.0).value
        # 캡처 해상도는 calibration 해상도와 반드시 일치해야 한다(intrinsic은 해상도 종속).
        self.width = p("width", 640).value
        self.height = p("height", 480).value
        # 카메라가 base에 상하 뒤집혀 실장된 경우 flip_vertical+flip_horizontal=180° 회전으로
        # 이미지를 똑바로 세운다. 상하 미러(flip_vertical 단독)는 tag 패턴을 깨뜨리므로 금지.
        self.flip_vertical = p("flip_vertical", True).value
        self.flip_horizontal = p("flip_horizontal", False).value
        self.swap_rb = p("swap_rb", True).value
        self.ext = {
            "x": p("camera_to_base.x", 0.08).value,
            "y": p("camera_to_base.y", 0.0).value,
            "z": p("camera_to_base.z", 0.10).value,
            "yaw": p("camera_to_base.yaw", 0.0).value,
        }
        fx = p("camera.fx", 0.0).value; fy = p("camera.fy", 0.0).value
        cx = p("camera.cx", 0.0).value; cy = p("camera.cy", 0.0).value
        if not (fx > 0.0 and fy > 0.0 and cx > 0.0 and cy > 0.0):
            # intrinsic 미보정 상태로 solvePnP를 돌리면 무의미한 pose가 나온다 → fatal
            raise RuntimeError(
                "camera intrinsic(fx/fy/cx/cy)이 0입니다. calibration 후 config.camera.*를 채우세요."
            )
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
        self.dist = np.array(p("camera.dist_coeffs", [0.0]*5).value, dtype=np.float64)

        s = self.tag_size / 2.0
        self.obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float64)
        adict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        self.detector = cv2.aruco.ArucoDetector(adict, cv2.aruco.DetectorParameters())

        self.cam = Picamera2()
        # size를 명시해 해상도를 고정(calibration 해상도와 일치). 기본 preview 해상도에
        # 암묵 의존하면 intrinsic 불일치로 pose가 계통적으로 틀어질 수 있다.
        self.cam.configure(self.cam.create_preview_configuration(
            main={"format": "RGB888", "size": (self.width, self.height)}))
        self.cam.start()

        # UDP 디버그 스트림 (annotated JPEG) → PC. 반드시 UDP.
        self.udp_host = p("udp_host", "192.168.2.5").value
        self.udp_port = int(p("udp_port", 5005).value)
        self.udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.pub_pose = self.create_publisher(PoseStamped, "/wasab/tag_pose", 10)
        self.pub_det = self.create_publisher(Bool, "/wasab/tag_detected", 10)
        from std_msgs.msg import String as _String
        self.pub_obs = self.create_publisher(_String, "/wasab/tag_observation", 10)
        # 도킹 상태 구독 — NAV_TO_APPROACH(Nav2 주행) 구간엔 UDP 스트리밍을 꺼서
        # detector 인코딩 부하가 Nav2 planner/controller와 경합하지 않게 한다(검출은 계속).
        self._dock_state = None
        self.create_subscription(_String, "/wasab/docking_state", self._on_dock_state, 10)
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self.get_logger().info(
            f"apriltag_detector: tag_id={self.tag_id} size={self.tag_size} "
            f"udp={self.udp_host}:{self.udp_port}")

    def _tick(self):
        arr = self.cam.capture_array()
        # NAV 구간(Nav2 주행)엔 검출·인코딩 전부 생략해 Nav2 planner/controller에 CPU를 양보한다.
        # 그 구간엔 태그가 아직 안 보여 검출이 불필요. 프레임만 버리고 즉시 반환.
        if self._dock_state in ("NAV_TO_APPROACH", "NAV_CANCELING"):
            self.pub_det.publish(Bool(data=False))
            return
        if self.flip_vertical:
            arr = arr[::-1, :, :]
        if self.flip_horizontal:
            arr = arr[:, ::-1, :]
        if self.swap_rb:
            arr = arr[:, :, ::-1]
        gray = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        found = False
        tags = []
        target_x = target_y = target_yaw = None
        if ids is not None:
            for i, tid in enumerate(ids.flatten()):
                tid = int(tid)
                if tid not in self.tag_ids and tid != int(self.tag_id):
                    continue
                ok, rvec, tvec = cv2.solvePnP(
                    self.obj, corners[i][0].astype(np.float64), self.K, self.dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    continue
                R, _ = cv2.Rodrigues(rvec)
                cam_xyz = (float(tvec[0]), float(tvec[1]), float(tvec[2]))
                # 카메라 수직축(광학 Y) 기준 tag 회전 → 순수 함수(단위 테스트됨)
                cam_yaw = g.yaw_about_vertical_from_R(R)
                bx, by, byaw = g.camera_optical_to_base(cam_xyz, cam_yaw, self.ext)
                if tid == int(self.tag_id) and not found:   # legacy first-wins 복원(중복 publish 방지)
                    self._publish(bx, by, byaw)
                    found = True
                    target_x, target_y, target_yaw = bx, by, byaw
                if tid in self.tag_ids:                     # 후보 수집(신규)
                    area = float(cv2.contourArea(corners[i][0].astype(np.float32)))
                    tags.append({"tag_id": tid, "x": bx, "y": by, "yaw": byaw, "area": area})
        self._send_udp(arr, corners, ids, target_x, target_y, target_yaw)   # NAV은 위에서 이미 반환됨
        self.pub_det.publish(Bool(data=found))
        now = self.get_clock().now().to_msg()
        from std_msgs.msg import String as _String
        self.pub_obs.publish(_String(data=rl.serialize_observation(tags, now.sec, now.nanosec)))

    def _on_dock_state(self, msg):
        """docking_state 의 state 추적. NAV 구간 UDP 스트리밍 게이트용."""
        try:
            self._dock_state = json.loads(msg.data).get("state")
        except Exception:
            pass

    def _send_udp(self, frame, corners, ids, tx, ty, tyaw):
        """annotated 컬러 프레임(마커·중앙십자·거리)을 JPEG로 UDP 송신. 단일 datagram.

        frame = 캡처 프레임. picamera2 "RGB888"은 실제로 BGR 배열을 준다(알려진 함정,
        swap_rb=false). imencode는 BGR 기대 → 변환 없이 그대로 보내야 색이 맞는다
        (RGB2BGR를 넣으면 콘솔 bgr_to_qimage와 겹쳐 R↔B 뒤바뀜=파랑→주황). 회색 아님.
        """
        try:
            dbg = np.ascontiguousarray(frame)
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(dbg, corners, ids)
            hh, ww = dbg.shape[:2]
            cv2.line(dbg, (ww // 2, 0), (ww // 2, hh), (0, 255, 0), 1)
            cv2.line(dbg, (0, hh // 2), (ww, hh // 2), (0, 255, 0), 1)
            if tx is not None:
                txt = f"x={tx*100:.1f}cm y={ty*100:+.1f}cm yaw={tyaw*57.2958:+.1f}deg"
                cv2.putText(dbg, txt, (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            else:
                cv2.putText(dbg, "NO TAG", (8, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            ok3, enc = cv2.imencode(".jpg", dbg, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ok3 and len(enc) < 65000:
                self.udp_sock.sendto(enc.tobytes(), (self.udp_host, self.udp_port))
        except Exception:
            pass

    def _publish(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.base_frame
        msg.pose.position.x = x
        msg.pose.position.y = y
        qx, qy, qz, qw = g.quat_from_yaw(yaw)
        msg.pose.orientation.x = qx; msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz; msg.pose.orientation.w = qw
        self.pub_pose.publish(msg)

    def destroy_node(self):
        # 카메라 busy 잔존 방지: timer 취소 + picamera2 정지/해제
        try:
            self.timer.cancel()
        except Exception:
            pass
        try:
            self.cam.stop()
            self.cam.close()
        except Exception as e:
            self.get_logger().warn(f"camera cleanup 실패: {e!r}")
        super().destroy_node()


def main():
    rclpy.init()
    node = AprilTagDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
