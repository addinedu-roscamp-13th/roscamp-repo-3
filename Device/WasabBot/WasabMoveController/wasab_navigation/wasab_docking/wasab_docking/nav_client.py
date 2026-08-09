#!/usr/bin/env python3
# wasab_docking/nav_client.py
"""NavigateToPose ActionClient 얇은 래핑(비차단). 상태머신엔 status 문자열만 노출.

노드가 매 제어틱 .status를 폴링한다. 무한 대기 없음(server_wait_s로 판정).
"""
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

from wasab_docking import geometry as g


class NavClientAdapter:
    def __init__(self, node, action_name="/navigate_to_pose", server_wait_s=5.0):
        self._node = node
        self._client = ActionClient(node, NavigateToPose, action_name)
        self._server_wait_s = server_wait_s
        self._status = "inactive"
        self._send_time = None
        self._goal_handle = None
        self._deferred_pose = None

    @property
    def status(self):
        # 서버 대기 판정: 아직 accept 전이고 서버 미가용이면 waiting/unavailable
        if self._status == "pending" and self._send_time is not None:
            if self._deferred_pose is not None:
                if self._client.server_is_ready():
                    self._flush_deferred()
                else:
                    elapsed = (self._node.get_clock().now().nanoseconds * 1e-9) - self._send_time
                    return "server_unavailable" if elapsed > self._server_wait_s else "waiting_server"
            elif not self._client.server_is_ready():
                elapsed = (self._node.get_clock().now().nanoseconds * 1e-9) - self._send_time
                return "server_unavailable" if elapsed > self._server_wait_s else "waiting_server"
        return self._status

    def send_goal(self, pose):
        # 서버 미매칭 상태로 보내면 DDS가 요청을 버려 영원히 pending에 갇힌다.
        # → ready 전이면 보관하고, status 폴링 시점에 전송한다.
        self._status = "pending"
        self._send_time = self._node.get_clock().now().nanoseconds * 1e-9
        self._deferred_pose = pose
        if self._client.server_is_ready():
            self._flush_deferred()

    def _flush_deferred(self):
        pose = self._deferred_pose
        self._deferred_pose = None
        x, y, yaw = pose
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = self._node.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        qx, qy, qz, qw = g.quat_from_yaw(float(yaw))
        ps.pose.orientation.x = qx; ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz; ps.pose.orientation.w = qw
        goal = NavigateToPose.Goal()
        goal.pose = ps
        fut = self._client.send_goal_async(goal)
        fut.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self._status = "rejected"
            return
        self._goal_handle = gh
        self._status = "active"
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        from action_msgs.msg import GoalStatus
        status = fut.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._status = "succeeded"
        elif status == GoalStatus.STATUS_CANCELED:
            self._status = "canceled"
        else:
            self._status = "failed"

    def cancel(self):
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
