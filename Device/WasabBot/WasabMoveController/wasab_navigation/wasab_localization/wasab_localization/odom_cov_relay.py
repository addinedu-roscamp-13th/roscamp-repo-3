"""odom 공분산 릴레이 — /odom 구독, 공분산 주입, /odom_cov 재발행."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from wasab_localization import cov_math


class OdomCovRelay(Node):
    def __init__(self):
        super().__init__("odom_cov_relay")
        self.declare_parameter("input_topic", "odom")
        self.declare_parameter("output_topic", "odom_cov")
        self.declare_parameter(
            "twist_cov_diag", [4.0e-4, 1.0e6, 1.0e6, 1.0e6, 1.0e6, 2.5e-3])
        self.declare_parameter(
            "pose_cov_diag", [1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3])

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self._twist_cov = cov_math.covariance_matrix(
            list(self.get_parameter("twist_cov_diag").value))
        self._pose_cov = cov_math.covariance_matrix(
            list(self.get_parameter("pose_cov_diag").value))

        self._pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self._cb, 10)
        self.get_logger().info(f"odom_cov_relay: {in_topic} -> {out_topic}")

    def _cb(self, msg):
        msg.pose.covariance = self._pose_cov
        msg.twist.covariance = self._twist_cov
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = OdomCovRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
