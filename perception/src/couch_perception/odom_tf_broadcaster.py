"""Broadcast odom → base_link TF from raw ARKit odometry.

Subscribes to /iphone/odom (or configurable topic) and publishes the
transform to TF2. This replaces the broken EKF with direct ARKit pose.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    """Broadcast TF from odometry messages."""

    def __init__(self) -> None:
        super().__init__("odom_tf_broadcaster")

        self.declare_parameter("odom_topic", "/iphone/odom")
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")

        odom_topic = self.get_parameter("odom_topic").value
        self._parent_frame = self.get_parameter("parent_frame").value
        self._child_frame = self.get_parameter("child_frame").value

        self._tf_broadcaster = TransformBroadcaster(self)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._odom_sub = self.create_subscription(
            Odometry, odom_topic, self._on_odom, qos
        )

        self.get_logger().info(
            f"Broadcasting TF {self._parent_frame} → {self._child_frame} from {odom_topic}"
        )

    def _on_odom(self, msg: Odometry) -> None:
        """Convert odometry to TF and broadcast."""
        tf = TransformStamped()
        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = self._parent_frame
        tf.child_frame_id = self._child_frame

        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = msg.pose.pose.position.z
        tf.transform.rotation = msg.pose.pose.orientation

        self._tf_broadcaster.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
