"""Pure pursuit path follower — converts Nav2 planned paths to cmd_vel.

Subscribes to the planned path from the Nav2 planner and robot odometry,
runs a pure pursuit algorithm to compute cmd_vel commands for the VESC driver.

Only active when the Foxglove hardware panel is in "nav2" mode.
"""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String


# Defaults — all tuneable via /nav/path_follower/config topic
_DEFAULT_LOOKAHEAD = 1.5  # meters
_DEFAULT_GOAL_TOLERANCE = 0.5  # meters — stop when this close to final waypoint
_DEFAULT_LINEAR_SPEED = 0.3  # m/s cruise speed
_DEFAULT_MAX_ANGULAR_VEL = 2.0  # rad/s
_DEFAULT_SLOWDOWN_RADIUS = 2.0  # meters — start slowing within this distance of goal


class PathFollower(Node):
    def __init__(self) -> None:
        super().__init__("path_follower")

        # Tuneable parameters
        self._lookahead = _DEFAULT_LOOKAHEAD
        self._goal_tolerance = _DEFAULT_GOAL_TOLERANCE
        self._linear_speed = _DEFAULT_LINEAR_SPEED
        self._max_angular_vel = _DEFAULT_MAX_ANGULAR_VEL
        self._slowdown_radius = _DEFAULT_SLOWDOWN_RADIUS

        # State
        self._path: Path | None = None
        self._pose_x = 0.0
        self._pose_y = 0.0
        self._yaw = 0.0
        self._has_pose = False
        self._nav2_mode = False
        self._e_stopped = False
        self._goal_reached = False

        qos = QoSProfile(depth=5)

        self.create_subscription(Path, "/nav/planned_path", self._on_path, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(String, "/teleop/status", self._on_teleop_status, qos)
        self.create_subscription(Bool, "/e_stop", self._on_e_stop, qos)
        self.create_subscription(String, "/nav/path_follower/config", self._on_config, qos)

        self._cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", qos)
        self._status_pub = self.create_publisher(String, "/nav/path_follower/status", qos)

        self.create_timer(0.1, self._control_loop)  # 10 Hz
        self.create_timer(1.0, self._publish_status)

        self.get_logger().info("Path follower started (pure pursuit)")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def _on_path(self, msg: Path) -> None:
        if len(msg.poses) < 2:
            self._path = None
            return
        self._path = msg
        self._goal_reached = False

    def _on_odom(self, msg: Odometry) -> None:
        self._pose_x = msg.pose.pose.position.x
        self._pose_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)
        self._has_pose = True

    def _on_teleop_status(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._nav2_mode = data.get("mode") == "nav2"
        except json.JSONDecodeError:
            pass

    def _on_e_stop(self, msg: Bool) -> None:
        self._e_stopped = msg.data

    def _on_config(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if "lookahead" in data:
            self._lookahead = max(0.3, float(data["lookahead"]))
        if "goal_tolerance" in data:
            self._goal_tolerance = max(0.1, float(data["goal_tolerance"]))
        if "linear_speed" in data:
            self._linear_speed = max(0.05, float(data["linear_speed"]))
        if "max_angular_vel" in data:
            self._max_angular_vel = max(0.1, float(data["max_angular_vel"]))
        if "slowdown_radius" in data:
            self._slowdown_radius = max(0.5, float(data["slowdown_radius"]))

    # ── Pure pursuit ──────────────────────────────────────────────────────

    def _control_loop(self) -> None:
        if (
            not self._nav2_mode
            or self._e_stopped
            or not self._has_pose
            or self._path is None
            or self._goal_reached
        ):
            return

        path = self._path
        poses = path.poses
        if len(poses) < 2:
            return

        # Distance to goal
        goal_x = poses[-1].pose.position.x
        goal_y = poses[-1].pose.position.y
        dist_to_goal = math.hypot(goal_x - self._pose_x, goal_y - self._pose_y)

        if dist_to_goal < self._goal_tolerance:
            self._cmd_vel_pub.publish(Twist())
            self._goal_reached = True
            self.get_logger().info("Goal reached")
            return

        # Find closest point index
        closest_idx = 0
        closest_dist = float("inf")
        for i, p in enumerate(poses):
            dx = p.pose.position.x - self._pose_x
            dy = p.pose.position.y - self._pose_y
            d = dx * dx + dy * dy
            if d < closest_dist:
                closest_dist = d
                closest_idx = i

        # Find lookahead point: first point >= lookahead_distance from us, ahead of closest
        lookahead_x, lookahead_y = goal_x, goal_y  # fallback: goal
        for i in range(closest_idx, len(poses)):
            px = poses[i].pose.position.x
            py = poses[i].pose.position.y
            if math.hypot(px - self._pose_x, py - self._pose_y) >= self._lookahead:
                lookahead_x, lookahead_y = px, py
                break

        # Transform lookahead to robot-local frame
        dx = lookahead_x - self._pose_x
        dy = lookahead_y - self._pose_y
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw

        L_sq = local_x * local_x + local_y * local_y
        if L_sq < 0.01:
            return

        twist = Twist()

        # Lookahead point is behind us — rotate in place toward it
        if local_x < 0:
            turn_dir = 1.0 if local_y >= 0 else -1.0
            twist.angular.z = turn_dir * min(self._max_angular_vel, self._max_angular_vel * 0.6)
            self._cmd_vel_pub.publish(twist)
            return

        # Pure pursuit curvature: kappa = 2 * y / L^2
        curvature = 2.0 * local_y / L_sq

        # Compute linear speed: cruise, reduced for sharp turns + near goal
        linear = self._linear_speed
        if abs(curvature) > 0.5:
            linear *= max(0.2, 1.0 - abs(curvature) * 0.3)
        if dist_to_goal < self._slowdown_radius:
            linear *= max(0.3, dist_to_goal / self._slowdown_radius)

        angular = linear * curvature
        angular = max(-self._max_angular_vel, min(self._max_angular_vel, angular))

        twist.linear.x = linear
        twist.angular.z = angular
        self._cmd_vel_pub.publish(twist)

    # ── Status ────────────────────────────────────────────────────────────

    def _publish_status(self) -> None:
        status: dict = {
            "active": self._nav2_mode and self._has_pose and self._path is not None and not self._goal_reached,
            "nav2_mode": self._nav2_mode,
            "has_path": self._path is not None,
            "has_pose": self._has_pose,
            "goal_reached": self._goal_reached,
            "e_stopped": self._e_stopped,
            "path_points": len(self._path.poses) if self._path else 0,
            "lookahead": self._lookahead,
            "linear_speed": self._linear_speed,
            "max_angular_vel": self._max_angular_vel,
        }
        if self._path and len(self._path.poses) > 0:
            gx = self._path.poses[-1].pose.position.x
            gy = self._path.poses[-1].pose.position.y
            status["dist_to_goal"] = round(
                math.hypot(gx - self._pose_x, gy - self._pose_y), 2
            )
        msg = String()
        msg.data = json.dumps(status)
        self._status_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
