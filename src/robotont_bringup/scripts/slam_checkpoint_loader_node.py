#!/usr/bin/env python3

import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from slam_toolbox.srv import DeserializePoseGraph


class SlamCheckpointLoaderNode(Node):
    def __init__(self):
        super().__init__("slam_checkpoint_loader")

        checkpoint = self.declare_parameter("checkpoint", "").value
        service_name = self.declare_parameter(
            "service_name", "/slam_toolbox/deserialize_map"
        ).value
        wait_timeout = float(self.declare_parameter("wait_timeout", 60.0).value)
        odom_topic = self.declare_parameter("odom_topic", "/odom").value
        odom_wait_timeout = float(self.declare_parameter("odom_wait_timeout", 5.0).value)
        fallback_x = float(self.declare_parameter("initial_x", 0.0).value)
        fallback_y = float(self.declare_parameter("initial_y", 0.0).value)
        fallback_theta = float(self.declare_parameter("initial_theta", 0.0).value)
        match_type = int(
            self.declare_parameter(
                "match_type", DeserializePoseGraph.Request.START_AT_GIVEN_POSE
            ).value
        )

        self.checkpoint = self._checkpoint_base(checkpoint)
        self.match_type = match_type
        self.client = self.create_client(DeserializePoseGraph, service_name)
        initial_pose = self._current_odom_pose(
            odom_topic, odom_wait_timeout, fallback_x, fallback_y, fallback_theta
        )

        self.get_logger().info(
            f"Loading slam_toolbox checkpoint {self.checkpoint} via {service_name}"
        )
        if not self.client.wait_for_service(timeout_sec=wait_timeout):
            raise RuntimeError(
                f"Timed out waiting {wait_timeout:.1f}s for {service_name}"
            )

        request = DeserializePoseGraph.Request()
        request.filename = self.checkpoint
        request.match_type = self.match_type
        request.initial_pose.x = initial_pose[0]
        request.initial_pose.y = initial_pose[1]
        request.initial_pose.theta = initial_pose[2]
        self.get_logger().info(
            "Deserializing checkpoint with initial map->base pose "
            f"x={initial_pose[0]:.3f}, y={initial_pose[1]:.3f}, "
            f"theta={initial_pose[2]:.3f}, match_type={self.match_type}"
        )

        future = self.client.call_async(request)
        future.add_done_callback(self._load_done)

    def _current_odom_pose(self, odom_topic, wait_timeout, fallback_x, fallback_y, fallback_theta):
        pose = None

        def odom_callback(msg):
            nonlocal pose
            orientation = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
            )
            pose = (
                float(msg.pose.pose.position.x),
                float(msg.pose.pose.position.y),
                float(yaw),
            )

        subscription = self.create_subscription(Odometry, odom_topic, odom_callback, 10)
        deadline = self.get_clock().now().nanoseconds / 1e9 + wait_timeout
        while rclpy.ok() and pose is None:
            if self.get_clock().now().nanoseconds / 1e9 >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        self.destroy_subscription(subscription)

        if pose is not None:
            self.get_logger().info(
                f"Using current odom pose from {odom_topic} as checkpoint initial pose"
            )
            return pose

        self.get_logger().warn(
            f"No odom received from {odom_topic} within {wait_timeout:.1f}s; "
            "using configured fallback initial pose"
        )
        return (fallback_x, fallback_y, fallback_theta)

    def _checkpoint_base(self, raw):
        if not raw:
            raise RuntimeError("checkpoint parameter is required")

        path = Path(raw)
        if not path.is_absolute():
            path = Path("/ws/saved_maps") / path
        if path.is_dir():
            path = path / path.name
        elif not path.suffix:
            bundle_path = path / path.name
            if bundle_path.with_suffix(".posegraph").is_file() or bundle_path.with_suffix(".data").is_file():
                path = bundle_path
        if path.suffix in (".posegraph", ".data"):
            path = path.with_suffix("")

        missing = [
            str(path.with_suffix(suffix))
            for suffix in (".posegraph", ".data")
            if not path.with_suffix(suffix).is_file()
        ]
        if missing:
            raise RuntimeError(
                "Cannot resume SLAM from an occupancy map alone. "
                "Expected slam_toolbox checkpoint files: " + ", ".join(missing)
            )

        return str(path)

    def _load_done(self, future):
        try:
            future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to load SLAM checkpoint: {exc}")
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"SLAM checkpoint loaded from {self.checkpoint}; mapping can continue"
        )


def main():
    rclpy.init()
    node = SlamCheckpointLoaderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
