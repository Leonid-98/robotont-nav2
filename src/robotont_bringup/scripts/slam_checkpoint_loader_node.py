#!/usr/bin/env python3

from pathlib import Path

import rclpy
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
        match_type = int(
            self.declare_parameter(
                "match_type", DeserializePoseGraph.Request.START_AT_FIRST_NODE
            ).value
        )

        self.checkpoint = self._checkpoint_base(checkpoint)
        self.match_type = match_type
        self.client = self.create_client(DeserializePoseGraph, service_name)

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
        request.initial_pose.x = 0.0
        request.initial_pose.y = 0.0
        request.initial_pose.theta = 0.0

        future = self.client.call_async(request)
        future.add_done_callback(self._load_done)

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
