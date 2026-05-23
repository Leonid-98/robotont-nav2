#!/usr/bin/env python3

import math
import json
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


def make_box(min_x, min_y, max_x, max_y):
    return [
        (min_x, min_y, max_x, min_y),
        (max_x, min_y, max_x, max_y),
        (max_x, max_y, min_x, max_y),
        (min_x, max_y, min_x, min_y),
    ]


def cross(ax, ay, bx, by):
    return ax * by - ay * bx


class FakeLaserNode(Node):
    def __init__(self):
        super().__init__("fake_laser_node")

        self.frame_id = self.declare_parameter("frame_id", "base_scan").value
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.scan_topic = self.declare_parameter("scan_topic", "/scan").value
        self.range_min = self.declare_parameter("range_min", 0.08).value
        self.range_max = self.declare_parameter("range_max", 6.0).value
        self.angle_min = self.declare_parameter("angle_min", -math.pi).value
        self.angle_max = self.declare_parameter("angle_max", math.pi).value
        self.angle_increment = self.declare_parameter("angle_increment", math.pi / 180.0).value
        self.scan_rate = self.declare_parameter("scan_rate", 10.0).value
        self.laser_x = self.declare_parameter("laser_x", 0.08).value
        self.laser_y = self.declare_parameter("laser_y", 0.0).value
        self.world_file = self.declare_parameter("world_file", "").value

        self.world_segments = []
        self.load_world()

        self.have_pose = False
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.scan_pub = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.create_timer(1.0 / max(self.scan_rate, 1.0), self.publish_scan)

        self.get_logger().info(
            f"Publishing fake laser {self.scan_topic} in frame {self.frame_id}"
        )

    def load_world(self):
        if self.world_file:
            try:
                self.load_json_world(Path(self.world_file))
                self.get_logger().info(
                    f"Loaded fake laser world from {self.world_file} with {len(self.world_segments)} segments"
                )
                return
            except Exception as exc:
                self.get_logger().warn(
                    f"Could not load world_file '{self.world_file}': {exc}. Using built-in demo world."
                )

        self.add_box(-3.0, -2.0, 3.0, 2.0)
        self.add_box(-1.2, -0.8, -0.8, 0.8)
        self.add_box(0.8, -1.3, 1.2, -0.2)
        self.add_box(1.6, 0.6, 2.2, 1.1)

    def load_json_world(self, path):
        with path.open("r", encoding="utf-8") as world_stream:
            world = json.load(world_stream)

        segments = []
        for wall in world.get("walls", []):
            segments.append((
                float(wall["x1"]),
                float(wall["y1"]),
                float(wall["x2"]),
                float(wall["y2"]),
            ))

        for box in world.get("boxes", []):
            min_x = float(box["min_x"])
            min_y = float(box["min_y"])
            max_x = float(box["max_x"])
            max_y = float(box["max_y"])
            segments.extend(make_box(
                min(min_x, max_x),
                min(min_y, max_y),
                max(min_x, max_x),
                max(min_y, max_y),
            ))

        if not segments:
            raise ValueError("world must contain at least one wall or box")

        self.world_segments = segments

    def add_box(self, min_x, min_y, max_x, max_y):
        self.world_segments.extend(make_box(min_x, min_y, max_x, max_y))

    def odom_callback(self, msg):
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw
        self.have_pose = True

    def cast_ray(self, ray_angle):
        origin_x = self.robot_x + math.cos(self.robot_yaw) * self.laser_x - math.sin(self.robot_yaw) * self.laser_y
        origin_y = self.robot_y + math.sin(self.robot_yaw) * self.laser_x + math.cos(self.robot_yaw) * self.laser_y
        ray_dx = math.cos(ray_angle)
        ray_dy = math.sin(ray_angle)
        nearest = self.range_max

        for x1, y1, x2, y2 in self.world_segments:
            seg_dx = x2 - x1
            seg_dy = y2 - y1
            qpx = x1 - origin_x
            qpy = y1 - origin_y
            denom = cross(ray_dx, ray_dy, seg_dx, seg_dy)

            if abs(denom) < 1e-9:
                continue

            distance = cross(qpx, qpy, seg_dx, seg_dy) / denom
            segment_ratio = cross(qpx, qpy, ray_dx, ray_dy) / denom

            if (
                self.range_min <= distance <= nearest
                and 0.0 <= segment_ratio <= 1.0
            ):
                nearest = distance

        return nearest

    def publish_scan(self):
        if not self.have_pose:
            return

        scan = LaserScan()
        scan.header.stamp = self.get_clock().now().to_msg()
        scan.header.frame_id = self.frame_id
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_max
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 1.0 / max(self.scan_rate, 1.0)
        scan.range_min = self.range_min
        scan.range_max = self.range_max

        sample_count = int(math.floor((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        scan.ranges = [
            float(self.cast_ray(self.robot_yaw + self.angle_min + index * self.angle_increment))
            for index in range(sample_count)
        ]
        self.scan_pub.publish(scan)


def main():
    rclpy.init()
    node = FakeLaserNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
