#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfNode(Node):
    def __init__(self):
        super().__init__("odom_tf_node")
        self.odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self.parent_frame = self.declare_parameter("parent_frame", "odom").value
        self.child_frame = self.declare_parameter("child_frame", "base_footprint").value
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 50)
        self.get_logger().info(
            f"Broadcasting TF from {self.odom_topic}: {self.parent_frame} -> {self.child_frame}"
        )

    def odom_callback(self, msg):
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = msg.header.frame_id or self.parent_frame
        transform.child_frame_id = msg.child_frame_id or self.child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = OdomTfNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
