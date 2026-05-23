#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class GoalBridgeNode(Node):
    def __init__(self):
        super().__init__("goal_bridge_node")

        self.goal_topic = self.declare_parameter("goal_topic", "/goal_pose").value
        self.default_frame = self.declare_parameter("default_frame", "map").value
        self.action_name = self.declare_parameter("action_name", "navigate_to_pose").value

        self.action_client = ActionClient(self, NavigateToPose, self.action_name)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        self.get_logger().info(
            f"Forwarding {self.goal_topic} PoseStamped messages to {self.action_name}"
        )

    def goal_callback(self, msg):
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn(
                f"Nav2 action server {self.action_name} is not available yet; dropping goal"
            )
            return

        goal = NavigateToPose.Goal()
        goal.pose = msg

        if not goal.pose.header.frame_id:
            goal.pose.header.frame_id = self.default_frame
        if goal.pose.header.stamp.sec == 0 and goal.pose.header.stamp.nanosec == 0:
            goal.pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            "Forwarding goal in "
            f"{goal.pose.header.frame_id}: x={goal.pose.pose.position.x:.2f}, "
            f"y={goal.pose.pose.position.y:.2f}"
        )
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected the forwarded goal")
            return

        self.get_logger().info("Nav2 accepted the forwarded goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(f"Nav2 goal finished with status {result.status}")


def main():
    rclpy.init()
    node = GoalBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
