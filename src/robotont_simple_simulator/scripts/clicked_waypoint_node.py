#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray


class ClickedWaypointNode(Node):
    def __init__(self):
        super().__init__("clicked_waypoint_node")

        self.clicked_point_topic = self.declare_parameter(
            "clicked_point_topic", "/clicked_point"
        ).value
        self.path_topic = self.declare_parameter("path_topic", "/waypoints/path").value
        self.marker_topic = self.declare_parameter(
            "marker_topic", "/waypoints/markers"
        ).value
        self.execute_topic = self.declare_parameter(
            "execute_topic", "/waypoints/execute"
        ).value
        self.clear_topic = self.declare_parameter("clear_topic", "/waypoints/clear").value
        self.default_frame = self.declare_parameter("default_frame", "map").value
        self.action_name = self.declare_parameter(
            "action_name", "navigate_through_poses"
        ).value

        self.waypoints = []
        self.last_marker_count = 0
        self.active_goal_handle = None
        self.action_client = ActionClient(self, NavigateThroughPoses, self.action_name)

        self.create_subscription(
            PointStamped, self.clicked_point_topic, self.clicked_point_callback, 10
        )
        self.create_subscription(Empty, self.execute_topic, self.execute_topic_callback, 10)
        self.create_subscription(Empty, self.clear_topic, self.clear_topic_callback, 10)

        visualization_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.path_pub = self.create_publisher(Path, self.path_topic, visualization_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, self.marker_topic, visualization_qos
        )

        self.get_logger().info(
            f"Collecting {self.clicked_point_topic} points as ordered Nav2 waypoints"
        )

    def clicked_point_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.default_frame
        if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
            pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = msg.point.x
        pose.pose.position.y = msg.point.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0

        self.waypoints.append(pose)
        waypoint_id = len(self.waypoints)
        self.get_logger().info(
            f"Added waypoint {waypoint_id}: "
            f"{pose.header.frame_id} x={pose.pose.position.x:.2f}, "
            f"y={pose.pose.position.y:.2f}"
        )
        self.publish_waypoints()

    def execute_topic_callback(self, _msg):
        self.start_navigation()

    def clear_topic_callback(self, _msg):
        self.clear_waypoints()

    def start_navigation(self):
        if not self.waypoints:
            message = "No queued waypoints to execute"
            self.get_logger().warn(message)
            return False, message

        if not self.action_client.wait_for_server(timeout_sec=1.0):
            message = f"Nav2 action server {self.action_name} is not available"
            self.get_logger().warn(message)
            return False, message

        goal = NavigateThroughPoses.Goal()
        goal.poses = list(self.waypoints)
        now = self.get_clock().now().to_msg()
        for pose in goal.poses:
            if not pose.header.frame_id:
                pose.header.frame_id = self.default_frame
            if pose.header.stamp.sec == 0 and pose.header.stamp.nanosec == 0:
                pose.header.stamp = now

        self.get_logger().info(f"Sending {len(goal.poses)} waypoint(s) to Nav2")
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        return True, f"Submitted {len(goal.poses)} waypoint(s) to Nav2"

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 rejected waypoint navigation goal")
            return

        self.active_goal_handle = goal_handle
        self.get_logger().info("Nav2 accepted waypoint navigation goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result()
        self.get_logger().info(
            f"Waypoint navigation finished with status {result.status}"
        )
        self.active_goal_handle = None

    def clear_waypoints(self):
        self.waypoints.clear()
        self.publish_waypoints()

    def publish_waypoints(self):
        now = self.get_clock().now().to_msg()
        frame_id = self.waypoints[0].header.frame_id if self.waypoints else self.default_frame

        path = Path()
        path.header.stamp = now
        path.header.frame_id = frame_id
        path.poses = list(self.waypoints)
        self.path_pub.publish(path)

        marker_array = MarkerArray()
        for marker_id in range(self.last_marker_count):
            marker_array.markers.append(self.delete_marker(marker_id))
            marker_array.markers.append(self.delete_marker(1000 + marker_id))

        for index, pose in enumerate(self.waypoints):
            marker_array.markers.append(self.point_marker(index, pose, now))
            marker_array.markers.append(self.text_marker(index, pose, now))

        self.last_marker_count = len(self.waypoints)
        self.marker_pub.publish(marker_array)

    def delete_marker(self, marker_id):
        marker = Marker()
        marker.header.frame_id = self.default_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "clicked_waypoints"
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def point_marker(self, index, pose, stamp):
        marker = Marker()
        marker.header = pose.header
        marker.header.stamp = stamp
        marker.ns = "clicked_waypoints"
        marker.id = index
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = 0.06
        marker.scale.x = 0.18
        marker.scale.y = 0.18
        marker.scale.z = 0.08
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.9
        return marker

    def text_marker(self, index, pose, stamp):
        marker = Marker()
        marker.header = pose.header
        marker.header.stamp = stamp
        marker.ns = "clicked_waypoints"
        marker.id = 1000 + index
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.pose.position.z = 0.28
        marker.scale.z = 0.18
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = str(index + 1)
        return marker


def main():
    rclpy.init()
    node = ClickedWaypointNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
