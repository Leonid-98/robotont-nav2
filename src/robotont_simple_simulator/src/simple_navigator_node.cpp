#include <algorithm>
#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>

using namespace std::chrono_literals;

class RobotontNavigator : public rclcpp::Node {
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    RobotontNavigator()
        : Node("simple_navigator_node") {

        // Declare and get parameters
        this->declare_parameter("linear_speed", 0.2);
        this->declare_parameter("angular_speed", 0.5);
        linear_speed_ = this->get_parameter("linear_speed").as_double();
        angular_speed_ = this->get_parameter("angular_speed").as_double();

        RCLCPP_INFO(this->get_logger(), "Navigator initialized with linear_speed=%.2f, angular_speed=%.2f",
                    linear_speed_, angular_speed_);

        // tf2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Publisher for velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RobotontNavigator::odom_callback, this, std::placeholders::_1));

        // Action server for navigation
        action_server_ = rclcpp_action::create_server<NavigateToPose>(
            this,
            "navigate_to_pose",
            std::bind(&RobotontNavigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotontNavigator::handle_cancel, this, std::placeholders::_1),
            std::bind(&RobotontNavigator::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Robotont Navigator Action Server Initialized");
    }
    

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    double goal_x_, goal_y_, goal_yaw_; // Goal position
    double linear_speed_, angular_speed_; // Speed limits
    bool goal_active_ = false;

    

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!goal_active_) return;

        double robot_x = msg->pose.pose.position.x;
        double robot_y = msg->pose.pose.position.y;
        double robot_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

        // Distance to goal position
        double dx = goal_x_ - robot_x;
        double dy = goal_y_ - robot_y;
        double distance_to_goal = std::sqrt(dx * dx + dy * dy);
        
        // Angle errors
        double angle_to_goal = std::atan2(dy, dx);
        double heading_error = normalize_angle(angle_to_goal - robot_yaw);
        double orientation_error = normalize_angle(goal_yaw_ - robot_yaw);  // NEW!

        constexpr double pos_threshold = 0.05;   // meters
        constexpr double yaw_threshold = 0.1;    // radians

        // Check if reached position AND orientation
        bool position_reached = distance_to_goal < pos_threshold;
        bool orientation_reached = std::fabs(orientation_error) < yaw_threshold;

        if (position_reached && orientation_reached) {
            RCLCPP_INFO(this->get_logger(), "Goal reached with correct orientation!");
            stop_robot();
            goal_active_ = false;
            if (current_goal_handle_) {
                auto result = std::make_shared<NavigateToPose::Result>();
                current_goal_handle_->succeed(result);
            }
            return;
        }

        double linear_velocity = 0.0;
        double angular_velocity = 0.0;

        if (!position_reached) {
            // Still need to reach position - navigate as before
            if (std::fabs(heading_error) > yaw_threshold) {
                angular_velocity = std::clamp(heading_error, -angular_speed_, angular_speed_);
            } else {
                linear_velocity = std::min(linear_speed_, distance_to_goal);
                angular_velocity = std::clamp(heading_error, -angular_speed_, angular_speed_);
            }
        } else {
            // Position reached, now orient to goal
            linear_velocity = 0.0;
            angular_velocity = std::clamp(orientation_error, -angular_speed_, angular_speed_);
        }

        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = linear_velocity;
        cmd_vel.angular.z = angular_velocity;
        cmd_vel_pub_->publish(cmd_vel);
    }

    void stop_robot() {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }

    double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
        // Convert quaternion to yaw (rotation about Z axis)
        return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    }

    double normalize_angle(double angle) {
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const NavigateToPose::Goal> goal) {
        const auto& quat = goal->pose.pose.orientation;
        double goal_yaw = get_yaw_from_quaternion(quat);
        RCLCPP_INFO(this->get_logger(), 
                    "Received goal request: x=%.2f, y=%.2f, quat(x=%.3f, y=%.3f, z=%.3f, w=%.3f), yaw=%.2f rad (%.1f deg)",
                    goal->pose.pose.position.x, goal->pose.pose.position.y,
                    quat.x, quat.y, quat.z, quat.w,
                    goal_yaw, goal_yaw * 180.0 / M_PI);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigateToPose>) {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        goal_active_ = false;
        stop_robot();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        current_goal_handle_ = goal_handle;
        goal_x_ = goal_handle->get_goal()->pose.pose.position.x;
        goal_y_ = goal_handle->get_goal()->pose.pose.position.y;
        goal_yaw_ = get_yaw_from_quaternion(goal_handle->get_goal()->pose.pose.orientation);
        goal_active_ = true;
        RCLCPP_INFO(this->get_logger(), "Goal accepted: x=%.2f, y=%.2f, yaw=%.2f", goal_x_, goal_y_, goal_yaw_);
    }

    std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotontNavigator>());
    rclcpp::shutdown();
    return 0;
}
