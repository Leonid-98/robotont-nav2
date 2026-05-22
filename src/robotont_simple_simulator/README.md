# robotont_simple_simulator

![ROS 2](https://img.shields.io/badge/ROS2%20-Jazzy-blue.svg) [![CI](https://github.com/robotont/robotont_simple_simulator/actions/workflows/industrial_ci_action.yml/badge.svg)](https://github.com/robotont/robotont_simple_simulator/actions/workflows/industrial_ci_action.yml) ![License](https://img.shields.io/badge/License-Apache_2.0-green.svg)

A minimalistic simulator package for the Robotont robot, providing basic navigation capabilities with naive physics integration. This package includes a simple driver for basic movement and a navigator for goal-based navigation.

## Installation

### Prerequisites
- ROS2 Jazzy
- Colcon build tools

### Install Dependencies

```bash
# Install required ROS2 packages
sudo apt update
sudo apt install ros-jazzy-nav2-msgs \
                 ros-jazzy-tf2-geometry-msgs \
                 ros-jazzy-teleop-twist-keyboard

# Install rosdep if not already installed
sudo apt install python3-rosdep

# Initialize rosdep (only needed once)
sudo rosdep init
rosdep update
```

### Clone and Build

```bash
# Navigate to your ROS2 workspace src directory
cd ~/your_workspace/src

# Clone the repository
git clone https://github.com/robotont/robotont_simple_simulator

# Install package dependencies using rosdep
cd ~/your_workspace
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select robotont_simple_simulator

# Source the workspace
source install/setup.bash
```

## Package Components

This package provides two main components:

### 1. Simple Driver
A basic driver that integrates velocity commands into odometry using naive kinematic integration. When commanded to move at 1 m/s for 1 second, it will publish odometry confirming 1 meter translation.

### 2. Simple Navigator
A goal-based navigation node that accepts NavigateToPose actions and drives the robot to specified positions and orientations using simple proportional control.

## Usage

### Simple Driver

Launch the simple driver:
```bash
ros2 launch robotont_simple_simulator simple_driver.launch.py
```

Test with keyboard teleop:
```bash
# In a separate terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The driver will:
- Subscribe to `/cmd_vel` for velocity commands
- Publish to `/odom` with integrated odometry
- Broadcast TF transforms between `odom` and `base_link`

### Simple Navigator

Launch the simple navigator with default parameters:
```bash
ros2 launch robotont_simple_simulator simple_navigator.launch.py
```

Launch with custom speed parameters:
```bash
# Launch with faster speeds
ros2 launch robotont_simple_simulator simple_navigator.launch.py linear_speed:=0.8 angular_speed:=1.5

# Launch with slower, more precise speeds
ros2 launch robotont_simple_simulator simple_navigator.launch.py linear_speed:=0.1 angular_speed:=0.3

# Override just one parameter
ros2 launch robotont_simple_simulator simple_navigator.launch.py linear_speed:=0.5
```

Test with action commands:
```bash
# Send a navigation goal to position (1.0, 2.0) with default orientation
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 1.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}}'

# Send a goal with specific orientation (90 degrees rotation)
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}}}}'
```

The navigator will:
- Accept NavigateToPose action goals
- Drive the robot to the specified position
- Rotate to match the goal orientation
- Provide feedback during navigation
- Report success/failure upon completion

## Topics and Services

### Simple Driver
**Subscribed Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands

**Published Topics:**
- `/odom` (nav_msgs/Odometry) - Robot odometry

**TF Broadcasts:**
- `odom` → `base_link`

### Simple Navigator
**Subscribed Topics:**
- `/odom` (nav_msgs/Odometry) - Robot odometry for navigation

**Published Topics:**
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to drive the robot

**Action Servers:**
- `/navigate_to_pose` (nav2_msgs/action/NavigateToPose) - Navigation goals

## Parameters

### Simple Navigator
- `linear_speed`: Maximum linear velocity (default: 0.2 m/s)
- `angular_speed`: Maximum angular velocity (default: 0.5 rad/s)

## Navigation Behavior

The simple navigator uses a two-phase approach:
1. **Phase 1**: Navigate to the goal position while roughly facing the target
2. **Phase 2**: Once at position, rotate to match the exact goal orientation

The navigation uses simple proportional control with configurable speed limits and tolerance thresholds.

## Limitations

This is a **simple simulator** with the following limitations:
- Naive kinematic integration (no physics simulation)
- No obstacle avoidance
- No path planning
- Basic proportional control only
- No sensor simulation

## Contributing

When contributing to this package, please:
- Follow ROS2 coding standards
- Test both components thoroughly
- Update documentation as needed
- Maintain the simplicity principle

## Credits

The simple_navigator_node was originally conceived and implemented by Märten Mikk as part of Robotics Technology course at the University of Tartu in 2025 spring semester.
