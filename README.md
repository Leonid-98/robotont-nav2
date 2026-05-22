# Robotont Foxglove Docker Simulation

Barebones ROS 2 Jazzy workspace for running the Robotont simple simulator in Docker and viewing the 3D robot model through Foxglove.

This is a headless URDF/TF/odometry simulation, not a Gazebo physics simulation. The container publishes the Robotont model, joint states, TF, odometry, and a Foxglove websocket bridge.

## Run

```bash
cd robotics-project
docker compose up --build
```

Foxglove Bridge is exposed at:

```text
ws://localhost:8765
```

## View in Foxglove

1. Open Foxglove Desktop or <https://app.foxglove.dev>.
2. Connect to `ws://localhost:8765`.
3. Open a 3D panel.
4. Set the fixed frame to `odom`.
5. Check that these topics are present:
   - `/tf`
   - `/tf_static`
   - `/odom`
   - `/robot_description`
   - `/cmd_vel`

The robot should appear at the odometry origin. It will move when velocity commands are published.

## Move the Robot

In another terminal:

```bash
cd robotics-project
docker compose exec robotont bash
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.5}}" --rate 10
```

Stop the publisher with `Ctrl+C`.

## Verify the ROS Graph

With the compose service running:

```bash
cd robotics-project
docker compose exec robotont bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 node list"
```

Expected nodes:

```text
/driver
/foxglove_bridge
/joint_state_publisher
/navigator
/robot_state_publisher
```

Verify topics:

```bash
docker compose exec robotont bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic list"
```

Expected topics include:

```text
/cmd_vel
/odom
/robot_description
/tf
/tf_static
```

## Launch Arguments

The default launch command is:

```bash
ros2 launch robotont_bringup robotont_foxglove.launch.py
```

Optional arguments:

```bash
generation:=3
primary_color:="0.16 0.65 0.98 1.0"
linear_speed:=0.2
angular_speed:=0.5
```
