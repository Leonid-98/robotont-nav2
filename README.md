# Robotont Gazebo + Nav2 + SLAM Foxglove Demo

Barebones ROS 2 Jazzy workspace for running a headless Robotont simulation in Docker, visualizing it in Foxglove, and demoing SLAM + autonomous navigation with Nav2.

The default launch uses Gazebo headlessly. Gazebo owns the truth world, robot physics proxy, `/odom`, and `/scan`. Foxglove always stays the visualization and goal input surface. A Python/JSON custom-world launch is also available when you want to draw a simple 2D world yourself.

## Run

```bash
cd robotics-project
docker compose up --build
```

Foxglove Bridge is exposed at:

```text
ws://localhost:8765
```

The default command launches:

```bash
/ws/scripts/launch_robotont.sh
```

By default, `ROBOTONT_WORLD_MODE=gazebo`, so the launcher starts:

```bash
ros2 launch robotont_bringup robotont_nav2_gazebo.launch.py
```

## View in Foxglove

1. Open Foxglove Desktop or <https://app.foxglove.dev>.
2. Connect to `ws://localhost:8765`.
3. Open a 3D panel.
4. Set the fixed frame to `map`.
5. Check that these topics are present:
   - `/tf`
   - `/tf_static`
   - `/map`
   - `/scan`
   - `/odom`
   - `/robot_description`
   - `/cmd_vel`
   - `/goal_pose`
   - `/navigate_to_pose/_action/status`

The robot should appear in the generated SLAM map. Add or enable map, laser scan, path, costmap, TF, and URDF layers in the 3D panel. Gazebo itself is running without a GUI; Foxglove is where you inspect the demo.

## Send a Nav2 Goal from Foxglove

Use a Foxglove Publish panel:

- Topic: `/goal_pose`
- Type: `geometry_msgs/msg/PoseStamped`

Example goal:

```json
{
  "header": { "stamp": { "sec": 0, "nanosec": 0 }, "frame_id": "map" },
  "pose": {
    "position": { "x": 1.5, "y": 0.5, "z": 0 },
    "orientation": { "x": 0, "y": 0, "z": 0, "w": 1 }
  }
}
```

The `goal_bridge` node forwards `/goal_pose` to Nav2's standard `/navigate_to_pose` action.

The same goal can be sent from a terminal:

```bash
docker compose exec robotont bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}"'
```

## Move the Robot

For manual teleop/debugging, publish velocity commands directly:

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

Nav2 also publishes `/cmd_vel` when it receives and accepts a navigation goal.

## Verify the ROS Graph

With the compose service running:

```bash
cd robotics-project
docker compose exec robotont bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 node list"
```

Expected nodes:

```text
/bt_navigator
/controller_server
/foxglove_bridge
/goal_bridge
/joint_state_publisher
/odom_tf
/planner_server
/robot_state_publisher
/ros_gz_bridge
/slam_toolbox
```

In Gazebo mode, these custom-world nodes should not be running:

```text
/driver
/fake_laser
```

Verify topics:

```bash
docker compose exec robotont bash -lc "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 topic list"
```

Expected topics include:

```text
/cmd_vel
/goal_pose
/map
/navigate_to_pose/_action/status
/odom
/robot_description
/scan
/tf
/tf_static
```

In Gazebo mode, `/scan` and `/odom` come from Gazebo through `ros_gz_bridge`. `/odom_tf` turns Gazebo odometry into the ROS TF edge `odom -> base_footprint`.

## Save the Generated Map

After SLAM has built enough of the fake world:

```bash
docker compose exec robotont bash -lc 'source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash && ros2 run nav2_map_server map_saver_cli -f /tmp/robotont_demo_map'
```

The files are created inside the container under `/tmp/robotont_demo_map.yaml` and `/tmp/robotont_demo_map.pgm`.

## Launch Arguments

The default Gazebo Nav2 demo launch command is:

```bash
ros2 launch robotont_bringup robotont_nav2_gazebo.launch.py
```

Optional arguments:

```bash
primary_color:="0.16 0.65 0.98 1.0"
use_sim_time:=true
foxglove_port:=8765
```

The custom Python fake-laser Nav2 demo launch command is:

```bash
ros2 launch robotont_bringup robotont_nav2_slam.launch.py
```

Optional arguments:

```bash
primary_color:="0.16 0.65 0.98 1.0"
foxglove_port:=8765
```

Run the custom JSON world mode in Docker with:

```bash
docker compose down
ROBOTONT_WORLD_MODE=custom docker compose up
```

Run custom mode with a different JSON world:

```bash
docker compose down
ROBOTONT_WORLD_MODE=custom ROBOTONT_WORLD_FILE=/ws/worlds/custom.json docker compose up
```

Run the original simple simulator:

```bash
docker compose down
ROBOTONT_WORLD_MODE=simple docker compose up
```

## World Model

Gazebo mode uses:

```text
src/robotont_bringup/worlds/robotont_room.sdf
```

That SDF contains a small room, static obstacle boxes, and a simplified Robotont physics proxy with a 2D lidar. The detailed Robotont URDF is still published for Foxglove visualization.

The custom fake-laser mode loads an editable JSON world. If the requested file cannot be read, `fake_laser_node.py` falls back to its built-in demo boxes. The default JSON copy of the Gazebo room is:

```text
worlds/robotont_room.json
```

Open the local editor:

```text
tools/world-editor/index.html
```

The editor exports this shape:

```json
{
  "version": 1,
  "name": "custom",
  "resolution": 0.05,
  "origin": { "x": -3, "y": -2 },
  "bounds": { "min_x": -3, "min_y": -2, "max_x": 3, "max_y": 2 },
  "walls": [
    { "x1": -3, "y1": -2, "x2": 3, "y2": -2 }
  ],
  "boxes": [
    { "name": "obstacle_1", "min_x": -1.2, "min_y": -0.8, "max_x": -0.8, "max_y": 0.8 }
  ]
}
```

Place exported files under:

```text
worlds/
```

Docker Compose mounts that directory into the container at `/ws/worlds`, so changing JSON worlds does not require rebuilding the image.

## Docker Mode Switch

The compose service reads these environment variables:

```text
ROBOTONT_WORLD_MODE=gazebo|custom|simple
ROBOTONT_WORLD_FILE=/ws/worlds/robotont_room.json
ROBOTONT_PRIMARY_COLOR="0.16 0.65 0.98 1.0"
ROBOTONT_FOXGLOVE_PORT=8765
ROBOTONT_FOXGLOVE_HOST_PORT=8765
ROBOTONT_EXTRA_LAUNCH_ARGS=""
GZ_DISCOVERY_MSG_PORT=10317
GZ_DISCOVERY_SRV_PORT=10318
```

Modes:

```text
gazebo    Headless Gazebo world, Gazebo /scan and /odom, Nav2, SLAM, Foxglove
custom    Simple driver, JSON fake laser /scan, Nav2, SLAM, Foxglove
simple    Original simple driver/navigator and URDF Foxglove visualization
```

Examples:

```bash
ROBOTONT_WORLD_MODE=gazebo docker compose up --build
ROBOTONT_WORLD_MODE=custom docker compose up
ROBOTONT_WORLD_MODE=custom ROBOTONT_WORLD_FILE=/ws/worlds/custom.json docker compose up
ROBOTONT_WORLD_MODE=custom ROBOTONT_WORLD_FILE=/ws/worlds/robotont_text.json docker compose up --build
```

Foxglove Bridge is always launched by these modes. It listens inside the container on `ROBOTONT_FOXGLOVE_PORT` and is exposed on the host as `ROBOTONT_FOXGLOVE_HOST_PORT`, both defaulting to `8765`.

Gazebo is separate from Foxglove. In Gazebo mode it runs headlessly and generates `/scan`, `/odom`, and `/clock`; Foxglove still visualizes the ROS topics through `ws://localhost:8765`. Optional Gazebo Transport discovery UDP ports are exposed separately as `10317` and `10318` for tooling that talks directly to Gazebo Transport.

The original simple simulator launch is still available:

```bash
ros2 launch robotont_bringup robotont_foxglove.launch.py
```

Optional arguments:

```bash
generation:=3
primary_color:="0.16 0.65 0.98 1.0"
linear_speed:=0.2
angular_speed:=0.5
foxglove_port:=8765
```
