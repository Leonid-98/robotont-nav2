FROM ros:jazzy-ros-base-noble

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=jazzy
ENV ROS_WS=/ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    ros-jazzy-ament-cmake \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-launch \
    ros-jazzy-launch-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-map-server \
    ros-jazzy-nav2-msgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-slam-toolbox \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-visualization-msgs \
    ros-jazzy-xacro \
 && rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}

COPY src ./src
COPY worlds ./worlds
COPY scripts ./scripts
COPY src/robotont_bringup/config ./config
COPY src/robotont_bringup/worlds ./gazebo_worlds

RUN chmod +x ./scripts/launch_robotont.sh

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --symlink-install

# slam_toolbox SerializePoseGraph writes relative filenames to the process CWD;
# defaulting CWD here makes Foxglove/slam "save as test1" land on the host via ./saved_maps
RUN mkdir -p /ws/saved_maps
WORKDIR /ws/saved_maps

EXPOSE 8765
EXPOSE 10317/udp
EXPOSE 10318/udp

CMD ["/ws/scripts/launch_robotont.sh"]
