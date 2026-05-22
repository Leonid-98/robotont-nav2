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
    ros-jazzy-nav2-msgs \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-tf2-geometry-msgs \
    ros-jazzy-xacro \
 && rm -rf /var/lib/apt/lists/*

WORKDIR ${ROS_WS}

COPY src ./src

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && colcon build --symlink-install

EXPOSE 8765

CMD ["bash", "-lc", "source /ws/install/setup.bash && ros2 launch robotont_bringup robotont_foxglove.launch.py"]
