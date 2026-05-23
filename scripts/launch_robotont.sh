#!/usr/bin/env bash
set -eo pipefail

source /ws/install/setup.bash
set -u

mode="${ROBOTONT_WORLD_MODE:-gazebo}"
primary_color="${ROBOTONT_PRIMARY_COLOR:-0.16 0.65 0.98 1.0}"
world_file="${ROBOTONT_WORLD_FILE:-/ws/worlds/room.json}"
foxglove_port="${ROBOTONT_FOXGLOVE_PORT:-8765}"
extra_args="${ROBOTONT_EXTRA_LAUNCH_ARGS:-}"

case "${mode}" in
  gazebo)
    exec ros2 launch robotont_bringup robotont_nav2_gazebo.launch.py \
      primary_color:="${primary_color}" \
      foxglove_port:="${foxglove_port}" \
      ${extra_args}
    ;;
  custom)
    exec ros2 launch robotont_bringup robotont_nav2_slam.launch.py \
      primary_color:="${primary_color}" \
      world_file:="${world_file}" \
      foxglove_port:="${foxglove_port}" \
      ${extra_args}
    ;;
  simple|foxglove)
    exec ros2 launch robotont_bringup robotont_foxglove.launch.py \
      generation:=3 \
      primary_color:="${primary_color}" \
      foxglove_port:="${foxglove_port}" \
      ${extra_args}
    ;;
  *)
    echo "Unknown ROBOTONT_WORLD_MODE='${mode}'" >&2
    echo "Expected one of: gazebo, custom, simple" >&2
    exit 2
    ;;
esac
