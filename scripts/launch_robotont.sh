#!/usr/bin/env bash
set -eo pipefail

source /ws/install/setup.bash
set -u

mode="${ROBOTONT_WORLD_MODE:-gazebo}"
primary_color="${ROBOTONT_PRIMARY_COLOR:-0.16 0.65 0.98 1.0}"
world_file="${ROBOTONT_WORLD_FILE:-/ws/worlds/room.json}"
map_name="${ROBOTONT_MAP_NAME:-}"
foxglove_port="${ROBOTONT_FOXGLOVE_PORT:-8765}"
extra_args="${ROBOTONT_EXTRA_LAUNCH_ARGS:-}"
nav2_params_file="${ROBOTONT_NAV2_PARAMS_FILE:-/ws/config/nav2_params.yaml}"
slam_params_file="${ROBOTONT_SLAM_PARAMS_FILE:-/ws/config/slam_toolbox.yaml}"
sim_driver_params_file="${ROBOTONT_SIM_DRIVER_PARAMS_FILE:-/ws/config/simple_sim_driver.yaml}"
gazebo_world_file="${ROBOTONT_GAZEBO_WORLD_FILE:-/ws/gazebo_worlds/robotont_room.sdf}"
map_args=()
if [[ -n "${map_name}" ]]; then
  map_args=(saved_map:="${map_name}")
fi

case "${mode}" in
  gazebo)
    exec ros2 launch robotont_bringup robotont_nav2_gazebo.launch.py \
      primary_color:="${primary_color}" \
      gazebo_world_file:="${gazebo_world_file}" \
      nav2_params_file:="${nav2_params_file}" \
      slam_params_file:="${slam_params_file}" \
      "${map_args[@]}" \
      foxglove_port:="${foxglove_port}" \
      ${extra_args}
    ;;
  custom)
    exec ros2 launch robotont_bringup robotont_nav2_slam.launch.py \
      primary_color:="${primary_color}" \
      world_file:="${world_file}" \
      nav2_params_file:="${nav2_params_file}" \
      slam_params_file:="${slam_params_file}" \
      sim_driver_params_file:="${sim_driver_params_file}" \
      "${map_args[@]}" \
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
    echo "Expected one of: gazebo, custom, simple, foxglove" >&2
    exit 2
    ;;
esac
