from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from nav2_common.launch import RewrittenYaml


def _checkpoint_path(checkpoint):
    if not checkpoint:
        return ""

    path = Path(checkpoint)
    if not path.is_absolute():
        path = Path("/ws/saved_maps") / path
    if path.is_dir():
        path = path / path.name
    elif not path.suffix:
        bundle_path = path / path.name
        if bundle_path.with_suffix(".posegraph").is_file() or bundle_path.with_suffix(".data").is_file():
            path = bundle_path
    if path.suffix in (".posegraph", ".data"):
        path = path.with_suffix("")

    missing = [
        str(path.with_suffix(suffix))
        for suffix in (".posegraph", ".data")
        if not path.with_suffix(suffix).is_file()
    ]
    if missing:
        raise RuntimeError(
            f"Saved SLAM checkpoint '{checkpoint}' is incomplete. Missing: "
            + ", ".join(missing)
        )
    return str(path)


def _setup(context, *args, **kwargs):
    description_pkg = get_package_share_path("robotont_description")
    slam_pkg = get_package_share_path("slam_toolbox")

    primary_color = LaunchConfiguration("primary_color").perform(context).strip()
    world_file = LaunchConfiguration("world_file").perform(context).strip()
    saved_map = LaunchConfiguration("saved_map").perform(context).strip()
    checkpoint = _checkpoint_path(saved_map)
    foxglove_port = int(LaunchConfiguration("foxglove_port").perform(context).strip())
    nav2_params_file = LaunchConfiguration("nav2_params_file").perform(context).strip()
    slam_params = LaunchConfiguration("slam_params_file").perform(context).strip()
    sim_driver_params = LaunchConfiguration("sim_driver_params_file").perform(context).strip()
    robot_model = str(description_pkg / "urdf/gen3/robotont.urdf.xacro")
    slam_launch = str(slam_pkg / "launch/online_async_launch.py")
    nav2_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key="",
            param_rewrites={"use_sim_time": "false"},
            convert_types=True,
        ),
        allow_substs=True,
    )

    robot_description = ParameterValue(
        Command([
            "xacro ",
            robot_model,
            ' main_color:="',
            primary_color,
            '"',
        ]),
        value_type=str,
    )

    slam_nodes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "use_sim_time": "false",
                "slam_params_file": slam_params,
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
    ]
    if checkpoint:
        slam_nodes.append(
            Node(
                package="robotont_bringup",
                executable="slam_checkpoint_loader_node.py",
                name="slam_checkpoint_loader",
                output="screen",
                parameters=[
                    {"checkpoint": checkpoint},
                    {"use_sim_time": False},
                ],
            )
        )

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
        ),
        Node(
            package="robotont_simple_simulator",
            executable="simple_driver_node",
            name="driver",
            output="screen",
            parameters=[sim_driver_params],
        ),
        Node(
            package="robotont_simple_simulator",
            executable="fake_laser_node.py",
            name="fake_laser",
            output="screen",
            parameters=[
                {"frame_id": "base_scan"},
                {"odom_topic": "/odom"},
                {"scan_topic": "/scan"},
                {"laser_x": 0.08},
                {"laser_y": 0.0},
                {"world_file": world_file},
            ],
        ),
        *slam_nodes,
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[nav2_params],
            remappings=nav2_remaps + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params],
            remappings=nav2_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params],
            remappings=nav2_remaps,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params],
            remappings=nav2_remaps + [("cmd_vel", "cmd_vel_nav")],
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params],
            remappings=nav2_remaps,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params],
            remappings=nav2_remaps,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params],
            remappings=nav2_remaps + [
                ("cmd_vel", "cmd_vel_nav"),
                ("cmd_vel_smoothed", "cmd_vel"),
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"autostart": True},
                {"node_names": lifecycle_nodes},
            ],
        ),
        Node(
            package="robotont_simple_simulator",
            executable="goal_bridge_node.py",
            name="goal_bridge",
            output="screen",
            parameters=[
                {"goal_topic": "/goal_pose"},
                {"default_frame": "map"},
                {"action_name": "navigate_to_pose"},
            ],
        ),
        Node(
            package="robotont_simple_simulator",
            executable="clicked_waypoint_node.py",
            name="clicked_waypoint_node",
            output="screen",
            parameters=[
                {"clicked_point_topic": "/clicked_point"},
                {"default_frame": "map"},
                {"action_name": "navigate_through_poses"},
                {"use_sim_time": False},
            ],
        ),
        Node(
            package="robotont_bringup",
            executable="map_saver_trigger_node.py",
            name="map_saver_trigger",
            output="screen",
            parameters=[
                {"save_directory": "/ws/saved_maps"},
                {"map_topic": "/map"},
                {"service_name": "save_map"},
                {"use_sim_time": False},
            ],
        ),
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            parameters=[
                {"address": "0.0.0.0"},
                {"port": foxglove_port},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument("primary_color", default_value="0.16 0.65 0.98 1.0"),
        DeclareLaunchArgument("world_file", default_value="/ws/worlds/room.json"),
        DeclareLaunchArgument("nav2_params_file", default_value="/ws/config/nav2_params.yaml"),
        DeclareLaunchArgument("slam_params_file", default_value="/ws/config/slam_toolbox.yaml"),
        DeclareLaunchArgument("sim_driver_params_file", default_value="/ws/config/simple_sim_driver.yaml"),
        DeclareLaunchArgument("saved_map", default_value=""),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        OpaqueFunction(function=_setup),
    ])
