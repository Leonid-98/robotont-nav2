from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def _setup(context, *args, **kwargs):
    description_pkg = get_package_share_path("robotont_description")
    bringup_pkg = get_package_share_path("robotont_bringup")
    slam_pkg = get_package_share_path("slam_toolbox")

    primary_color = LaunchConfiguration("primary_color").perform(context).strip()
    world_file = LaunchConfiguration("world_file").perform(context).strip()
    foxglove_port = int(LaunchConfiguration("foxglove_port").perform(context).strip())
    robot_model = str(description_pkg / "urdf/gen3/robotont.urdf.xacro")
    nav2_params = str(bringup_pkg / "config/nav2_params.yaml")
    slam_params = str(bringup_pkg / "config/slam_toolbox.yaml")
    slam_launch = str(slam_pkg / "launch/online_async_launch.py")
    nav2_remaps = [("/tf", "tf"), ("/tf_static", "tf_static")]

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
            parameters=[
                {"odom_frame": "odom"},
                {"base_frame": "base_footprint"},
            ],
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                "use_sim_time": "false",
                "slam_params_file": slam_params,
                "autostart": "true",
                "use_lifecycle_manager": "false",
            }.items(),
        ),
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
                {
                    "node_names": [
                        "controller_server",
                        "smoother_server",
                        "planner_server",
                        "behavior_server",
                        "bt_navigator",
                        "waypoint_follower",
                        "velocity_smoother",
                    ]
                },
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
        DeclareLaunchArgument("world_file", default_value="/ws/worlds/robotont_room.json"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        OpaqueFunction(function=_setup),
    ])
