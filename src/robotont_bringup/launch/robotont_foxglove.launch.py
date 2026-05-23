from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path


def _setup(context, *args, **kwargs):
    pkg = get_package_share_path("robotont_description")

    generation = LaunchConfiguration("generation").perform(context).strip()
    primary_color = LaunchConfiguration("primary_color").perform(context).strip()
    linear_speed = LaunchConfiguration("linear_speed")
    angular_speed = LaunchConfiguration("angular_speed")
    foxglove_port = int(LaunchConfiguration("foxglove_port").perform(context).strip())

    if generation == "2.1":
        robot_model = str(pkg / "urdf/gen2_1/robotont.urdf.xacro")
        xacro_command = ["xacro ", robot_model]
    else:
        robot_model = str(pkg / "urdf/gen3/robotont.urdf.xacro")
        xacro_command = [
            "xacro ",
            robot_model,
            ' main_color:="',
            primary_color,
            '"',
        ]

    robot_description = ParameterValue(
        Command(xacro_command),
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
        ),
        Node(
            package="robotont_simple_simulator",
            executable="simple_navigator_node",
            name="navigator",
            output="screen",
            parameters=[
                {"linear_speed": linear_speed},
                {"angular_speed": angular_speed},
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
        DeclareLaunchArgument("generation", default_value="3"),
        DeclareLaunchArgument("primary_color", default_value="0.16 0.65 0.98 1.0"),
        DeclareLaunchArgument("linear_speed", default_value="0.2"),
        DeclareLaunchArgument("angular_speed", default_value="0.5"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        OpaqueFunction(function=_setup),
    ])
