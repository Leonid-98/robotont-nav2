from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('linear_speed', default_value='0.2'),
        DeclareLaunchArgument('angular_speed', default_value='0.5'),
        
        Node(
            package='robotont_simple_simulator',
            namespace='',
            executable='simple_driver_node',
            name='driver',
            parameters=[],
        ),
        Node(
            package='robotont_simple_simulator',
            namespace='',
            executable='simple_navigator_node',
            name='navigator',
            parameters=[
                {'linear_speed': LaunchConfiguration('linear_speed')},
                {'angular_speed': LaunchConfiguration('angular_speed')},
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('robotont_description'), 'launch/display_robot_model.launch.py')
            ),
            launch_arguments={'rviz_fixed_frame': 'odom'}.items()
        )
    ])