from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('px4_offboard')
    return LaunchDescription([
            Node(
            package='px4_offboard',
            namespace='px4_offboard',
            executable='offboard_takeoff_example',
            name='offboard_takeoff_example',
        ),
    ])