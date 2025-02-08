import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    launch_description = LaunchDescription([
        Node(
            package='virtualGPS',
            executable='virtual_gps',
            name='virtual_gps',
            parameters=[],
        ),
        Node(
            package='virtualGPS',
            executable='sensor',
            name='sensor',
            parameters=[],
        ),
    ])

    return launch_description
