import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    launch_description = LaunchDescription([

        Node(
            package='virtual_gps',
            executable='mouse_control',
            name='mouse_control_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='kalman_filter',
            name='kalman_filter_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='kalman_error_measure',
            name='kalman_error_measure_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        
    ])

    return launch_description
