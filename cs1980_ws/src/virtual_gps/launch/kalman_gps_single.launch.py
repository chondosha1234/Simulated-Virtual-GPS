import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    launch_description = LaunchDescription([
        Node(
            package='virtual_gps',
            executable='drone_control_single',
            name='drone_control_x500_0',
            parameters=[
                {'robot_name': 'x500_0'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='sensor',
            name='sensor_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='orientation',
            name='orientation_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='virtual_gps_v3',
            name='virtual_gps_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
                {'num_drones': 1},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='error_measure',
            name='error_measure_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='writer',
            name='writer_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='kalman_filter',
            name='filter_raspimouse',
            parameters=[
                {'robot_name':'raspimouse'},
            ],
        ),
        
    ])

    return launch_description
