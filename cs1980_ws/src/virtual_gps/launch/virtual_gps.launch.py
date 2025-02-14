import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    launch_description = LaunchDescription([
        Node(
            package='virtual_gps',
            executable='tf_pose_broadcaster',
            name='tf_pose_broadcaster',
            parameters=[
                {'robot_name':'x500_1'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='tf_pose_broadcaster',
            name='tf_pose_broadcaster',
            parameters=[
                {'robot_name':'x500_2'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='tf_pose_broadcaster',
            name='tf_pose_broadcaster',
            parameters=[
                {'robot_name':'x500_3'},
            ],
        ),
        Node(
            package='virtual_gps',
            executable='tf_pose_broadcaster',
            name='tf_pose_broadcaster',
            parameters=[
                {'robot_name':'x500_4'},
            ],
        ),
    ])

    return launch_description
