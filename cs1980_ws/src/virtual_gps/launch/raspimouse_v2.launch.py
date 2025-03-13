# The MIT License (MIT)
#
# Copyright 2023-2024 RT Corporation <support@rt-net.jp>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of
# this software and associated documentation files (the "Software"), to deal in
# the Software without restriction, including without limitation the rights to
# use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
# the Software, and to permit persons to whom the Software is furnished to do so,
# subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
# FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
# COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
# IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
# CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch_ros.descriptions import ComposableNode

from raspimouse_description.robot_description_loader import RobotDescriptionLoader


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory('virtual_gps'),
        'config',
        'diff_drive_controller.yaml'
    )

    declare_arg_lidar = DeclareLaunchArgument(
        'lidar',
        default_value='none',
        description='Set "none", "urg", "lds", or "rplidar".',
    )
    declare_arg_lidar_frame = DeclareLaunchArgument(
        'lidar_frame', default_value='laser', description='Set lidar link name.'
    )
    declare_arg_use_rgb_camera = DeclareLaunchArgument(
        'use_rgb_camera',
        default_value='false',
        description='Set "true" to mount rgb camera.',
    )
    declare_arg_camera_downward = DeclareLaunchArgument(
        'camera_downward',
        default_value='false',
        description='Set "true" to point the camera downwards.',
    )
    declare_arg_world_name = DeclareLaunchArgument(
        'world_name',
        default_value=get_package_share_directory('raspimouse_gazebo')
        + '/worlds/default.sdf',
        description='Set world name.',
    )
    declare_arg_spawn_x = DeclareLaunchArgument(
        'spawn_x', default_value='0.0', description='Set initial position x.'
    )
    declare_arg_spawn_y = DeclareLaunchArgument(
        'spawn_y', default_value='0.0', description='Set initial position y.'
    )
    declare_arg_spawn_z = DeclareLaunchArgument(
        'spawn_z', default_value='0.02', description='Set initial position z.'
    )

    env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
        'GZ_SIM_RESOURCE_PATH': os.path.dirname(
            get_package_share_directory('raspimouse_description')
        )
        + ':'
        + os.path.join(get_package_share_directory('raspimouse_gazebo'), 'models')
        + ':'
        + os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gz/models/'),       # default world is here 
    }
#    gui_config = os.path.join(
#        get_package_share_directory('raspimouse_gazebo'), 'gui', 'gui.config'
#    )
    gz_sim = ExecuteProcess(
        cmd=[
            'gz sim -r',
            LaunchConfiguration('world_name'),
#            '--gui-config',
#            gui_config,
        ],
        output='screen',
        additional_env=env,
        shell=True,
    )

    gz_spawn_entity1 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            '/robot_description',
            '-name',
            'raspimouse_1',
            '-x',
            LaunchConfiguration('spawn_x'),
            '-y',
            LaunchConfiguration('spawn_y'),
            '-z',
            LaunchConfiguration('spawn_z'),
            '-allow_renaming',
            'true',
        ],
    )

    gz_spawn_entity2 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic',
            '/robot_description',
            '-name',
            'raspimouse_2',
            '-x',
            '2.0',
            '-y',
            '2.0',
            '-z',
            '0.02',
            '-allow_renaming',
            'true',
        ],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.lidar = LaunchConfiguration('lidar')
    description_loader.lidar_frame = LaunchConfiguration('lidar_frame')
    description_loader.use_gazebo = 'true'
    description_loader.use_rgb_camera = LaunchConfiguration('use_rgb_camera')
    description_loader.camera_downward = LaunchConfiguration('camera_downward')
    description_loader.gz_control_config_package = 'raspimouse_gazebo'
    description_loader.gz_control_config_file_path = (
        'config/raspimouse_controllers.yaml'
    )

    control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[config_file],  # Load the YAML file here
            output="screen",
    )

    robot_state_publisher1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        #namespace='raspimouse_1',
        output='screen',
        parameters=[{'robot_description': description_loader.load()}],
    )

    diff_drive_controller_1 = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robot1_diff_drive_controller"],
            output="screen",
    )

    diff_drive_controller_2 = Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robot2_diff_drive_controller"],
            output="screen",
    )

    spawn_joint_state_broadcaster1 = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
        shell=True,
        output='screen',
    )

    """
    robot_state_publisher2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='raspimouse_2',
        output='screen',
        parameters=[{'robot_description': description_loader.load()}],
    )
    
    
    spawn_joint_state_broadcaster1 = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
        shell=True,
        output='screen',
    )
    
    spawn_joint_state_broadcaster2 = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
        shell=True,
        output='screen',
    )
    

    spawn_diff_drive_controller1 = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner diff_drive_controller'],
        shell=True,
        output='screen',
    )

    spawn_diff_drive_controller2 = ExecuteProcess(
        cmd=['ros2 run controller_manager spawner diff_drive_controller'],
        shell=True,
        output='screen',
    )
    """

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/camera/color/image_raw@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        output='screen',
    )

    container1 = ComposableNodeContainer(
        name='fake_raspimouse_container_1',
        namespace='raspimouse_1',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='raspimouse_fake',
                plugin='fake_raspimouse::Raspimouse',
                name='raspimouse_1',
            ),
        ],
        output='screen',
    )

    container2 = ComposableNodeContainer(
        name='fake_raspimouse_container_2',
        namespace='raspimouse_2',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='raspimouse_fake',
                plugin='fake_raspimouse::Raspimouse',
                name='raspimouse_2',
            ),
        ],
        output='screen',
    )

    return LaunchDescription(
        [
            SetParameter(name='use_sim_time', value=True),
            declare_arg_lidar,
            declare_arg_lidar_frame,
            declare_arg_use_rgb_camera,
            declare_arg_camera_downward,
            declare_arg_world_name,
            declare_arg_spawn_x,
            declare_arg_spawn_y,
            declare_arg_spawn_z,
            gz_sim,
            gz_spawn_entity1,
            gz_spawn_entity2,
            control_node,
            robot_state_publisher1,
            #robot_state_publisher2,
            spawn_joint_state_broadcaster1,
            #spawn_joint_state_broadcaster2,
            #spawn_diff_drive_controller1,
            #spawn_diff_drive_controller2,
            diff_drive_controller_1,
            diff_drive_controller_2,
            bridge,
            container1,
            container2,
        ]
    )
