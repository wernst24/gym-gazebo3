#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_gym_gazebo3 = get_package_share_directory('gym_gazebo3')

    # Launch configuration variables
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gym_gazebo3, 'worlds', 'turtlebot3_camera_world.world'),
        description='Full path to world model file to load'
    )

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Whether to execute gzclient'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_server.launch.py')),
        launch_arguments={'world_sdf_file': world}.items()
    )

    # Start Gazebo client (GUI) conditionally
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'world_sdf_file': world, 'headless': headless}.items(),
        condition=IfCondition(PythonExpression(["'", headless, "' != 'true'"]))
    )

    # ROS-Gazebo bridge for depth camera
    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for laser scan
    laser_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ],
        output='screen'
    )

    # ROS-Gazebo bridge for cmd_vel and odometry
    control_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add Gazebo launch actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add ROS-Gazebo bridges
    ld.add_action(depth_bridge)
    ld.add_action(laser_bridge)
    ld.add_action(control_bridge)

    return ld