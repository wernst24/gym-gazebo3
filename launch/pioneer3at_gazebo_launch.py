#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_gym_gazebo3 = get_package_share_directory('gym_gazebo3')

    # Launch configuration variables
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare the launch arguments
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gym_gazebo3, 'worlds', 'pioneer3at_world.world'),
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
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression(['not ', headless]))
    )

    # Robot state publisher
    urdf_file = os.path.join(pkg_gym_gazebo3, 'models', 'pioneer3at', 'pioneer3at.urdf')
    
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file]
    )

    # Spawn the robot
    spawn_pioneer_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'pioneer3at',
                  '-file', urdf_file,
                  '-x', '0.0',
                  '-y', '0.0',
                  '-z', '0.1'],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_pioneer_cmd)

    return ld