#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the urdf file
    sdf_path = os.path.join(
        get_package_share_directory('bunker_gazebo_sim'),
        'models',
        'bunker',
        'bunker.sdf'
    )

    # Launch configuration variables specific to simulation
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # Declare the launch arguments
    declare_x_position_arg = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify robot x position')

    declare_y_position_arg = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify robot y position')

    start_gazebo_ros_spawner_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bunker',
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.4'
        ],
        output='screen',
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_x_position_arg)
    ld.add_action(declare_y_position_arg)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld


