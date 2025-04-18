#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():

    # Launch configuration variables specific to simulation
    namespace = LaunchConfiguration("namespace", default="scout_mini")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    yaw_pose = LaunchConfiguration("yaw_pose", default="0.0")

    # Declare the launch arguments
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace", default_value=namespace, description="Specify robot namespace"
    )

    declare_x_pose_arg = DeclareLaunchArgument(
        "x_pose", default_value=x_pose, description="Specify robot x position"
    )

    declare_y_pose_arg = DeclareLaunchArgument(
        "y_pose", default_value=y_pose, description="Specify robot y position"
    )

    declare_yaw_pose_arg = DeclareLaunchArgument(
        "yaw_pose", default_value=yaw_pose, description="Specify robot yaw angle"
    )

    # Nodes
    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "scout_mini/robot_description",
            "-name",  "scout_mini",
            "-x", x_pose,
            "-y", y_pose,
            "-z", "0.245",
            "-Y", yaw_pose,
        ],
        output="screen",
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_x_pose_arg)
    ld.add_action(declare_y_pose_arg)
    ld.add_action(declare_yaw_pose_arg)

    # Add any conditioned actions
    ld.add_action(start_gazebo_ros_spawner_cmd)

    return ld
