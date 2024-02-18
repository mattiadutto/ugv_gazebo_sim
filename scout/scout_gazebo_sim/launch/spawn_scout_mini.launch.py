#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the urdf file
    sdf_path = os.path.join(
        get_package_share_directory("scout_gazebo_sim"),
        "models",
        "scout_mini",
        "scout_mini.sdf",
    )

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

    # Add namespace to gazebo model file
    namespaced_sdf_file = ReplaceString(
        source_file=os.path.join(
            get_package_share_directory("scout_gazebo_sim"),
            "models",
            "scout_mini",
            "scout_mini.sdf",
        ),
        replacements={"/robot_namespace": ("/", namespace)},
    )

    # Nodes
    start_gazebo_ros_spawner_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            (namespace, "_robot"),
            "-file",
            namespaced_sdf_file,
            "-x",
            x_pose,
            "-y",
            y_pose,
            "-z",
            "0.245",
            "-Y",
            yaw_pose,
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
