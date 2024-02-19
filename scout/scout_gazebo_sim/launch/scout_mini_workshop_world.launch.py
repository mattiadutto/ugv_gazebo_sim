#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Specify the name of the package
    pkg_name = "scout_gazebo_sim"
    namespace = "scout_mini"

    launch_file_dir = os.path.join(get_package_share_directory(pkg_name), "launch")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    # Arguments and parameters
    use_rviz = LaunchConfiguration("use_rviz", default="true")
    rviz_config_file = LaunchConfiguration(
        "rviz_config_file", default="scout_mini.rviz"
    )
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pose = LaunchConfiguration("x_pose", default="0.0")
    y_pose = LaunchConfiguration("y_pose", default="0.0")
    yaw_pose = LaunchConfiguration("yaw_pose", default="3.14")
    world_name = LaunchConfiguration("world_name", default="workshop_big.world")

    declare_world_name_arg = DeclareLaunchArgument(
        "world_name", default_value=world_name, description="Specify Gazebo world name"
    )

    declare_user_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value=use_rviz, description="Run rviz if true"
    )

    declare_rviz_config_file_arg = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=rviz_config_file,
        description="Rviz configuration file",
    )

    # Set GAZEBO environment variables
    install_dir = get_package_prefix(pkg_name)
    gazebo_models_path = os.path.join(pkg_name, "models")

    if "GAZEBO_MODEL_PATH" in os.environ:
        os.environ["GAZEBO_MODEL_PATH"] = (
            os.environ["GAZEBO_MODEL_PATH"]
            + ":"
            + install_dir
            + "/share"
            + ":"
            + gazebo_models_path
        )
    else:
        os.environ["GAZEBO_MODEL_PATH"] = (
            install_dir + "/share" + "/" + gazebo_models_path
        )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": PathJoinSubstitution(
                [get_package_share_directory(pkg_name), "worlds", world_name]
            )
        }.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
        )
    )

    # Add namespace to rviz config file
    namespaced_rviz_config_file = ReplaceString(
        source_file=PathJoinSubstitution(
            [get_package_share_directory("scout_description"), "rviz", rviz_config_file]
        ),
        replacements={"/robot_namespace": ("/", namespace)},
    )

    # Nodes
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace=namespace,
        name="rviz2",
        arguments=["-d", namespaced_rviz_config_file],
        condition=IfCondition(use_rviz),
        remappings=[
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
            ("/initialpose", "initialpose"),
            ("/move_base_simple/goal", "move_base_simple/goal"),
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "scout_mini_robot_state_publisher.launch.py")
        ),
        launch_arguments={"use_sim_time": use_sim_time, "namespace": namespace}.items(),
    )

    robot_spawn_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, "spawn_scout_mini.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "x_pose": x_pose,
            "y_pose": y_pose,
            "yaw_pose": yaw_pose,
        }.items(),
    )
    # File with twist_mux params
    twist_mux_params = os.path.join(
        get_package_share_directory("scout_description"), "config", "twist_mux.yaml"
    )

    # Add the twist_mux node
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings={("/cmd_vel_out", "cmd_vel")},
        parameters=[twist_mux_params, {"use_sim_time": True}],
    )

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_world_name_arg)
    ld.add_action(declare_user_rviz_arg)
    ld.add_action(declare_rviz_config_file_arg)

    # Add the commands to the launch description
    ld.add_action(twist_mux_node)

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_spawn_cmd)
    ld.add_action(robot_state_publisher_cmd)

    ld.add_action(rviz_node)

    return ld
