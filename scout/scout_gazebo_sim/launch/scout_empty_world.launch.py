#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Specify the name of the package
    pkg_name = 'scout_gazebo_sim'

    launch_file_dir = os.path.join(get_package_share_directory(pkg_name), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
 
    # Arguments and parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    world_name = LaunchConfiguration('world_name', default=os.path.join(
        get_package_share_directory('scout_gazebo_sim'),
        'worlds', 'weston_robot_empty.world'))

    declare_robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace', default_value='/',
        description='Specify namespace of the robot')

    declare_world_name_arg = DeclareLaunchArgument(
        'world_name', default_value=os.path.join(
        get_package_share_directory('scout_gazebo_sim'),
        'worlds', 'weston_robot_empty.world'),
        description='Specify Gazebo world name')

    # Set GAZEBO environment variables
    install_dir = get_package_prefix(pkg_name)
    gazebo_models_path = os.path.join(pkg_name, 'models')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + '/' + gazebo_models_path

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_name}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'scout_robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_scout_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_scout_v2.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )
    
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_robot_namespace_arg)
    ld.add_action(declare_world_name_arg)

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_scout_cmd)

    return ld

