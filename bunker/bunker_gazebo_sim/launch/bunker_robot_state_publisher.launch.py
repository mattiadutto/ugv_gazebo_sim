#!/usr/bin/env python3

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.substitutions import LaunchConfiguration, FindExecutable
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
 
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value=use_rviz,
        description='Run rviz if true')
        
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    # Load xacro file for rviz
    xacro_path = os.path.join(
        get_package_share_directory('bunker_description'),
        'urdf',
        'bunker.xacro')

    robot_description_raw = xacro.process_file(xacro_path).toxml()

    start_rviz = ExecuteProcess(
       condition=IfCondition(use_rviz),
        cmd=[[
            FindExecutable(name='ros2'),
            ' run rviz2 rviz2 -d ',
            str(os.path.join(get_package_share_directory('bunker_description'),'rviz/navigation.rviz'))
        ]],
        shell=True
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_raw
        }],
        remappings=[
            ('/joint_states', '/bunker/joint_states')
        ]
    )

    return LaunchDescription([
        use_rviz_arg,
        use_sim_time_arg,
        robot_state_publisher_node,

        RegisterEventHandler(
            OnProcessStart(
                target_action=robot_state_publisher_node,
                on_start=[
                   TimerAction(
                      period=2.0,
                      actions=[start_rviz],   
                   )     
                ]
            )
        ),
    ])

