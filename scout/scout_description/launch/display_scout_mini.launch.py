import os
import xacro
import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

 
def generate_launch_description():
    # Manage input arguments
    use_gui_value = LaunchConfiguration('use_gui')
 
    use_gui_launch_arg = DeclareLaunchArgument(
        'use_gui',
        default_value='False',
        description='Select if you want to start the gui.'
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Specify the name of the package, xacro and rviz files within the package
    pkg_name = 'scout_description'
    xacro_file = 'scout_mini.xacro'
    rviz_config_file = 'scout_mini_model_display.rviz'

    # Create paths to xacro and rviz files
    xacro_path = os.path.join(get_package_share_directory(pkg_name),'urdf',xacro_file)
    rviz_config_path = os.path.join(get_package_share_directory(pkg_name),'rviz',rviz_config_file)
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # Add namespace to rviz config file
    namespaced_rviz_config_file = ReplaceString(
        source_file=PathJoinSubstitution([get_package_share_directory('scout_description'),'rviz',
            rviz_config_file]), 
        replacements={'/robot_namespace': ('/', 'scout_mini')})

    # Configure the nodes
    node_joint_state_publisher_gui = Node(
        namespace='scout_mini',
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.IfCondition(use_gui_value)
    )

    node_joint_state_publisher = Node(
        namespace='scout_mini',
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=launch.conditions.UnlessCondition(use_gui_value)
    )

    node_robot_state_publisher = Node(
        namespace='scout_mini',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
       ]
    )

    rviz = Node(
        namespace='scout_mini',
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', namespaced_rviz_config_file],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static')
        ]
    )

    # Run the nodes
    return LaunchDescription([
        use_sim_time_arg,
        use_gui_launch_arg,
        node_joint_state_publisher_gui,
        node_joint_state_publisher,
        node_robot_state_publisher,
        rviz,
    ])

