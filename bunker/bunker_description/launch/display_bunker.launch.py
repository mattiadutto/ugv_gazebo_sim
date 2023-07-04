import os
import xacro
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

 
def generate_launch_description():
    # Specify the name of the package, xacro and rviz files within the package
    pkg_name = 'bunker_description'
    xacro_file = 'bunker.xacro'
    rviz_config_file = 'model_display.rviz'

    # Set parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Create paths to xacro and rviz files
    xacro_path = os.path.join(get_package_share_directory(pkg_name),'urdf',xacro_file)
    rviz_config_path = os.path.join(get_package_share_directory(pkg_name),'rviz',rviz_config_file)
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # Configure the nodes
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': use_sim_time}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', str(rviz_config_path)],
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Run the nodes
    return LaunchDescription([
        use_sim_time_arg,
        node_joint_state_publisher_gui,
        node_robot_state_publisher,
        rviz,
    ])

