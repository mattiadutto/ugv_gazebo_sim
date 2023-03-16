import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

 
def generate_launch_description():
    # Specify the name of the package and xacro files
    pkg_name = 'scout_description'
    xacro_file = 'scout_mini.urdf'

    # Set parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Create paths to xacro file
    xacro_path = os.path.join(get_package_share_directory(pkg_name),'urdf',xacro_file)
    robot_description_raw = xacro.process_file(xacro_path).toxml()

    # Set GAZEBO environment variables
    install_dir = get_package_prefix(pkg_name)
    gazebo_models_path = os.path.join(pkg_name, 'meshes')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    # Configure the node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
        'use_sim_time': use_sim_time}]
    )
 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )
 
    spawn_entity = Node(
         package='gazebo_ros',
         executable='spawn_entity.py',
         arguments=['-topic', 'robot_description','-entity', 'my_bot'],
         output='screen'
    )

    static_transform = Node(
         package = "tf2_ros", 
         executable = "static_transform_publisher",
         arguments = ["0", "0", "0", "0", "0", "0", "base_link", "base_footprint 40"]
    )
 
    fake_joint_calibration = ExecuteProcess(
         name='fake_joint_calibration',
         cmd = ['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/Bool', '{data: True}'],
         output = 'screen',
         shell='True'
    )
      
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

   # Run the node
    return LaunchDescription([
        use_sim_time_arg,
        gazebo,
        node_robot_state_publisher,
        static_transform,
        spawn_entity,
        fake_joint_calibration
    ])

