import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )

    print(f"PATH: {get_package_share_directory('scout_description')}")

    robot_localization_local_node = Node(
        namespace="scout_mini",
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("scout_description"), 'config', 'ekf.yaml'), 
                    {"use_sim_time": use_sim_time}],
        remappings=[("/tf", "tf"), 
                    ("/tf_static", "tf_static"),]
                    # ("/odometry/filtered", "odom/local2")]
        # remappings=remapping
        # + [
        #     ("odometry/filtered", "odometry/filtered/local"),
        # ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(robot_localization_local_node)

    return ld
