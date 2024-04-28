from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="laser_filters",
            executable="scan_to_scan_filter_chain",
            name="scan_to_scan_filter_chain",
            parameters=[
                PathJoinSubstitution([
                    get_package_share_directory("scout_description"),
                    "config", "laser_filter.yaml",
                ])],
            remappings=[("/tf", "scout_mini/tf"), 
                        ("/tf_static", "scout_mini/tf_static"),
                        ("/scan", "scout_mini/scan"),
                        ("/scan_filtered", "scout_mini/scan_filtered")
                        ]
        )
    ])