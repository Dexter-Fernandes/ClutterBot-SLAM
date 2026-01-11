from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("my_bot")
    params_file = LaunchConfiguration("params_file")
    scan_cloud_topic = LaunchConfiguration("scan_cloud_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(
                pkg_share, "config", "rtabmap_lidar_slam.yaml"),
        ),
        DeclareLaunchArgument(
            "scan_cloud_topic",
            default_value="/scan/points",
        ),

        Node(
            package="rtabmap_slam",
            executable="rtabmap",
            name="rtabmap",
            output="screen",
            parameters=[params_file],
            remappings=[
                ("scan_cloud", scan_cloud_topic),
            ],
        ),

        # Optional (uncomment if you want the GUI)
        # Node(
        #     package="rtabmap_viz",
        #     executable="rtabmap_viz",
        #     name="rtabmap_viz",
        #     output="screen",
        #     parameters=[params_file],
        #     remappings=[
        #         ("scan_cloud", scan_cloud_topic),
        #     ],
        # ),
    ])
