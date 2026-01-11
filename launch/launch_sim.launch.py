import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_name = "my_bot"
    pkg_share = get_package_share_directory(pkg_name)

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share, "launch", "rsp.launch.py")]
        ),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                )
            ]
        ),
        launch_arguments={
            "gz_args": [
                "-r -v4 ",
                os.path.join(
                    pkg_share,
                    "worlds",
                    "warehouse.sdf",
                ),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "my_bot", "-topic",
                   "robot_description", "-z", "0.1"],
        output="screen",
    )

    rviz_config = os.path.join(pkg_share, "config", "view_bot.rviz")
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    bridge_params = os.path.join(pkg_share, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        output="screen",
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
    )

    slam_toolbox_launch = PathJoinSubstitution(
        [
            get_package_share_directory("slam_toolbox"),
            "launch",
            "online_async_launch.py",
        ]
    )
    mapper_params = PathJoinSubstitution(
        [pkg_share, "config", "mapper_params_online_async.yaml"]
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_toolbox_launch),
        launch_arguments={"use_sim_time": "true",
                          "params_file": mapper_params}.items(),
    )

    rtabmap_launch = PathJoinSubstitution(
        [pkg_share, "launch", "rtabmap_lidar.launch.py"]
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch)
    )

    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
            ros_gz_image_bridge,
            # slam_toolbox,
            # rtabmap,
            rviz2,
        ]
    )
