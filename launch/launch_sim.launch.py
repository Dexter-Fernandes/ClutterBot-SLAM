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

    # /home/dev_ws/src/my_bot/description/robot.urdf.xacro
    urdf_path = PathJoinSubstitution([pkg_share, "description", "robot.urdf.xacro"])

    # urdf_path = "/home/dev_ws/src/my_bot/description/robot.urdf.xacro"

    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)

    # rsp = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [
    #             os.path.join(
    #                 pkg_share, "launch", "rsp.launch.py"
    #             )
    #         ]
    #     ),
    #     launch_arguments={"use_sim_time": "true"}.items(),
    # )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
        output="screen",
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
                    "my_world.world",
                ),
            ],
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "my_bot", "-topic", "robot_description", "-z", "0.1"],
        output="screen",
    )

    # controller_manager = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[
    #         {"robot_description": robot_description},
    #         PathJoinSubstitution([pkg_share, "config", "controller_manager.yaml"]),
    #     ],
    #     output="screen",
    # )

    # joint_broad_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=[
    #         "joint_broad"
    # "joint_state_broadcaster",
    # "--controller-manager",
    # "/controller_manager",
    #     ],
    # )

    # diff_drive_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    # arguments=["diff_drive", "--controller-manager", "/controller_manager"],
    #     arguments=["diff_cont"],
    # )

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

    return LaunchDescription(
        [
            rsp,
            gazebo,
            spawn_entity,
            ros_gz_bridge,
            ros_gz_image_bridge,
            # controller_manager,
            # joint_broad_spawner,
            # diff_drive_spawner,
        ]
    )
