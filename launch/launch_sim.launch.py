import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():

    pkg_name = "my_bot"
    pkg_share = get_package_share_directory(pkg_name)
    use_ros2_control = LaunchConfiguration("use_ros2_control")
    use_sensor_fusion = LaunchConfiguration("use_sensor_fusion")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(pkg_share, "launch", "rsp.launch.py")]
        ),
        launch_arguments={
            "use_sim_time": "true",
            "use_ros2_control": use_ros2_control,
            "use_sensor_fusion": use_sensor_fusion,
        }.items(),
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

    bridge_params_legacy = os.path.join(pkg_share, "config", "gz_bridge.yaml")
    bridge_params_legacy_sensor_fusion = os.path.join(
        pkg_share, "config", "gz_bridge_legacy_sensor_fusion.yaml"
    )
    bridge_params_ros2_control = os.path.join(
        pkg_share, "config", "gz_bridge_ros2_control.yaml"
    )
    ros_gz_bridge_legacy = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params_legacy}"],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", use_ros2_control, "' == 'false' and '", use_sensor_fusion, "' == 'false'"]
            )
        ),
    )

    ros_gz_bridge_legacy_sensor_fusion = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params_legacy_sensor_fusion}",
        ],
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["'", use_ros2_control, "' == 'false' and '", use_sensor_fusion, "' == 'true'"]
            )
        ),
    )

    ros_gz_bridge_ros2_control = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["--ros-args", "-p", f"config_file:={bridge_params_ros2_control}"],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    ros_gz_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=["/camera/image_raw"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "120",
        ],
        output="screen",
        condition=IfCondition(use_ros2_control),
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner],
        condition=IfCondition(use_ros2_control),
    )

    delayed_diff_drive_controller_spawner = TimerAction(
        period=5.0,
        actions=[diff_drive_controller_spawner],
        condition=IfCondition(use_ros2_control),
    )

    ekf_params = os.path.join(pkg_share, "config", "ekf.yaml")
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params],
        condition=IfCondition(use_sensor_fusion),
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
            DeclareLaunchArgument(
                "use_ros2_control",
                default_value="true",
                description="Use ros2_control diff_drive stack instead of Gazebo DiffDrive plugin",
            ),
            DeclareLaunchArgument(
                "use_sensor_fusion",
                default_value="true",
                description="Enable EKF sensor fusion and use EKF as odom TF source",
            ),
            rsp,
            gazebo,
            spawn_entity,
            ros_gz_bridge_legacy,
            ros_gz_bridge_legacy_sensor_fusion,
            ros_gz_bridge_ros2_control,
            ros_gz_image_bridge,
            delayed_joint_state_broadcaster_spawner,
            delayed_diff_drive_controller_spawner,
            ekf_node,
            # slam_toolbox,
            # rtabmap,
            rviz2,
        ]
    )
