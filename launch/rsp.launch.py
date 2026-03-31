import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_sensor_fusion = LaunchConfiguration('use_sensor_fusion')

    # Process the URDF file
    pkg_path = os.path.join(get_package_share_directory('my_bot'))
    xacro_file = os.path.join(pkg_path,'description','robot.urdf.xacro')
    robot_description_config = ParameterValue(
        Command(
            [
                "xacro ",
                xacro_file,
                " use_ros2_control:=",
                use_ros2_control,
                " use_sensor_fusion:=",
                use_sensor_fusion,
            ]
        ),
        value_type=str,
    )
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )


    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'use_ros2_control',
            default_value='true',
            description='Use ros2_control-based drive stack when true'),

        DeclareLaunchArgument(
            'use_sensor_fusion',
            default_value='true',
            description='Enable EKF sensor fusion and make EKF own odom TF'),

        node_robot_state_publisher
    ])
