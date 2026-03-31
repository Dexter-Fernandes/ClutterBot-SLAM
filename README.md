# Clutterbot-SLAM

Successor project to [`my_bot`](https://github.com/Dexter-Fernandes/my_bot).

This repo extends my original ROS2 mobile robot simulation by adding:

- Simulated RGB-D camera and IMU
- Sensor fusion with `robot_localization` (EKF)
- SLAM using RTAB-Map / Cartographer / slam_toolbox (fallback)
- Switchable drive control backends:
  - `ros2_control` + `diff_drive_controller` (default)
  - Gazebo DiffDrive plugin fallback (`use_ros2_control:=false`)
- EKF sensor fusion toggle (`use_sensor_fusion:=true|false`, default `true`)
- Filtered odometry output on `/odometry/filtered`
- A cluttered “messy warehouse” world with static and dynamic obstacles
- Basic evaluation of LiDAR-only vs RGB-D + LiDAR mapping quality
