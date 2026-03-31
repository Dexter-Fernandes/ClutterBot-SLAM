# Running ClutterBot-SLAM

This workspace uses ROS 2 Jazzy and the `my_bot` package.

## 1) Build the workspace

From the workspace root (`/home/dev_ws`):

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 2) Launch simulation + robot + RViz

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_bot launch_sim.launch.py
```

What this launches (from `launch/launch_sim.launch.py`):
- robot state publisher (`rsp.launch.py`)
- Gazebo (`ros_gz_sim`) with `worlds/warehouse.sdf`
- robot spawn in sim
- ROS <-> Gazebo bridges (sensor-only in ros2_control mode)
- `ros2_control` controller spawners (`joint_state_broadcaster`, `diff_drive_controller`)
- EKF sensor fusion (`robot_localization`) publishing `/odometry/filtered` (enabled by default)
- RViz with `config/view_bot.rviz`

Note: `slam_toolbox` and `rtabmap` are currently commented out in `launch_sim.launch.py`, so SLAM is started separately.

Optional legacy fallback (Gazebo DiffDrive plugin):

```bash
ros2 launch my_bot launch_sim.launch.py use_ros2_control:=false
```

Optional disable sensor fusion fallback:

```bash
ros2 launch my_bot launch_sim.launch.py use_sensor_fusion:=false
```

## 3) Run RTAB-Map LiDAR SLAM (separate terminal)

Open a second terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch my_bot rtabmap_lidar.launch.py
```

This uses `config/rtabmap_lidar_slam.yaml` by default and subscribes to `/scan/points`.

## 4) Reset RTAB-Map database between runs (optional)

If you want a clean map each run:

```bash
rm -f ~/.ros/rtabmap.db
```

Then relaunch `rtabmap_lidar.launch.py`.

## 5) Run Nav2 (separate terminal)

Open another terminal after simulation is running:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```

Tip: with sensor fusion enabled, Nav2/SLAM consume EKF TF (`odom -> base_link`) and filtered odometry is available on `/odometry/filtered`.

## 6) Teleop (keyboard control)

Open another terminal:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/diff_drive_controller/cmd_vel
```

Legacy fallback teleop command (`use_ros2_control:=false`):

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 7) Typical workflow

1. Terminal A: run `launch_sim.launch.py`
2. Terminal B: optionally delete `~/.ros/rtabmap.db`
3. Terminal B: run `rtabmap_lidar.launch.py`
4. Terminal C: run Nav2 `navigation_launch.py use_sim_time:=true`
5. Terminal D: run stamped teleop command for `/diff_drive_controller/cmd_vel`

## 8) Check sensor fusion status (optional)

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic echo /odometry/filtered --once
```
