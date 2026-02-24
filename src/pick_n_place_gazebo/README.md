# Table Tennis Gazebo

Gazebo simulation for table tennis playing robots using ROS 2 Jazzy.

## Features

- Dual Franka FR3 robots with paddle end effectors
- Physics-accurate table tennis ball (40mm, 2.7g, restitution 0.89)
- Standard table tennis table (2.74m × 1.525m × 0.76m)
- 4 RealSense D435 RGBD cameras:
  - 2 arm-mounted (one per robot)
  - 1 overhead camera
  - 1 side camera viewing along net
- Service-based ball spawning at any position
- Manual robot control via joint_state_publisher_gui

## Building

```bash
cd ~/Projects/ros2_ws
colcon build --packages-select table_tennis_description table_tennis_gazebo
source install/setup.bash
```

## Running the Simulation

```bash
ros2 launch table_tennis_gazebo simulation.launch.py
```

## Spawning a Ball

```bash
ros2 service call /ball_spawner/spawn_ball table_tennis_gazebo/srv/SpawnBall "{position: {x: 0.0, y: 0.0, z: 1.0}}"
```

## Camera Topics

### Red Robot
- `/red/arm_cam/color/image_raw`
- `/red/arm_cam/depth/image_rect_raw`
- `/red/arm_cam/color/camera_info`

### Green Robot
- `/green/arm_cam/color/image_raw`
- `/green/arm_cam/depth/image_rect_raw`
- `/green/arm_cam/color/camera_info`

### Overhead Camera
- `/overhead_cam/color/image_raw`
- `/overhead_cam/depth/image_rect_raw`
- `/overhead_cam/color/camera_info`

### Side Camera
- `/side_cam/color/image_raw`
- `/side_cam/depth/image_rect_raw`
- `/side_cam/color/camera_info`

## Robot Control

Two joint_state_publisher_gui windows will open for manual control of red and green robots.

## Package Structure

```
table_tennis_gazebo/
├── config/          # Controller configurations
├── launch/          # Launch files
├── models/          # Gazebo models (ball, table, cameras)
├── src/             # Source code (ball spawner node)
├── srv/             # Service definitions
└── worlds/          # Gazebo world files
```
