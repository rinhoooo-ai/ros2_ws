# Table Tennis Gazebo Simulation - Implementation Summary

## Successfully Implemented

### 1. Package Structure ✓
- **table_tennis_description**: URDF/xacro descriptions for robots, paddle, and cameras
- **table_tennis_gazebo**: Gazebo world, models, launch files, and ball spawner service

### 2. Robot Components ✓

#### Paddle End Effector
- Location: [urdf/end_effectors/paddle/paddle.urdf.xacro](src/table_tennis_description/urdf/end_effectors/paddle/paddle.urdf.xacro)
- Standard regulation dimensions (15.24cm × 15cm blade, 10cm handle)
- Mass: 0.09kg with proper inertial properties
- Red rubber visual with wood handle
- Gazebo contact properties (friction mu=0.8, high stiffness)

#### RealSense D435 Camera
- Location: [urdf/sensors/realsense_d435.urdf.xacro](src/table_tennis_description/urdf/sensors/realsense_d435.urdf.xacro)
- Complete frame tree: camera_link → depth_frame → depth_optical_frame, color_frame → color_optical_frame, infra frames
- Optical frames follow REP-103 convention (X-right, Y-down, Z-forward)
- Sensor specs: 640×480, 87° FOV, 0.3-10m range, 30Hz
- Publishes to standard RealSense topics

#### Robot URDF
- Location: [urdf/robots/franka_with_paddle.urdf.xacro](src/table_tennis_description/urdf/robots/franka_with_paddle.urdf.xacro)
- Extends Franka FR3 with paddle instead of gripper
- RealSense mounted on link7 angled toward table
- Parametrized for namespace, position, and orientation
- Includes ros2_control with franka_gz_ros2_control plugin

### 3. Gazebo Models ✓

#### Ball Model
- Location: [models/ball/](src/table_tennis_gazebo/models/ball/)
- Physics-accurate: 40mm diameter, 2.7g mass
- Inertia: 4.32e-7 kg⋅m² (sphere formula)
- Contact properties: restitution 0.89, friction 0.4, high stiffness (kp=1e6)
- Yellow/orange material with specular highlights
- Spawnable via service (not in world by default)

#### Table Tennis Table
- Location: [models/table/](src/table_tennis_gazebo/models/table/)
- Regulation dimensions: 2.74m × 1.525m × 0.76m height
- Blue playing surface with white boundary lines
- Dark gray net: 1.83m × 0.1525m height
- Four black legs at corners
- Contact properties: friction 0.35, restitution 0.75
- Static model centered at world origin

#### Fixed Cameras
- [models/overhead_camera/](src/table_tennis_gazebo/models/overhead_camera/): Birds-eye view
- [models/side_camera/](src/table_tennis_gazebo/models/side_camera/): Net-level side view
- Same RealSense specs as arm-mounted cameras
- Static placement via world file

### 4. Arena World ✓
- Location: [worlds/arena.world](src/table_tennis_gazebo/worlds/arena.world)
- Physics system: Dart engine, 0.001s timestep, real-time factor 1.0
- Sensors system: Ogre2 renderer for RGBD cameras
- UserCommands & SceneBroadcaster for entity spawning
- Overhead camera at (0, 0, 2.5) pitched -90°
- Side camera at (0, -1.5, 0.9) viewing along Y-axis
- Sun lighting with shadows
- Ground plane with bounce/friction properties

### 5. Controllers ✓
- Location: [config/controllers.yaml](src/table_tennis_gazebo/config/controllers.yaml)
- Controller manager: 100Hz update rate
- joint_state_broadcaster: Publishes joint states
- joint_trajectory_controller: Position control for 7 joints
- State interfaces: position, velocity
- Namespaced for /red and /green robots

### 6. Ball Spawner Service ✓
- Service definition: [srv/SpawnBall.srv](src/table_tennis_gazebo/srv/SpawnBall.srv)
- Node implementation: [src/ball_spawner_node.cpp](src/table_tennis_gazebo/src/ball_spawner_node.cpp)
- Input: geometry_msgs/Point position
- Output: bool success + string message
- Features:
  - Position validation (z > 0)
  - Generates unique ball names with timestamp
  - Embeds complete SDF with physics properties
  - Service: `/ball_spawner/spawn_ball`
  
**Note**: Currently uses placeholder for actual Gazebo entity creation. The service framework is ready but needs ros_gz_bridge mapping for gz.msgs.EntityFactory to be fully functional.

### 7. ROS-Gazebo Bridges ✓
- Location: [launch/bridge.launch.py](src/table_tennis_gazebo/launch/bridge.launch.py)
- **Clock synchronization**: /clock topic bridged
- **Camera bridges** (4 cameras × 3 topics each = 12 bridges):
  - Color images: ros_gz_image
  - Depth images: ros_gz_image  
  - Camera info: ros_gz_bridge
- **Topic mappings** to RealSense convention:
  - `/[namespace]/arm_cam/color/image_raw`
  - `/[namespace]/arm_cam/depth/image_rect_raw`
  - `/[namespace]/arm_cam/color/camera_info`
  - Same structure for /overhead_cam and /side_cam

### 8. Master Launch File ✓
- Location: [launch/simulation.launch.py](src/table_tennis_gazebo/launch/simulation.launch.py)
- **Environment**: Sets GZ_SIM_RESOURCE_PATH to models directory
- **Gazebo**: Launches gz_sim with arena.world
- **Red robot** (namespace: /red):
  - robot_state_publisher with processed URDF
  - Spawn at (-1.37, 0, 0.76) facing +X
  - Sequential controller loading after spawn
  - joint_state_publisher_gui for manual control
- **Green robot** (namespace: /green):
  - robot_state_publisher with processed URDF
  - Spawn at (1.37, 0, 0.76) rotated 180° facing -X
  - Sequential controller loading after spawn
  - joint_state_publisher_gui for manual control
- **Camera bridges**: Includes bridge.launch.py
- **Ball spawner**: Launches ball_spawner_node

### 9. Build System ✓
- Both packages compiled successfully
- Dependencies properly declared
- Symlink install for development
- Service interface code generation working

## Usage Instructions

### Build
```bash
cd ~/Projects/ros2_ws
colcon build --packages-select table_tennis_description table_tennis_gazebo --symlink-install
source install/setup.bash
```

### Run Simulation
```bash
ros2 launch table_tennis_gazebo simulation.launch.py
```

This will:
- Launch Gazebo with the table tennis arena
- Spawn two Franka FR3 robots with paddles at opposite ends
- Start all camera bridges
- Open two joint_state_publisher_gui windows for manual robot control
- Start the ball spawner service

### Spawn a Ball
```bash
ros2 service call /ball_spawner/spawn_ball table_tennis_gazebo/srv/SpawnBall "{position: {x: 0.0, y: 0.0, z: 1.0}}"
```

### View Camera Feeds
```bash
# Red robot camera
ros2 run image_view image_view --ros-args --remap image:=/red/arm_cam/color/image_raw

# Overhead camera
ros2 run image_view image_view --ros-args --remap image:=/overhead_cam/color/image_raw
```

### List Available Topics
```bash
ros2 topic list
```

Expected topics:
- `/clock`
- `/red/joint_states`, `/green/joint_states`
- `/red/arm_cam/*`, `/green/arm_cam/*`
- `/overhead_cam/*`, `/side_cam/*`
- `/red/controller_manager/*`, `/green/controller_manager/*`

## Known Limitations & Next Steps

1. **Ball spawner implementation**: Service definition complete but actual Gazebo entity creation needs proper ros_gz_bridge service mapping or gz::transport C++ API integration

2. **Controller tuning**: Default joint_trajectory_controller parameters may need tuning for fast table tennis movements

3. **Ball-paddle collision**: May need additional contact parameter tuning for realistic ball deflection

4. **Camera positioning**: Arm cameras may need fine-tuning of mount position/orientation for optimal table view

5. **Missing franka_gz_ros2_control**: The simulation requires this plugin which may not be built yet in the workspace

## Files Created

### table_tennis_description (6 files)
- package.xml, CMakeLists.txt
- urdf/robots/franka_with_paddle.urdf.xacro
- urdf/end_effectors/paddle/paddle.urdf.xacro
- urdf/sensors/realsense_d435.urdf.xacro
- README.md

### table_tennis_gazebo (19 files)
- package.xml, CMakeLists.txt
- worlds/arena.world
- models/ball/model.{sdf,config}
- models/table/model.{sdf,config}
- models/overhead_camera/model.{sdf,config}
- models/side_camera/model.{sdf,config}
- config/controllers.yaml
- srv/SpawnBall.srv
- src/ball_spawner_node.cpp
- launch/simulation.launch.py
- launch/bridge.launch.py
- README.md

**Total: 25 files across 2 packages**

## Robot Positioning Details

- **Red robot**: Base at (-1.37, 0, 0.76), 10cm behind left table edge (-1.27m), facing +X (right)
- **Green robot**: Base at (1.37, 0, 0.76), 10cm behind right table edge (1.27m), facing -X (left)
- **Table center**: (0, 0, 0.76), net along Y-axis at X=0
- **Overhead camera**: 1.74m above table center, pointing down
- **Side camera**: 1.5m from table edge along -Y, at table height, viewing across width

The simulation is ready for testing and further development of control algorithms!
