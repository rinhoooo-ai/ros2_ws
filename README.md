# Franka FR3 Pick-and-Place Simulation

Complete simulation with autonomous multi-block color sorting using Gazebo Harmonic + ROS 2 Jazzy.

## Demo

[![Franka FR3 Pick-and-Place Demo](https://img.youtube.com/vi/y0wR3j1XnKg/0.jpg)](https://youtu.be/y0wR3j1XnKg)

---

## Environment

| Component | Version / Detail |
|-----------|-----------------|
| OS | Ubuntu 24.04 (WSL2) |
| Middleware | ROS 2 Jazzy |
| Simulator | Gazebo Harmonic 8.10.0 |
| Robot | Franka FR3 7-DOF arm |
| IK Solver | ikpy (analytical + numerical) |
| GPU | Intel Arc |
| Packages | simple_pick_and_place_gazebo, _moveit, _perception |

---

## Quick Start

### 1. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch (5 terminals in order)

**T1 — Simulation**
```bash
ros2 launch simple_pick_and_place_gazebo simulation.launch.py
```

**T2 — Fix clock bridge**
```bash
pkill -f "parameter_bridge.*clock"
ros2 run ros_gz_bridge parameter_bridge \
  /world/pick_and_place/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
  --ros-args -r /world/pick_and_place/clock:=/clock
```

**T3 — Spawn blocks**
```bash
python3 ~/ros2_ws/src/simple_pick_and_place_gazebo/scripts/spawn_blocks.py
```

**T4 — Block detector**
```bash
ros2 run simple_pick_and_place_perception block_detector
```

**T5 — State machine**
```bash
python3 -u ~/ros2_ws/src/simple_pick_and_place_moveit/scripts/state_machine.py
```

---

## How It Works

The state machine autonomously sorts 4 colored blocks into their matching bins:

| Color | Block Spawn (x, y) | Bin Position (x, y) |
|-------|-------------------|---------------------|
| Red | (0.40, 0.00) | (0.212, -0.212) |
| Blue | (0.00, 0.40) | (-0.212, 0.212) |
| Green | (0.45, -0.10) | (-0.212, -0.212) |
| Yellow | (-0.10, -0.40) | (0.212, 0.212) |

### State Machine Loop

```
SCAN → detect block → PICK → PLACE → HOME → repeat
```

- **SCAN**: Rotates joint1 through 4 positions (0°, 90°, 180°, -90°) to find any unsorted block
- **PICK**: Pre-grasp → re-read pose → grasp → close gripper → lift
- **PLACE**: Move above correct bin → lower → release
- **HOME**: Return to home → start next scan
- **DONE**: Full scan with no blocks found → stop

---

## Verify Controllers

```bash
ros2 control list_controllers
```

Expected:
```
gripper_controller      position_controllers/GripperActionController           active
arm_controller          joint_trajectory_controller/JointTrajectoryController  active
joint_state_broadcaster joint_state_broadcaster/JointStateBroadcaster          active
```

---

## Manual Controls

**Move arm to home:**
```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: ['fr3_joint1','fr3_joint2','fr3_joint3','fr3_joint4','fr3_joint5','fr3_joint6','fr3_joint7'],
    points: [{positions: [0.0003, 0.0243, -0.0007, -0.1311, 0.0015, 0.4398, 0.0217], time_from_start: {sec: 3}}]}"
```

**Open gripper:**
```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 0.0}}"
```

**Close gripper:**
```bash
ros2 action send_goal /mnt/user-data/gripper_controller/gripper_cmd control_msgs/action/GripperCommand \
  "{command: {position: 0.02, max_effort: 15.0}}"
```

**Respawn blocks (without restarting Gazebo):**
```bash
for name in red_cuboid_1 blue_cuboid_1 green_cuboid_1 yellow_cuboid_1; do
  gz service -s /world/pick_and_place/remove \
    --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \
    --timeout 1000 --req "name: \"$name\" type: 2"
done
python3 ~/ros2_ws/src/simple_pick_and_place_gazebo/scripts/spawn_blocks.py
```

---

## Known Issues & Fixes

### 1. Xacro Boolean Evaluation Bug
**Problem:** Gazebo plugin not included despite `gazebo='true'` argument  
**Fix:** Convert args to properties in `franka_pick_and_place.urdf.xacro`:
```xml
<xacro:property name="use_gazebo" value="$(arg gazebo)"/>
<xacro:if value="${use_gazebo}">
  <xacro:gazebo_ros2_control/>
</xacro:if>
```

### 2. ABI Mismatch with gz_ros2_control
**Problem:** Controllers timeout with "No state interfaces found"  
**Fix:** Build gz_ros2_control from source:
```bash
cd src/
git clone https://github.com/ros-controls/gz_ros2_control.git -b jazzy
cd ..
colcon build --packages-up-to gz_ros2_control simple_pick_and_place_gazebo
```

### 3. Gripper Misalignment (joint7 Issue)
**Problem:** ikpy seed had joint7=0.785 rad (45°), gripper rotated 45° relative to block  
**Fix:** Changed `IK_SEED` joint7 from `0.785` → `0.0` and override joint7=0.0 after every IK solve

### 4. Block Out of Camera FOV
**Problem:** Blue/yellow blocks outside home position camera view  
**Fix:** Added 8-position scan loop rotating joint1 across full ±166° range

---

## Project Structure

```
ros2_ws/src/
├── simple_pick_and_place_gazebo/
│   ├── launch/simulation.launch.py
│   ├── models/
│   │   ├── red_cuboid/
│   │   ├── blue_cuboid/
│   │   ├── green_cuboid/
│   │   └── yellow_cuboid/
│   └── scripts/spawn_blocks.py
├── simple_pick_and_place_moveit/
│   └── scripts/state_machine.py
├── simple_pick_and_place_perception/
│   └── simple_pick_and_place_perception/block_detector.py
└── simple_pick_and_place_description/
    └── urdf/robot/franka_pick_and_place.urdf.xacro
```

---

## References
- [ROS 2 Jazzy Documentation](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic](https://gazebosim.org/docs/harmonic)
- [Franka Robotics](https://frankaemika.github.io/docs/)
- [ros2_control](https://control.ros.org/)
