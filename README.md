# ros2_ws
Collection of packages of ROS packages for Gazebo worlds, robotic arms and autonomous cars.

Important commands:
`colcon build && source ~/.bashrc && ros2 launch table_tennis_gazebo simulation.launch.py`

`source ~/.bashrc && ros2 launch table_tennis_gazebo ball_spawn.launch.py x:=0.3 y:=0.5 z:=2.0`

```
ros2 action send_goal /red/arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7'],
    points: [
      {
        positions: [0.3, -0.3, 0.0, -1.2, 0.0, 1.0, 0.3],
        time_from_start: {sec: 2, nanosec: 0}
      }
    ]
  }
}" --feedback
```
