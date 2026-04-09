import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient

def main():
    rclpy.init()
    node = Node('simple_pick_script')
    
    # Ở đây chúng ta sẽ dùng MoveGroupInterface qua lệnh CLI nhưng được bọc lại
    # Để đơn giản nhất cho Alex, hãy dùng lệnh 'set_start_state_to_current'
    
    print("--- ĐANG THỰC HIỆN PICK BLOCK ---")
    # Lệnh này ép MoveIt cập nhật trạng thái trước khi thực hiện
    import os
    
    # 1. Di chuyển tới Pre-grasp (Z=0.23)
    os.system('ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{request: {group_name: \'arm\', goal_constraints: [{position_constraints: [{header: {frame_id: \'world\'}, link_name: \'fr3_hand\', constraint_region: {primitive_poses: [{position: {x: 0.227, y: -0.230, z: 0.230}}], primitives: [{type: 1, dimensions: [0.05, 0.05, 0.05]}]}}]}], allowed_planning_time: 10.0, max_velocity_scaling_factor: 0.1}}"')

    # 2. Hạ xuống (Z=0.085)
    os.system('ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{request: {group_name: \'arm\', goal_constraints: [{position_constraints: [{header: {frame_id: \'world\'}, link_name: \'fr3_hand\', constraint_region: {primitive_poses: [{position: {x: 0.227, y: -0.230, z: 0.085}}], primitives: [{type: 1, dimensions: [0.02, 0.02, 0.02]}]}}]}], allowed_planning_time: 10.0, max_velocity_scaling_factor: 0.1}}"')

    # 3. Kẹp (Position = 0.0)
    os.system('ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"')

    # 4. Nhấc lên (Z=0.25)
    os.system('ros2 action send_goal /move_action moveit_msgs/action/MoveGroup "{request: {group_name: \'arm\', goal_constraints: [{position_constraints: [{header: {frame_id: \'world\'}, link_name: \'fr3_hand\', constraint_region: {primitive_poses: [{position: {x: 0.227, y: -0.230, z: 0.250}}], primitives: [{type: 1, dimensions: [0.05, 0.05, 0.05]}]}}]}], allowed_planning_time: 10.0, max_velocity_scaling_factor: 0.1}}"')

    print("--- HOÀN THÀNH ---")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()