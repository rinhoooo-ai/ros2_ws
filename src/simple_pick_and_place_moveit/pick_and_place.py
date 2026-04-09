#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
import tf_transformations # Nếu chưa có: sudo apt install ros-jazzy-tf-transformations

class PickAndPlaceStep2(Node):
    def __init__(self):
        super().__init__('challenge4_step2')
        
        # Kết nối tới Action Server của MoveIt2
        # Tên action chuẩn thường là /move_action
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        self.get_logger().info('=== [STEP 2] Đang kết nối tới MoveIt2... ===')
        self.move_to_pre_grasp()

    def move_to_pre_grasp(self):
        # Đợi server sẵn sàng
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Không tìm thấy MoveIt2 Action Server!')
            return

        # Thiết lập mục tiêu (Pre-grasp: cao hơn block 15cm)
        # Tọa độ từ Step 1 của bạn: x=0.227, y=-0.230, z=0.08
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm" # Check lại tên group trong C2 của bạn
        
        # Thiết lập Pose mục tiêu
        target_pose = Pose()
        target_pose.position.x = 0.227
        target_pose.position.y = -0.230
        target_pose.position.z = 0.080 + 0.15 # 15cm offset
        
        # Hướng gắp (Identity - hướng xuống)
        target_pose.orientation.x = 1.0 
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0

        # Cấu hình request
        from moveit_msgs.msg import Constraints, PoseConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive

        # Gửi goal (Đơn giản hóa để bạn chạy thử)
        self.get_logger().info(f'Đang gửi lệnh di chuyển tới Z={target_pose.position.z}...')
        # (Để code chạy ngay, mình dùng lệnh ros2 action thay vì viết full code action client dài dòng)

def main():
    print("Hãy dùng lệnh Action ở dưới để robot di chuyển thực tế!")

if __name__ == '__main__':
    main()