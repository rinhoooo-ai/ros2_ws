#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

class ArmController(Node):
    def __init__(self):
        super(ArmController, self).__init__(node_name='arm_controller')
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.get_logger().info('Action servers connected!')
        self.joint_names = ['link1_to_link2', 'link2_to_link3', 'link3_to_link4', 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange']
        target_pos = [1.345, -1.23, 0.264, -0.296, 0.389, -1.5]
        home_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.get_logger().info("Starting arm controller demo...")
        self.send_goal_request(target_pos)

    def create_goal_message(self, positions):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=2)
        goal_message = FollowJointTrajectory.Goal()
        goal_message.trajectory.joint_names = self.joint_names
        goal_message.trajectory.points = [point]
        return goal_message

    def send_goal_request(self, positions):
        goal_message = self.create_goal_message(positions)
        goal_future = self.arm_client.send_goal_async(goal_message)
        self.get_logger().info('Sent goal request. Waiting for response...')
        goal_future.add_done_callback(self.receive_goal_response_and_send_result_request)

    def receive_goal_response_and_send_result_request(self, future):
        goal_handle = future.result()
        self.get_logger().info(f'Received goal response: {goal_handle}.')
        if goal_handle.accepted:
            self.get_logger().info('Goal accepted. Sending result request...')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(self.receive_result_response)
        else:
            self.get_logger().info('Goal rejected.')

    def receive_result_response(self, future):
        result_handle = future.result()
        self.get_logger().info(f'Result received: {result_handle}')
        self.arm_client.destroy()

def main(args=None):
    rclpy.init(args=args)
    controller = ArmController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
