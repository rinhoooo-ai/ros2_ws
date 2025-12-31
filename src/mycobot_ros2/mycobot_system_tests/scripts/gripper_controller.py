#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus

class GripperController(Node):
    def __init__(self):
        super(GripperController, self).__init__(node_name='gripper_controller')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_action_controller/gripper_cmd')
        self.get_logger().info('Waiting for action server...')
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action server connected!')
        target_position = -0.7
        home_position = 0.0
        self.get_logger().info("Starting gripper controller demo...")
        self.send_goal_request(target_position)

    def create_goal_message(self, position):
        goal_message = GripperCommand.Goal()
        goal_message.command.position = position
        goal_message.command.max_effort = 5.0
        return goal_message

    def send_goal_request(self, position):
        goal_message = self.create_goal_message(position)
        goal_future = self.gripper_client.send_goal_async(goal_message)
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
        self.get_logger().info(f'Result received: {result_handle}.')
        self.gripper_client.destroy()

def main(args=None):
    rclpy.init(args=args)
    controller = GripperController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
