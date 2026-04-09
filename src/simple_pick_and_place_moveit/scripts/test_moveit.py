#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, WorkspaceParameters, Constraints, JointConstraint
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Vector3

class MoveItTest(Node):
    def __init__(self):
        super().__init__('test_moveit')
        self._client = ActionClient(self, MoveGroup, '/move_action')

    def move_to_joints(self, joint_values):
        self.get_logger().info('Waiting for MoveGroup action server...')
        self._client.wait_for_server()

        goal = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = 'fr3_arm'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        joint_names = [
            'fr3_joint1', 'fr3_joint2', 'fr3_joint3', 'fr3_joint4',
            'fr3_joint5', 'fr3_joint6', 'fr3_joint7'
        ]

        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        req.goal_constraints.append(constraints)
        goal.request = req
        goal.planning_options.plan_only = False

        self.get_logger().info('Sending goal...')
        future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted — waiting for result')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        self.get_logger().info(f'Done: error_code={result_future.result().result.error_code.val}')

def main():
    rclpy.init()
    node = MoveItTest()
    home = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
    node.move_to_joints(home)
    rclpy.shutdown()

if __name__ == '__main__':
    main()