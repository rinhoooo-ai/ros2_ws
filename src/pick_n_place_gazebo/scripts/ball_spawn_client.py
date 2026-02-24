#!/usr/bin/env python3
"""
Ball spawn action client that sends goals to the ball spawner action server.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from table_tennis_gazebo.action import SpawnBall


class BallSpawnClient(Node):
    def __init__(self):
        super().__init__('ball_spawn_client')
        
        # Declare parameters
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 1.0)
        
        # Get spawn coordinates
        self.x = self.get_parameter('x').value
        self.y = self.get_parameter('y').value
        self.z = self.get_parameter('z').value
        
        # Create action client
        self._action_client = ActionClient(self, SpawnBall, 'spawn_ball')
        
        self.get_logger().info('Waiting for ball spawner action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to action server')
        
        # Send goal
        self.send_goal()
    
    def send_goal(self):
        goal_msg = SpawnBall.Goal()
        goal_msg.x = self.x
        goal_msg.y = self.y
        goal_msg.z = self.z
        
        self.get_logger().info(f'Sending goal: spawn ball at [{self.x}, {self.y}, {self.z}]')
        
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted by action server, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            f'Feedback - Status: {feedback.status}, '
            f'Position: [{feedback.current_x:.3f}, {feedback.current_y:.3f}, {feedback.current_z:.3f}]'
        )
    
    def get_result_callback(self, future):
        result = future.result().result
        
        self.get_logger().info('=== Ball Action Result ===')
        self.get_logger().info(f'Status: {result.final_status}')
        self.get_logger().info(
            f'Final Position: [{result.final_x:.3f}, {result.final_y:.3f}, {result.final_z:.3f}]'
        )
        self.get_logger().info(f'Time Alive: {result.time_alive:.2f} seconds')
        self.get_logger().info('==========================')
        
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        client = BallSpawnClient()
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
