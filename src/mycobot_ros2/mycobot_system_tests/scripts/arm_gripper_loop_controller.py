# """
# Control robot arm and gripper to perform repetitive movements between positions.

# This script creates a ROS 2 node that moves a robot arm between target and home positions, coordinating with gripper actions (open/close) at each position. The movement happens in a continuous loop.

# Action Clients:
#     /arm_controller/follow_joint_trajectory (control_msgs/FollowJointTrajectory): Commands for controlling arm joint positions
#     /gripper_action_controller/gripper_cmd (control_msgs/GripperCommand): Commands for opening and closing the gripper
# """

# import time
# import rclpy
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from control_msgs.action import FollowJointTrajectory, GripperCommand
# from trajectory_msgs.msg import JointTrajectoryPoint
# from builtin_interfaces.msg import Duration

# class ArmGripperLoopController(Node):
#     """
#     A ROS 2 node for controlling robot arm movements and gripper actions.

#     This class creates a simple control loop that:
#     1. Moves the arm to a target position
#     2. Closes the gripper
#     3. Returns the arm to home position
#     4. Opens the gripper
#     """

#     def __init__(self):
#         """
#         Initialize the node and set up action clients for arm and gripper control.

#         Sets up two action clients:
#         - One for controlling arm movements
#         - One for controlling gripper open/close actions
#         Also defines the positions for movement and starts a timer for the control loop.
#         """
#         super(ArmGripperLoopController, self).__init__(node_name='arm_gripper_loop_controller')

#         # Set up arm trajectory action client for arm movement control
#         self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')

#         # Set up gripper action client for gripper control
#         self.gripper_client = ActionClient(self, GripperCommand, '/gripper_action_controller/gripper_cmd')

#         # Wait for both action servers to be available
#         self.get_logger().info('Waiting for action servers...')
#         self.arm_client.wait_for_server()
#         self.gripper_client.wait_for_server()
#         self.get_logger().info('Action servers connected!')

#         # List of joint names for the robot arm
#         self.joint_names = ['link1_to_link2', 'link2_to_link3', 'link3_to_link4', 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange']

#         # Define target and home positions for the arm
#         self.target_pos = [1.345, -1.23, 0.264, -0.296, 0.389, -1.5]
#         self.home_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

#         # Create timer that triggers the control loop quickly after start (0.1 seconds)
#         self.create_timer(1, self.control_loop_callback)

#     def send_arm_command(self, positions: list) -> None:
#         """
#         Send a command to move the robot arm to specified joint positions.

#         Args:
#             positions (list): List of 6 joint angles in radians
#         """
#         # Create a trajectory point with the target positions
#         point = JointTrajectoryPoint()
#         point.positions = positions
#         point.time_from_start = Duration(sec=2)  # Allow 2 seconds for movement

#         # Create and send the goal message
#         goal_msg = FollowJointTrajectory.Goal()
#         goal_msg.trajectory.joint_names = self.joint_names
#         goal_msg.trajectory.points = [point]

#         self.arm_client.send_goal_async(goal_msg)

#     def send_gripper_command(self, position: float) -> None:
#         """
#         Send a command to the gripper to open or close.

#         Args:
#             position (float): Position value for gripper (0.0 for open, -0.7 for closed)
#         """
#         # Create and send the gripper command
#         goal_msg = GripperCommand.Goal()
#         goal_msg.command.position = position
#         goal_msg.command.max_effort = 5.0

#         self.gripper_client.send_goal_async(goal_msg)

#     def control_loop_callback(self) -> None:
#         """
#         Execute one cycle of the control loop.

#         This method performs the following sequence:
#         1. Move arm to target position
#         2. Pause at target
#         3. Close gripper
#         4. Move arm to home position
#         5. Pause at home
#         6. Open gripper
#         7. Pause before next cycle
#         """
#         # Move to target position
#         self.get_logger().info('Moving to target position')
#         self.send_arm_command(self.target_pos)
#         time.sleep(2.5)  # Wait for arm to reach target (2.5s)

#         # Pause at target position
#         self.get_logger().info('Reached target position - Pausing')
#         time.sleep(1.0)  # Pause for 1 second at target

#         # Close gripper
#         self.get_logger().info('Closing gripper')
#         self.send_gripper_command(-0.7)  # Close gripper
#         time.sleep(0.5)  # Wait for gripper to close

#         # Move to home position
#         self.get_logger().info('Moving to home position')
#         self.send_arm_command(self.home_pos)
#         time.sleep(2.5)  # Wait for arm to reach home (2.5s)

#         # Pause at home position
#         self.get_logger().info('Reached home position - Pausing')
#         time.sleep(1.0)  # Pause for 1 second at home

#         # Open gripper
#         self.get_logger().info('Opening gripper')
#         self.send_gripper_command(0.0)  # Open gripper
#         time.sleep(0.5)  # Wait for gripper to open

#         # Final pause before next cycle
#         time.sleep(1.0)


# def main(args=None):
#     """
#     Initialize and run the arm gripper control node.

#     Args:
#         args: Command-line arguments (default: None)
#     """
#     rclpy.init(args=args)
#     controller = ArmGripperLoopController()

#     try:
#         rclpy.spin(controller)
#     except KeyboardInterrupt:
#         controller.get_logger().info('Shutting down arm gripper controller...')
#     finally:
#         controller.destroy_node()
#         rclpy.shutdown()


# if __name__ == '__main__':
#     main()

"""
Control robot arm and gripper using asynchronous action callbacks.

Sequence:
1. Move Arm to Target -> Wait for result
2. Close Gripper -> Wait for result
3. Open Gripper -> Wait for result
4. Move Arm to Home -> Wait for result
5. Loop back to step 1
"""
    
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from action_msgs.msg import GoalStatus

class ArmGripperAsyncController(Node):
    def __init__(self):
        super(ArmGripperAsyncController, self).__init__(node_name='arm_gripper_async_controller')

        # 1. Initialize Action Clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_action_controller/gripper_cmd')

        self.get_logger().info('Waiting for action servers...')
        self.arm_client.wait_for_server()
        self.gripper_client.wait_for_server()
        self.get_logger().info('Action servers connected!')

        # 2. Define Positions
        self.joint_names = ['link1_to_link2', 'link2_to_link3', 'link3_to_link4', 'link4_to_link5', 'link5_to_link6', 'link6_to_link6_flange']
        self.target_pos = [1.345, -1.23, 0.264, -0.296, 0.389, -1.5]
        self.home_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # 3. Start the Cycle
        self.start_movement_cycle()

    def create_arm_message(self, positions, duration):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(sec=duration)
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [point]
        return goal_msg

    def create_gripper_message(self, position, max_effort):
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort
        return goal_msg

    def start_movement_cycle(self):
        """Entry point to start the sequence."""
        self.get_logger().info('--- Starting Cycle: Moving to Target ---')
        self.send_arm_goal(self.target_pos, self.on_target_reached)

    def send_arm_goal(self, positions, done_callback):
        """Sends an arm trajectory goal and assigns the specific result callback."""
        duration = 2
        goal_msg = self.create_arm_message(positions, duration)
        self.get_logger().info(f'Sending Arm Goal: {positions}')
        future = self.arm_client.send_goal_async(goal_msg)
        # Attach a partial callback so we know which function to call next
        future.add_done_callback(lambda f: self.goal_response_callback(f, done_callback))

    def send_gripper_goal(self, position, done_callback):
        """Sends a gripper goal and assigns the specific result callback."""
        max_effort = 5.0
        goal_msg = self.create_gripper_message(position, max_effort)
        self.get_logger().info(f'Sending Gripper Goal: {position}')
        future = self.gripper_client.send_goal_async(goal_msg)
        
        # Attach a partial callback so we know which function to call next
        future.add_done_callback(lambda f: self.goal_response_callback(f, done_callback))

    def goal_response_callback(self, future, next_step_callback):
        """Handles the server's acceptance/rejection of the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        # Pass the next step callback to the result handler
        get_result_future.add_done_callback(lambda f: self.get_result_callback(f, next_step_callback))

    def get_result_callback(self, future, next_step_callback):
        """Handles the final result (Success/Aborted/Canceled)."""
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Action Succeeded!')
            # Trigger the next step in the chain
            next_step_callback()
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Action Aborted.')
            # Logic to handle abortion (e.g., stop or retry) can go here
            self.start_movement_cycle() # Restarting for robustness
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Action Canceled.')
            self.start_movement_cycle() 
        else:
            self.get_logger().warn(f'Action finished with unknown status: {status}')

    # --- SEQUENCE LOGIC (THE STATE MACHINE) ---

    def on_target_reached(self):
        """Called when arm reaches target. Triggers Gripper Close."""
        self.get_logger().info('Arm at Target. Requesting Gripper CLOSE.')
        self.send_gripper_goal(-0.7, self.on_gripper_closed)

    def on_gripper_closed(self):
        """Called when gripper is closed. Triggers Gripper Open."""
        self.get_logger().info('Gripper Closed. Requesting Gripper OPEN.')
        self.send_gripper_goal(0.0, self.on_gripper_opened)

    def on_gripper_opened(self):
        """Called when gripper is open. Triggers Arm Home."""
        self.get_logger().info('Gripper Opened. Requesting Arm HOME.')
        self.send_arm_goal(self.home_pos, self.on_home_reached)

    def on_home_reached(self):
        """Called when arm reaches home. Restarts the cycle."""
        self.get_logger().info('Arm at Home. Cycle Complete. Restarting...')
        self.start_movement_cycle()

def main(args=None):
    rclpy.init(args=args)
    controller = ArmGripperAsyncController()
    # Spin strictly allows the callbacks to process
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
