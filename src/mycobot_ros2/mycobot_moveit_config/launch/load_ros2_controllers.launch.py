import launch as l
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    """Generate a launch description for sequentially starting robot controllers.

    Returns:
        LaunchDescription: Launch description containing sequenced controller starts
    """
    # Start arm controller
    start_arm_controller_cmd = l.actions.ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'], output='screen')

    # Start gripper action controller
    start_gripper_action_controller_cmd = l.actions.ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_action_controller'], output='screen')

    # Launch joint state broadcaster
    start_joint_state_broadcaster_cmd = l.actions.ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'], output='screen')

    # Add delay to joint state broadcaster (if necessary)
    delayed_start = l.actions.TimerAction(period=10.0, actions=[start_joint_state_broadcaster_cmd])

    # Register event handlers for sequencing
    # Launch the joint state broadcaster after spawning the robot
    load_joint_state_broadcaster_cmd = l.actions.RegisterEventHandler(event_handler=OnProcessExit(target_action=start_joint_state_broadcaster_cmd, on_exit=[start_arm_controller_cmd]))

    # Launch the arm controller after launching the joint state broadcaster
    load_arm_controller_cmd = l.actions.RegisterEventHandler(event_handler=OnProcessExit(target_action=start_arm_controller_cmd, on_exit=[start_gripper_action_controller_cmd]))

    # Create the launch description and populate
    ld = l.LaunchDescription()

    # Add the actions to the launch description in sequence
    ld.add_action(delayed_start)
    ld.add_action(load_joint_state_broadcaster_cmd)
    ld.add_action(load_arm_controller_cmd)

    return ld
