#!/usr/bin/env python3
"""
Pick and Place Gazebo Simulation Launch File

This launch file performs the following sequence:

1. Start Gazebo with a simple world (floor only)
2. Publish robot state (URDF via robot_state_publisher)
3. Spawn Franka FR3 robot with gripper
4. Spawn a cube object
5. Load ros2_control controllers
6. Launch RViz for visualization

Designed for ROS 2 Jazzy + Gazebo (ros_gz_sim).
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """
    Generate launch description with proper sequencing
    """
    
    # ==========================================================
    # Package directories
    # ==========================================================
    pkg_gazebo = get_package_share_directory('pick_n_place_gazebo')
    pkg_description = get_package_share_directory('pick_n_place_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_franka_description = get_package_share_directory('franka_description')
    
    # ==========================================================
    # File paths
    # ==========================================================
    world_file = os.path.join(pkg_gazebo, 'worlds', 'pick_place.world')
    robot_urdf_path = os.path.join(
        pkg_description,
        'urdf',
        'robots',
        'franka_with_gripper.urdf.xacro'
    )
    models_path = os.path.join(pkg_gazebo, 'models')
    rviz_config = os.path.join(pkg_gazebo, 'rviz', 'robot.rviz')
    
    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # ==========================================================
    # Gazebo Resource Path
    # Makes Gazebo aware of custom models + Franka meshes
    # ==========================================================
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=f'{models_path}:{pkg_franka_description}:{pkg_description}:{os.environ.get("GZ_SIM_RESOURCE_PATH", "")}'
    )
    
    # ==========================================================
    # Process Robot URDF via Xacro
    # Enables:
    #   - ros2_control
    #   - Gazebo plugins
    #   - Franka gripper (hand = true)
    # ==========================================================
    robot_description = xacro.process_file(
        robot_urdf_path,
        mappings={
            'robot_type': 'fr3',
            'hand': 'true',
            'ros2_control': 'true',
            'gazebo': 'true',
            'use_fake_hardware': 'false',
        }
    ).toxml()
    
    # ==========================================================
    # 1. Launch Gazebo
    # ==========================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'{world_file} -r'}.items(),
    )
    
    # ==========================================================
    # 2. Robot State Publisher
    # Publishes TF tree based on URDF
    # ==========================================================
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_description},
        ],
        output='screen'
    )
    
    delayed_robot_state_pub = TimerAction(
        period=3.0,
        actions=[robot_state_pub]
    )
    
    # ==========================================================
    # 3. Spawn Robot in Gazebo
    # ==========================================================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'fr3',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.76',
        ],
        output='screen'
    )
    
    delayed_spawn_robot = TimerAction(
        period=5.0,
        actions=[spawn_robot]
    )

    # ==========================================================
    # 4. Spawn Cube Object
    # ==========================================================
    spawn_cube = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'cube',
            '-file', os.path.join(models_path, 'cube', 'model.sdf'),
            '-x', '0.5',
            '-y', '0.0',
            '-z', '0.8',
        ],
        output='screen'
    )

    delayed_spawn_cube = TimerAction(
        period=6.0,
        actions=[spawn_cube]
    )
    
    # ==========================================================
    # 5. Load ros2_control Controllers
    # ==========================================================
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    delayed_joint_state = TimerAction(10.0, [joint_state_broadcaster])

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    delayed_arm = TimerAction(11.0, [arm_controller])

    gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'hand_controller',
            '--controller-manager', '/controller_manager'
        ],
        output='screen'
    )

    delayed_gripper = TimerAction(12.0, [gripper_controller])
    
    # ==========================================================
    # 6. Launch RViz
    # ==========================================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    delayed_rviz = TimerAction(10.0, [rviz])
    
    # ==========================================================
    # Launch Order Definition
    # ==========================================================
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true'
        ),

        # Set environment
        set_gz_resource_path,

        # 1. Start Gazebo first
        gazebo,

        # 2. Start robot state publishers (delayed)
        delayed_robot_state_pub,

        # 3. Spawn robots (delayed more)
        delayed_spawn_robot,

        # 4. Spawn cube
        delayed_spawn_cube,

        # 5. Load controller (delayed after spawn)
        delayed_joint_state,
        delayed_arm,
        delayed_gripper,

        # 6. Rviz windows (delayed)
        delayed_rviz,
    ])
