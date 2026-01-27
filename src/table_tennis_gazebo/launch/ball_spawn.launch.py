#!/usr/bin/env python3
"""
Launch file to spawn a ball at specified coordinates using the spawn_ball action.
This acts as an action client that sends goals to the ball spawner action server.
"""

import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    x_arg = DeclareLaunchArgument('x', default_value='0.0',
                                   description='X coordinate for ball spawn')
    y_arg = DeclareLaunchArgument('y', default_value='0.0',
                                   description='Y coordinate for ball spawn')
    z_arg = DeclareLaunchArgument('z', default_value='1.0',
                                   description='Z coordinate for ball spawn (height)')

    # Ball spawn action client node
    ball_spawn_client = Node(
        package='table_tennis_gazebo',
        executable='ball_spawn_client.py',
        name='ball_spawn_client',
        output='screen',
        parameters=[{
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
        }]
    )

    return LaunchDescription([
        x_arg,
        y_arg,
        z_arg,
        ball_spawn_client,
    ])
