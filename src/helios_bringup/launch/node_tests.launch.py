#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='helios_navigation',
            executable='wheel_odom',
            name='wheel_odom',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'robot_namespace': '',
                'wheel_radius': 0.033,
                'wheel_separation': 0.160,
                'odom_frame': 'odom',
                'base_frame': 'base_link'
            }]
        )
    ])