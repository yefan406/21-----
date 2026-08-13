#!/usr/bin/env python3
"""3队轻量控制启动：编排器加避障控制。"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomy_core',
            executable='charlie_stage_orchestrator',
            name='charlie_stage_orchestrator',
            output='screen',
            parameters=[{
                'hard_deadline_sec': 180.0,
                'required_heading_rad': 5.2,
                'return_radius': 0.18,
            }],
        ),
        Node(
            package='autonomy_core',
            executable='hazard_handler',
            name='charlie_hazard_handler',
            output='screen',
        ),
    ])
