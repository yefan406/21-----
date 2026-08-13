import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('centerline_percept'), 'config',
        'charlie_channel_center.bin')

    centerline_detector_node = Node(
        package='centerline_percept',
        executable='centerline_detector',
        name='centerline_detector',
        output='screen',
        parameters=[
            {"sub_img_topic": "/nv12_img"},
            {"model_path": model_path},
            {"mode_name": "centerline"}
        ],
        arguments=['--ros-args', '--log-level', 'warn']
    )

    return LaunchDescription([centerline_detector_node])
