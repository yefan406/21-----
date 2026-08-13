#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# >> Team 3 — 智慧医疗服务机器人竞赛 全功能集成启动, 全部使用全新命名体系
"""
3队 — 智慧医疗服务机器人竞赛 主启动文件
所有节点均使用全新命名体系，与1队/2队/省赛完全不同。
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():
    """3队完整竞赛启动"""

    # ============ 参数声明 ============
    cruise_vel_arg = DeclareLaunchArgument('cruise_velocity', default_value='0.7')
    steer_kp_arg = DeclareLaunchArgument('steer_p_gain', default_value='0.0052')
    channel_vel_arg = DeclareLaunchArgument('channel_velocity', default_value='0.7')
    creep_vel_arg = DeclareLaunchArgument('creep_velocity', default_value='0.6')
    mission_timeout_arg = DeclareLaunchArgument('mission_timeout', default_value='180.0')
    stage1_timeout_arg = DeclareLaunchArgument('stage1_timeout', default_value='55.0')
    stage2_timeout_arg = DeclareLaunchArgument('stage2_timeout', default_value='105.0')
    dock_tol_arg = DeclareLaunchArgument('dock_tolerance', default_value='0.06')
    camera_dev_arg = DeclareLaunchArgument('camera_device', default_value='/dev/video2')
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyCH341USB1')
    web_preview_arg = DeclareLaunchArgument('web_preview', default_value='1')
    display_enable_arg = DeclareLaunchArgument('display_enable', default_value='true')

    # ============ 底盘驱动 ============
    medibot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('medibot_driver') +
            '/launch/base_serial.launch.py'))

    # ============ 摄像头 + 显示管线 ============
    rosbridge = ExecuteProcess(
        cmd=['ros2', 'launch', 'rosbridge_server',
             'rosbridge_websocket_launch.xml'], output='screen')

    usb_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_usb_cam') +
            '/launch/hobot_usb_cam.launch.py'),
        launch_arguments={
            'usb_image_width': '640',
            'usb_image_height': '480',
            'usb_video_device': LaunchConfiguration('camera_device'),
        }.items())

    nv12_decode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec') +
            '/launch/hobot_codec_decode.launch.py'),
        launch_arguments={
            'codec_channel': '1',
            'codec_in_format': 'jpeg', 'codec_out_format': 'nv12',
            'codec_in_mode': 'shared_mem', 'codec_out_mode': 'shared_mem',
            'codec_sub_topic': '/hbmem_img', 'codec_pub_topic': '/nv12_img',
        }.items())

    jpeg_encode_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('hobot_codec') +
            '/launch/hobot_codec_encode.launch.py'),
        launch_arguments={
            'codec_channel': '2', 'codec_jpg_quality': '70.0',
            'codec_in_format': 'nv12', 'codec_out_format': 'jpeg',
            'codec_in_mode': 'shared_mem', 'codec_out_mode': 'ros',
            'codec_sub_topic': '/nv12_img', 'codec_pub_topic': '/jpeg_img',
        }.items())

    web_preview_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('websocket') +
            '/launch/websocket.launch.py'),
        launch_arguments={
            'websocket_image_topic': '/jpeg_img',
            'websocket_image_type': 'mjpeg',
        }.items(),
        condition=IfCondition(
            PythonExpression(['"', LaunchConfiguration('web_preview'), '" == "1"'])))

    # ============ 车道引导 (统一包，4方向) ============
    # ============ 中线感知 ============
    centerline_node = Node(
        package='centerline_percept', executable='centerline_detector',
        name='centerline_detector', output='screen',
        parameters=[{
            'model_path': os.path.join(
                get_package_share_directory('centerline_percept'),
                'config', 'charlie_channel_center.bin'),
            'sub_img_topic': '/nv12_img',
            'mode_name': 'yellow',
        }],
        arguments=['--ros-args', '--log-level', 'warn'])

    # ============ 视觉监控 ============
    vision_guard_node = Node(
        package='vision_guard', executable='vision_guard',
        name='vision_guard', output='screen',
        parameters=[{
            'config_file': os.path.join(
                get_package_prefix('vision_guard'), 'lib', 'vision_guard',
                'config', 'guard_config_v2.json'),
            'sub_img_topic': '/nv12_img',
            'pub_ai_topic': '/vision_guard_result',
        }],
        arguments=['--ros-args', '--log-level', 'warn'])

    # ============ 码标识别 ============
    barcode_decoder_node = Node(
        package='barcode_intel', executable='barcode_decoder',
        name='barcode_decoder', output='screen',
        parameters=[{
            'input_topic': '/nv12_img',
            'code_output_topic': '/barcode_raw',
            'id_output_topic': '/barcode_id',
            'decode_timeout_ms': 200,
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    # ============ 自主驾驶核心 ============
    autonomy_master_node = Node(
        package='autonomy_core', executable='charlie_stage_orchestrator',
        name='charlie_stage_orchestrator', output='screen',
        parameters=[{
            'hard_deadline_sec': LaunchConfiguration('mission_timeout'),
            'required_heading_rad': 5.2,
            'return_radius': LaunchConfiguration('dock_tolerance'),
            'branch_duration': 1.05,
            'branch_rate': 0.70,
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    hazard_handler_node = Node(
        package='autonomy_core', executable='hazard_handler',
        name='hazard_handler', output='screen',
        parameters=[{
            'dock_stop': 415, 'barcode_stop': 170, 'barrier_stop': 148,
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    # ============ 速度仲裁 ============
    # ============ 音频系统 ============
    speech_synth_node = Node(
        package='speech_render', executable='speech_render_node',
        name='charlie_speech_renderer', output='screen',
        parameters=[{
            'playback_device': 'plughw:1,0',
            'volume_gain': 0.9,
            'topic_sub': '/speech_cmd',
            'common_chars_file': os.path.join(
                get_package_share_directory('speech_render'),
                'resources', 'common_3500_chars.txt'),
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    # ============ 场景理解 ============
    scene_interpreter_node = Node(
        package='scene_caption', executable='scene_interpreter',
        name='scene_interpreter', output='screen',
        parameters=[{
            'save_scene_picture': 'true',
            'scene_picture_dir': '/tmp/scene_captures',
            'margin_scene': 0.2, 'scene_jpeg_quality': 90,
            'volc_base_url': 'https://api.volcengine.com/vision/v1',
            'volc_model': 'doubao-vision-pro-32k',
            'vlm_timeout': 25.0, 'vlm_max_retries': 1,
            'scene_prompt': '观察图中出现的医用指示标牌人物，归纳其体态、着装及动作特征。',
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    display_terminal_node = Node(
        package='scene_caption', executable='display_terminal',
        name='display_terminal', output='screen',
        parameters=[{
            'display_text_topic': '/display_text',
            'scene_text_topic': '/scene_text',
            'scene_image_topic': '/scene_image_path',
        }],
        arguments=['--ros-args', '--log-level', 'info'])

    # ============ 组装 ============
    return LaunchDescription([
        cruise_vel_arg, steer_kp_arg, channel_vel_arg, creep_vel_arg,
        mission_timeout_arg, stage1_timeout_arg, stage2_timeout_arg,
        dock_tol_arg, camera_dev_arg, serial_port_arg,
        web_preview_arg, display_enable_arg,

        # 基础设施
        rosbridge, usb_cam_launch, nv12_decode_launch, jpeg_encode_launch,
        web_preview_launch,

        # 底盘
        medibot_launch,

        # 感知
        centerline_node,
        vision_guard_node,

        # 码标
        barcode_decoder_node,

        # 核心控制
        hazard_handler_node,
        autonomy_master_node,

        # 播报
        speech_synth_node,

        # 场景
        scene_interpreter_node, display_terminal_node,
    ])
