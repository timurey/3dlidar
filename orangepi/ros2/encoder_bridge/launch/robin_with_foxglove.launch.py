#!/usr/bin/env python3
"""
Complete ROBIN system with Foxglove visualization

Launches:
1. Encoder bridge - reads platform angle from Teensy
2. Point cloud reconstructor - corrects for rotation
3. Foxglove bridge - web-based visualization

Access via browser: https://app.foxglove.dev
Connect to: ws://<pi5-ip>:8765
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    enable_correction_arg = DeclareLaunchArgument(
        'enable_correction',
        default_value='true',
        description='Enable platform rotation correction'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA4',
        description='Serial port for Teensy encoder'
    )

    foxglove_port_arg = DeclareLaunchArgument(
        'foxglove_port',
        default_value='8765',
        description='Foxglove WebSocket port'
    )

    # Encoder bridge node — respawn keeps it alive through Pi 5 UART stalls
    encoder_node = Node(
        package='encoder_bridge',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': 230400,
        }],
    )

    # Point cloud reconstructor node
    reconstructor_node = Node(
        package='encoder_bridge',
        executable='point_cloud_reconstructor',
        name='point_cloud_reconstructor',
        output='screen',
        parameters=[{
            'encoder_topic': '/rotating_platform/angle',
            'lidar_topic': '/velodyne_points',
            'output_topic': '/velodyne_points_corrected',
            'angle_buffer_size': 1000,
            'enable_correction': LaunchConfiguration('enable_correction'),
        }],
    )

    # Foxglove bridge
    foxglove_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            ])
        ]),
        launch_arguments={
            'port': LaunchConfiguration('foxglove_port'),
        }.items()
    )

    return LaunchDescription([
        enable_correction_arg,
        serial_port_arg,
        foxglove_port_arg,
        encoder_node,
        reconstructor_node,
        foxglove_bridge,
    ])
