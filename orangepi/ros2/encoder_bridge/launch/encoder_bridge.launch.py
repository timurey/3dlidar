#!/usr/bin/env python3
"""
Launch file for encoder bridge node
Reads rotating platform encoder data from Teensy via UART4 (GPIO 12/13)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyAMA4',
        description='Serial port for Teensy communication (UART4 on GPIO 12/13)'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )

    reconnect_delay_arg = DeclareLaunchArgument(
        'reconnect_delay',
        default_value='5.0',
        description='Seconds to wait before attempting serial reconnect'
    )

    # Encoder node — respawn=True ensures the node restarts if it crashes
    encoder_node = Node(
        package='encoder_bridge',
        executable='encoder_node',
        name='encoder_node',
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'reconnect_delay': LaunchConfiguration('reconnect_delay'),
        }],
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        reconnect_delay_arg,
        encoder_node,
    ])
