#!/usr/bin/env python3
"""
Complete ROBIN-style reconstruction launch file

Launches:
1. Encoder bridge - reads platform angle from Teensy
2. Point cloud reconstructor - synchronizes and corrects point cloud for platform rotation

(The lidar driver itself is launched separately — see hesai_config/hesai_pandar40p.launch.py.)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    enable_correction_arg = DeclareLaunchArgument(
        'enable_correction',
        default_value='true',
        description='Enable platform rotation correction'
    )

    invert_rotation_arg = DeclareLaunchArgument(
        'invert_rotation',
        default_value='false',
        description='Invert rotation direction if reconstruction appears backwards'
    )
    use_per_point_time_arg = DeclareLaunchArgument(
        'use_per_point_time',
        default_value='true',
        description='Use per-point timing from the point cloud "time" field'
    )
    rotation_axis_arg = DeclareLaunchArgument(
        'rotation_axis',
        default_value='z',
        description='Platform rotation axis: x, y, or z'
    )
    mount_rpy_deg_arg = DeclareLaunchArgument(
        'mount_rpy_deg',
        default_value='[0.0, 0.0, 0.0]',
        description='LiDAR mount roll, pitch, yaw in degrees (YAML list)'
    )
    mount_axes_arg = DeclareLaunchArgument(
        'mount_axes',
        default_value="['+x', '+y', '+z']",
        description='LiDAR axes mapped into platform frame (YAML list)'
    )
    encoder_time_offset_arg = DeclareLaunchArgument(
        'encoder_time_offset_ms',
        default_value='0.0',
        description='Time offset to apply to encoder alignment (ms)'
    )
    rotation_center_arg = DeclareLaunchArgument(
        'rotation_center',
        default_value='[0.0, 0.0, 0.0]',
        description='Center of rotation in platform frame (meters)'
    )
    angle_offset_arg = DeclareLaunchArgument(
        'angle_offset_deg',
        default_value='0.0',
        description='Angle offset (deg) subtracted from encoder angle'
    )

    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Teensy encoder (USB serial — bypasses Pi 5 PL011 UART bug)'
    )

    max_rpm_arg = DeclareLaunchArgument(
        'max_rpm',
        default_value='80.0',
        description='Maximum RPM to reject encoder jumps beyond this rate'
    )

    auto_zero_on_start_arg = DeclareLaunchArgument(
        'auto_zero_on_start',
        default_value='false',
        description='Capture first encoder sample as zero reference'
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
            'max_rpm': LaunchConfiguration('max_rpm'),
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
            'encoder_state_topic': '/rotating_platform/joint_state',
            'use_encoder_state': True,
            'lidar_topic': '/velodyne_points',
            'output_topic': '/velodyne_points_corrected',
            'angle_buffer_size': 1000,
            'enable_correction': LaunchConfiguration('enable_correction'),
            'invert_rotation': LaunchConfiguration('invert_rotation'),
            'rotation_axis': LaunchConfiguration('rotation_axis'),
            'mount_rpy_deg': LaunchConfiguration('mount_rpy_deg'),
            'mount_axes': LaunchConfiguration('mount_axes'),
            'rotation_center': LaunchConfiguration('rotation_center'),
            'encoder_time_offset_ms': LaunchConfiguration('encoder_time_offset_ms'),
            'use_per_point_time': LaunchConfiguration('use_per_point_time'),
            'angle_offset_deg': LaunchConfiguration('angle_offset_deg'),
            'auto_zero_on_start': LaunchConfiguration('auto_zero_on_start'),
        }],
    )

    return LaunchDescription([
        enable_correction_arg,
        invert_rotation_arg,
        use_per_point_time_arg,
        rotation_axis_arg,
        mount_rpy_deg_arg,
        mount_axes_arg,
        encoder_time_offset_arg,
        rotation_center_arg,
        angle_offset_arg,
        serial_port_arg,
        max_rpm_arg,
        auto_zero_on_start_arg,
        encoder_node,
        # reconstructor_node,  # disabled - offline deskewing
    ])
