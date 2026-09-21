from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for ESP32 connection'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='115200',
            description='Serial baud rate'
        ),
        DeclareLaunchArgument(
            'use_serial',
            default_value='false',
            description='Enable serial communication (set to false for testing)'
        ),
        DeclareLaunchArgument(
            'bag_directory',
            default_value='/home/openclaw/bags',
            description='Directory to store rosbag recordings'
        ),

        Node(
            package='hmi_bridge',
            executable='hmi_bridge',
            name='hmi_bridge',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'use_serial': LaunchConfiguration('use_serial'),
                'bag_directory': LaunchConfiguration('bag_directory'),
            }]
        ),
    ])
