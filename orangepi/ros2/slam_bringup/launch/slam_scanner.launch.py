"""
SLAM Scanner bringup launch file.

Starts all nodes for the 3D LIDAR SLAM scanner:
  - IMU driver (Yahboom, /dev/myimu)
  - IMU Madgwick filter
  - Spin controller (RP2040, /dev/ttyS3)
  - HMI manager (CYD ESP32, /dev/ttyUSB0)
  - Velodyne VLP-16 driver (optional, enabled by arg)

Usage:
  ros2 launch slam_bringup slam_scanner.launch.py
  ros2 launch slam_bringup slam_scanner.launch.py with_velodyne:=true
  ros2 launch slam_bringup slam_scanner.launch.py target_rpm:=30.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ─────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument('with_velodyne',  default_value='false',
                              description='Launch Velodyne VLP-16 driver'),
        DeclareLaunchArgument('target_rpm',     default_value='40.0',
                              description='Platform rotation target RPM'),
        DeclareLaunchArgument('spin_port',      default_value='/dev/ttyS3',
                              description='Serial port for spin controller'),
        DeclareLaunchArgument('hmi_port',       default_value='/dev/ttyS4',
                              description='Serial port for CYD HMI'),
        DeclareLaunchArgument('imu_port',       default_value='/dev/myimu',
                              description='Serial port for IMU'),
        DeclareLaunchArgument('bag_dir',        default_value='/home/openclaw/bags',
                              description='Directory for bag recordings'),
        DeclareLaunchArgument('velodyne_ip',    default_value='192.168.100.201',
                              description='Velodyne device IP'),
    ]

    # ── IMU driver ────────────────────────────────────────────────────────────
    # 6-axis fusion (no mag): QUAT пакеты идут нормально, нет помех от мотора.
    # /imu/data_raw → /imu remap: matches PandarMapper topic convention.
    imu_driver = Node(
        package='imu_ros2_device',
        executable='ybimu_driver',
        name='ybimu_driver',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        remappings=[('/imu/data_raw', '/imu')],
    )

    # imu_tf_broadcaster публикует TF world → imu_link из /imu (6-axis quaternion).
    # Roll/pitch точны. Yaw дрейфует медленно (приемлемо для длительности одного скана).
    imu_tf_broadcaster = Node(
        package='slam_bringup',
        executable='imu_tf_broadcaster',
        name='imu_tf_broadcaster',
        output='screen',
        parameters=[{
            'parent_frame': 'world',
            'child_frame':  'imu_link',
        }],
    )

    # ── Spin controller ───────────────────────────────────────────────────────
    spin_controller = Node(
        package='spin_controller',
        executable='spin_controller_node',
        name='spin_controller',
        output='screen',
        parameters=[{
            'port':        LaunchConfiguration('spin_port'),
            'target_rpm':  LaunchConfiguration('target_rpm'),
            'auto_start':  False,
        }],
    )

    # ── HMI manager ───────────────────────────────────────────────────────────
    hmi_manager = Node(
        package='hmi_manager',
        executable='hmi_manager_node',
        name='hmi_manager',
        output='screen',
        parameters=[{
            'port':        LaunchConfiguration('hmi_port'),
            'bag_dir':     LaunchConfiguration('bag_dir'),
            'bag_topics':  '/velodyne_points /imu /imu/mag '
                           '/rotating_platform/angle '
                           '/rotating_platform/velocity '
                           '/rotating_platform/joint_state '
                           '/tf /tf_static',
        }],
    )

    # ── Velodyne (optional) ───────────────────────────────────────────────────
    velodyne = GroupAction(
        condition=IfCondition(LaunchConfiguration('with_velodyne')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('velodyne'),
                        'launch',
                        'velodyne-all-nodes-VLP16-launch.py',
                    ])
                ]),
                launch_arguments={
                    'device_ip':  LaunchConfiguration('velodyne_ip'),
                    'frame_id':   'velodyne',
                }.items(),
            )
        ],
    )

    # ── Static TF tree ────────────────────────────────────────────────────────
    #
    # Физическая схема (X=вверх во всех фреймах платформы):
    #
    #   world  (NWU: X=North, Y=West, Z=Up)
    #     └─ imu_link          ← Madgwick публикует динамически (ориентация из IMU)
    #          └─ platform_base  [0, 0, -0.025] — ось вращения на 25мм по -Z от IMU
    #               └─ platform_rotating  ← spin_controller (вращение вокруг X)
    #                    └─ velodyne  [0.165, 0, 0.0108] — 165мм вверх + 10.8мм эксцентриситет

    # imu_link → platform_base: ось вращения на 25мм ниже IMU (по -Z imu_link)
    tf_imu_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_imu_to_platform_base',
        arguments=['--x', '0', '--y', '0', '--z', '-0.025',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'imu_link',
                   '--child-frame-id', 'platform_base'],
    )

    # platform_rotating → velodyne:
    #   - 165мм вдоль оси вращения (X = вверх)
    #   - 10.8мм по Z (эксцентриситет: ось лидара смещена от оси вращения)
    #   - yaw=-1.6° (-0.02793 rad): calibrated 2026-09-21 via preview tuning;
    #     matches offline_deskew.py MOUNT_RPY_DEG[2]=-1.6
    tf_platform_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_platform_rotating_to_velodyne',
        arguments=['--x', '0.165', '--y', '0', '--z', '0.0108',
                   '--roll', '0', '--pitch', '0', '--yaw', '-0.02793',
                   '--frame-id', 'platform_rotating',
                   '--child-frame-id', 'velodyne'],
    )

    # ── Foxglove bridge ───────────────────────────────────────────────────────
    foxglove = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port':                  8765,
            'address':               '0.0.0.0',
            'tls':                   False,
            'topic_whitelist':       ['.*'],
            'service_whitelist':     ['.*'],
            'param_whitelist':       ['.*'],
            'num_threads':           4,
            'max_qos_depth':         10,
            'use_compression':       False,
            'capabilities':          ['clientPublish', 'parameters', 'parametersSubscribe',
                                      'services', 'connectionGraph', 'assets'],
        }],
    )

    return LaunchDescription(args + [
        tf_imu_to_base,
        tf_platform_velodyne,
        imu_driver,
        imu_tf_broadcaster,
        spin_controller,
        hmi_manager,
        velodyne,
        foxglove,
    ])
