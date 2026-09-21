#!/usr/bin/env python3
"""
Timing synchronization analyzer for ROBIN reconstruction.

Reports encoder rate, lidar rate, per-point time ranges, and latency.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class TimingAnalyzer(Node):
    def __init__(self):
        super().__init__('timing_analyzer')

        self.encoder_sub = self.create_subscription(
            Float64, '/rotating_platform/angle',
            self.encoder_callback, 10)

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/velodyne_points',
            self.lidar_callback, SENSOR_QOS)

        self.last_encoder_time = None
        self.encoder_count = 0
        self.lidar_count = 0

        self.get_logger().info('Timing Analyzer started')

    def encoder_callback(self, msg):
        now = self.get_clock().now()
        if self.last_encoder_time is not None:
            dt = (now - self.last_encoder_time).nanoseconds / 1e6
            if self.encoder_count % 50 == 0:
                self.get_logger().info(
                    f'[ENCODER] dt: {dt:.2f}ms  rate: {1000.0/dt:.1f} Hz')
        self.last_encoder_time = now
        self.encoder_count += 1

    def lidar_callback(self, cloud_msg):
        cloud_time = rclpy.time.Time(
            seconds=cloud_msg.header.stamp.sec,
            nanoseconds=cloud_msg.header.stamp.nanosec,
            clock_type=self.get_clock().clock_type)
        now = self.get_clock().now()
        latency = (now - cloud_time).nanoseconds / 1e6

        dtype = pc2.dtype_from_fields(cloud_msg.fields, cloud_msg.point_step)
        pts = np.frombuffer(cloud_msg.data, dtype=dtype)
        if 'time' in pts.dtype.names and len(pts) > 0:
            t = pts['time']
            self.get_logger().info(
                f'[lidar] latency: {latency:.1f}ms  '
                f'time range: {t.min()*1000:.2f}..{t.max()*1000:.2f}ms  '
                f'encoder_age: '
                f'{(now - self.last_encoder_time).nanoseconds/1e6:.1f}ms'
                if self.last_encoder_time else '???')
        self.lidar_count += 1


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TimingAnalyzer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
