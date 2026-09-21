#!/usr/bin/env python3
"""
Point cloud comparison diagnostic.

Subscribes to /velodyne_points and /velodyne_points_corrected and reports
how the correction transformed each cloud (rotation magnitude, axis, etc.).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np

SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class PointCloudComparator(Node):
    def __init__(self):
        super().__init__('point_cloud_comparator')
        self.original_cloud = None

        self.original_sub = self.create_subscription(
            PointCloud2, '/velodyne_points',
            self.original_callback, SENSOR_QOS)

        self.corrected_sub = self.create_subscription(
            PointCloud2, '/velodyne_points_corrected',
            self.corrected_callback, SENSOR_QOS)

        self.get_logger().info('Point Cloud Comparator started')

    def original_callback(self, msg):
        self.original_cloud = msg

    def corrected_callback(self, msg):
        orig = self.original_cloud
        if orig is None:
            return

        # Read both clouds as structured arrays (fast path, no Python iteration)
        dtype_o = pc2.dtype_from_fields(orig.fields, orig.point_step)
        dtype_c = pc2.dtype_from_fields(msg.fields, msg.point_step)
        op = np.frombuffer(orig.data, dtype=dtype_o)
        cp = np.frombuffer(msg.data, dtype=dtype_c)

        if len(op) == 0 or len(cp) == 0 or len(op) != len(cp):
            return

        ox, oy, oz = op['x'], op['y'], op['z']
        cx, cy, cz = cp['x'], cp['y'], cp['z']

        dx, dy, dz = cx - ox, cy - oy, cz - oz
        dist = np.sqrt(dx**2 + dy**2 + dz**2)
        mean_d = np.mean(dist)

        if mean_d < 0.001:
            self.get_logger().info(
                '[IDENTICAL] No correction applied', throttle_duration_sec=2.0)
        else:
            # Check if pure rotation (radial distance preserved)
            orig_r = np.sqrt(ox**2 + oy**2)
            corr_r = np.sqrt(cx**2 + cy**2)
            r_diff = np.mean(np.abs(corr_r - orig_r))

            info = (f'mean={mean_d*1000:.1f}mm  '
                    f'max={np.max(dist)*1000:.1f}mm  '
                    f'std={np.std(dist)*1000:.1f}mm')

            if r_diff < 0.01:
                theta = np.arctan2(cy, cx) - np.arctan2(oy, ox)
                info += f'  rot={np.rad2deg(np.mean(theta)):.2f} deg'

            z_diff = np.mean(np.abs(dz))
            if z_diff > 0.01:
                info += f'  Z_SHIFT={z_diff*1000:.1f}mm'

            self.get_logger().info(info, throttle_duration_sec=1.0)

        self.original_cloud = None


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PointCloudComparator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
