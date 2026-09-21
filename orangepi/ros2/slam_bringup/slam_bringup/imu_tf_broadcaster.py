#!/usr/bin/env python3
"""
Publishes TF world → imu_link from the Yahboom IMU's /imu topic.

The Yahboom firmware treats Z as the vertical axis (az≈1g when device is upright).
All platform TF frames (spin_controller, velodyne offset) use X as the rotation axis.
A corrective rotation of +90° around Y maps firmware-Z → imu_link-X so the TF chain
is consistent: imu_link-X = physical rotation axis = vertical.

Correction: q_imulink = q_firmware ⊗ q_Y_neg90
where q_Y_neg90 = (x=0, y=-sin(45°), z=0, w=cos(45°))
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

# -90° around Y: maps firmware-Z → imu_link-X  (R_y(-90°)*[1,0,0]=[0,0,1]=world-Z=up)
_CY = math.cos(math.pi / 4)  # cos(45°)
_SY = math.sin(math.pi / 4)  # sin(45°)
_QY90 = (0.0, -_SY, 0.0, _CY)  # (x, y, z, w)


def _qmul(q1, q2):
    """Quaternion multiply q1 ⊗ q2, both as (x, y, z, w)."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
    )


class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('imu_tf_broadcaster')
        self.declare_parameter('parent_frame', 'world')
        self.declare_parameter('child_frame',  'imu_link')

        self._parent = self.get_parameter('parent_frame').value
        self._child  = self.get_parameter('child_frame').value
        self._tf     = TransformBroadcaster(self)

        self.create_subscription(Imu, '/imu', self._cb, 10)
        self.get_logger().info(
            f'/imu quaternion + Y90 correction → TF {self._parent} → {self._child}'
        )

    def _cb(self, msg: Imu):
        q = msg.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            return

        qx, qy, qz, qw = _qmul((q.x, q.y, q.z, q.w), _QY90)

        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = self._parent
        t.child_frame_id  = self._child
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self._tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
