#!/usr/bin/env python3
"""
Point Cloud Reconstructor for ROBIN-style rotating platform LiDAR

Synchronizes lidar point cloud data with rotating platform encoder angles
to create dense 3D reconstructions by accounting for platform rotation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, JointState
from std_msgs.msg import Float64
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import traceback
import yaml


# Sensor-compatible QoS: matches typical spinning-lidar driver output
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
)


class PointCloudReconstructor(Node):
    def __init__(self):
        super().__init__('point_cloud_reconstructor')

        # ---------- Parameters ----------
        self.declare_parameter('encoder_topic', '/rotating_platform/angle')
        self.declare_parameter('encoder_state_topic', '/rotating_platform/joint_state')
        self.declare_parameter('use_encoder_state', True)
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_corrected')
        self.declare_parameter('angle_buffer_size', 1000)
        self.declare_parameter('enable_correction', True)
        self.declare_parameter('invert_rotation', False)
        self.declare_parameter('rotation_axis', 'z')
        self.declare_parameter('mount_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('mount_axes', ['+x', '+y', '+z'])
        self.declare_parameter('rotation_center', [0.0, 0.0, 0.0])
        self.declare_parameter('encoder_time_offset_ms', 0.0)
        self.declare_parameter('use_per_point_time', True)
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('auto_zero_on_start', False)

        encoder_topic = self.get_parameter('encoder_topic').value
        encoder_state_topic = self.get_parameter('encoder_state_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.buffer_size = self.get_parameter('angle_buffer_size').value
        self.enable_correction = self.get_parameter('enable_correction').value
        self.invert_rotation = self.get_parameter('invert_rotation').value
        self.use_encoder_state = self.get_parameter('use_encoder_state').value
        self.rotation_axis = self.get_parameter('rotation_axis').value.lower()
        self.mount_rpy_deg = self.get_parameter('mount_rpy_deg').value
        self.mount_axes = self.get_parameter('mount_axes').value
        self.rotation_center = self.get_parameter('rotation_center').value
        self.encoder_time_offset_ms = float(self.get_parameter('encoder_time_offset_ms').value)
        self.use_per_point_time = self.get_parameter('use_per_point_time').value
        self.angle_offset_deg = float(self.get_parameter('angle_offset_deg').value)
        self.auto_zero_on_start = self.get_parameter('auto_zero_on_start').value
        self.auto_offset_deg = None

        # Allow YAML list strings from launch args
        if isinstance(self.mount_rpy_deg, str):
            self.mount_rpy_deg = yaml.safe_load(self.mount_rpy_deg)
        if isinstance(self.mount_axes, str):
            self.mount_axes = yaml.safe_load(self.mount_axes)
        if isinstance(self.rotation_center, str):
            self.rotation_center = yaml.safe_load(self.rotation_center)

        # ---------- Angle ring buffer (pre-allocated numpy arrays) ----------
        self._buf_times = np.zeros(self.buffer_size, dtype=np.float64)
        self._buf_angles = np.zeros(self.buffer_size, dtype=np.float64)
        self._buf_len = 0
        self._buf_idx = 0  # next write position
        self.last_angle = 0.0

        # ---------- Precompute mount rotation ----------
        roll = np.deg2rad(float(self.mount_rpy_deg[0]))
        pitch = np.deg2rad(float(self.mount_rpy_deg[1]))
        yaw = np.deg2rad(float(self.mount_rpy_deg[2]))
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)
        self.mount_R = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr],
        ], dtype=np.float64)
        self.mount_R_inv = self.mount_R.T

        # Rotation center
        if not (isinstance(self.rotation_center, (list, tuple)) and len(self.rotation_center) == 3):
            self.rotation_center = [0.0, 0.0, 0.0]
        self.rotation_center = np.array(self.rotation_center, dtype=np.float64)

        # ---------- Precompute axis mapping (forward + inverse) ----------
        self._parse_axis_mapping()

        # ---------- Subscribers ----------
        if self.use_encoder_state:
            self.encoder_sub = self.create_subscription(
                JointState, encoder_state_topic,
                self.encoder_state_callback, 10)
        else:
            self.encoder_sub = self.create_subscription(
                Float64, encoder_topic,
                self.encoder_callback, 10)

        # CRITICAL: use BEST_EFFORT QoS to match the Velodyne driver's sensor QoS.
        # A RELIABLE subscriber cannot receive from a BEST_EFFORT publisher.
        self.lidar_sub = self.create_subscription(
            PointCloud2, lidar_topic,
            self.lidar_callback, SENSOR_QOS)

        # Publisher
        self.corrected_pub = self.create_publisher(
            PointCloud2, output_topic, SENSOR_QOS)

        # Statistics
        self.clouds_processed = 0
        self.points_processed = 0
        self.last_stats_time = self.get_clock().now()

        self.get_logger().info(
            f'Reconstructor ready | correction={self.enable_correction} '
            f'axis={self.rotation_axis} per_point={self.use_per_point_time} '
            f'offset={self.angle_offset_deg}deg '
            f'center={self.rotation_center.tolist()} '
            f'invert={self.invert_rotation}')

    # ------------------------------------------------------------------ #
    #  Axis mapping helpers
    # ------------------------------------------------------------------ #
    def _parse_axis_mapping(self):
        """Parse mount_axes strings and precompute forward/inverse index maps.

        Forward map:  mapped[i] = sign_fwd[i] * lidar[src_fwd[i]]
        Inverse map:  lidar[dst_inv[i]] = sign_inv[i] * mapped[i]

        Stored as integer index arrays for fast numpy fancy-indexing.
        """
        axis_index = {'x': 0, 'y': 1, 'z': 2}

        # Forward: mount_axes_map[i] = (axis_char, sign)
        fwd = []
        for axis_str in self.mount_axes:
            axis_str = str(axis_str).lower().strip()
            sign = -1.0 if axis_str.startswith('-') else 1.0
            key = axis_str.lstrip('+-')
            if key not in axis_index:
                self.get_logger().warn(f'Invalid mount axis "{axis_str}", using identity')
                fwd = [('x', 1.0), ('y', 1.0), ('z', 1.0)]
                break
            fwd.append((key, sign))
        if len(fwd) != 3:
            fwd = [('x', 1.0), ('y', 1.0), ('z', 1.0)]

        # Forward arrays: mapped[i] = sign_fwd[i] * xyz[src_fwd[i]]
        self._src_fwd = np.array([axis_index[a] for a, _ in fwd], dtype=np.intp)
        self._sign_fwd = np.array([s for _, s in fwd], dtype=np.float64)

        # Inverse arrays: xyz[dst] = sign * mapped[i]
        # Forward: mapped[i] = sign[i] * lidar[src[i]]
        # Inverse: lidar[src[i]] = sign[i] * mapped[i]   (since sign is ±1)
        self._dst_inv = np.array([axis_index[a] for a, _ in fwd], dtype=np.intp)
        self._sign_inv = self._sign_fwd.copy()

    def _apply_fwd_axis_map(self, x, y, z):
        """LiDAR xyz -> mapped axes (vectorized)."""
        xyz = (x, y, z)
        m0 = self._sign_fwd[0] * xyz[self._src_fwd[0]]
        m1 = self._sign_fwd[1] * xyz[self._src_fwd[1]]
        m2 = self._sign_fwd[2] * xyz[self._src_fwd[2]]
        return m0, m1, m2

    def _apply_inv_axis_map(self, m0, m1, m2):
        """Mapped axes -> LiDAR xyz (vectorized)."""
        mapped = (m0, m1, m2)
        # Pre-allocate with zeros matching the shape of the input
        out = [np.zeros_like(m0)] * 3
        for i in range(3):
            out[self._dst_inv[i]] = self._sign_inv[i] * mapped[i]
        return out[0], out[1], out[2]

    # ------------------------------------------------------------------ #
    #  Angle buffer
    # ------------------------------------------------------------------ #
    def _buf_append(self, time_ns: float, angle_rad: float):
        """Append to the ring buffer (O(1), no Python object allocation)."""
        idx = self._buf_idx
        self._buf_times[idx] = time_ns
        self._buf_angles[idx] = angle_rad
        self._buf_idx = (idx + 1) % self.buffer_size
        if self._buf_len < self.buffer_size:
            self._buf_len += 1

    def _buf_sorted(self):
        """Return (times, angles) arrays in chronological order."""
        n = self._buf_len
        if n == 0:
            return np.empty(0), np.empty(0)
        if n < self.buffer_size:
            return self._buf_times[:n].copy(), self._buf_angles[:n].copy()
        # Ring buffer wrapped: concatenate tail + head
        idx = self._buf_idx  # oldest element
        t = np.concatenate((self._buf_times[idx:], self._buf_times[:idx]))
        a = np.concatenate((self._buf_angles[idx:], self._buf_angles[:idx]))
        return t, a

    # ------------------------------------------------------------------ #
    #  Encoder callbacks
    # ------------------------------------------------------------------ #
    def encoder_callback(self, msg):
        angle_rad = np.deg2rad(msg.data)
        if self.auto_zero_on_start and self.auto_offset_deg is None:
            self.auto_offset_deg = msg.data
            self.get_logger().info(f'Auto zero offset: {self.auto_offset_deg:.2f} deg')
        self._buf_append(float(self.get_clock().now().nanoseconds), angle_rad)
        self.last_angle = angle_rad

    def encoder_state_callback(self, msg):
        if not msg.position:
            return
        stamp = msg.header.stamp
        time_ns = float(stamp.sec * 1_000_000_000 + stamp.nanosec)
        angle_rad = float(msg.position[0])
        if self.auto_zero_on_start and self.auto_offset_deg is None:
            self.auto_offset_deg = np.rad2deg(angle_rad)
            self.get_logger().info(f'Auto zero offset: {self.auto_offset_deg:.2f} deg')
        self._buf_append(time_ns, angle_rad)
        self.last_angle = angle_rad

    # ------------------------------------------------------------------ #
    #  LiDAR callback
    # ------------------------------------------------------------------ #
    def lidar_callback(self, cloud_msg):
        if not self.enable_correction:
            cloud_msg.header.stamp = self.get_clock().now().to_msg()
            self.corrected_pub.publish(cloud_msg)
            return

        if self._buf_len < 2:
            self.get_logger().warn(
                'Waiting for encoder data...', throttle_duration_sec=2.0)
            return

        try:
            self._process_cloud(cloud_msg)
        except Exception as e:
            self.get_logger().error(
                f'Error processing cloud: {e}\n{traceback.format_exc()}',
                throttle_duration_sec=2.0)

    def _process_cloud(self, cloud_msg):
        cloud_time_ns = float(
            cloud_msg.header.stamp.sec * 1_000_000_000 +
            cloud_msg.header.stamp.nanosec)

        # Parse point cloud into structured numpy array
        dtype = pc2.dtype_from_fields(cloud_msg.fields, cloud_msg.point_step)
        points = np.frombuffer(cloud_msg.data, dtype=dtype).copy()
        if len(points) == 0:
            return

        x = points['x']
        y = points['y']
        z = points['z']

        # Build sorted angle buffer snapshot
        buf_t, buf_a = self._buf_sorted()
        buf_a_unwrapped = np.unwrap(buf_a)

        # Compute per-point platform angles
        time_offset_ns = self.encoder_time_offset_ms * 1e6
        if self.use_per_point_time:
            if 'time' not in points.dtype.names:
                self.get_logger().error(
                    "Point cloud has no 'time' field - cannot do per-point "
                    "correction. Set use_per_point_time:=false or check your "
                    "Velodyne driver config.", throttle_duration_sec=5.0)
                return
            pt_ns = cloud_time_ns + (points['time'].astype(np.float64) * 1e9) + time_offset_ns
            pt_ns = np.clip(pt_ns, buf_t[0], buf_t[-1])
            platform_angles = np.interp(pt_ns, buf_t, buf_a_unwrapped)
        else:
            target_ns = np.clip(cloud_time_ns + time_offset_ns, buf_t[0], buf_t[-1])
            angle = np.interp(target_ns, buf_t, buf_a_unwrapped)
            platform_angles = np.full(len(points), angle)

        # Compute rotation angles with offset
        effective_offset_rad = np.deg2rad(
            self.angle_offset_deg + (self.auto_offset_deg or 0.0))
        if self.invert_rotation:
            rot = platform_angles - effective_offset_rad
        else:
            rot = -platform_angles - effective_offset_rad
        cos_a = np.cos(rot)
        sin_a = np.sin(rot)

        # Forward axis mapping: LiDAR -> platform frame
        mx, my, mz = self._apply_fwd_axis_map(x, y, z)

        # Mount rotation: LiDAR frame -> platform frame
        px = self.mount_R[0, 0] * mx + self.mount_R[0, 1] * my + self.mount_R[0, 2] * mz
        py = self.mount_R[1, 0] * mx + self.mount_R[1, 1] * my + self.mount_R[1, 2] * mz
        pz = self.mount_R[2, 0] * mx + self.mount_R[2, 1] * my + self.mount_R[2, 2] * mz

        # Translate to rotation center
        px -= self.rotation_center[0]
        py -= self.rotation_center[1]
        pz -= self.rotation_center[2]

        # Platform de-rotation
        if self.rotation_axis == 'z':
            rx = px * cos_a - py * sin_a
            ry = px * sin_a + py * cos_a
            rz = pz
        elif self.rotation_axis == 'x':
            rx = px
            ry = py * cos_a - pz * sin_a
            rz = py * sin_a + pz * cos_a
        elif self.rotation_axis == 'y':
            rx = px * cos_a + pz * sin_a
            ry = py
            rz = -px * sin_a + pz * cos_a
        else:
            self.get_logger().warn(
                f'Invalid rotation_axis "{self.rotation_axis}", using z',
                throttle_duration_sec=5.0)
            rx = px * cos_a - py * sin_a
            ry = px * sin_a + py * cos_a
            rz = pz

        # Translate back from rotation center
        rx += self.rotation_center[0]
        ry += self.rotation_center[1]
        rz += self.rotation_center[2]

        # Inverse mount rotation: platform frame -> LiDAR frame
        mx2 = self.mount_R_inv[0, 0] * rx + self.mount_R_inv[0, 1] * ry + self.mount_R_inv[0, 2] * rz
        my2 = self.mount_R_inv[1, 0] * rx + self.mount_R_inv[1, 1] * ry + self.mount_R_inv[1, 2] * rz
        mz2 = self.mount_R_inv[2, 0] * rx + self.mount_R_inv[2, 1] * ry + self.mount_R_inv[2, 2] * rz

        # Inverse axis mapping: platform frame -> LiDAR xyz
        xc, yc, zc = self._apply_inv_axis_map(mx2, my2, mz2)

        # Write corrected coordinates
        points['x'] = xc
        points['y'] = yc
        points['z'] = zc

        # Build output message (reuse all metadata from input)
        out = PointCloud2()
        out.header = cloud_msg.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.height = cloud_msg.height
        out.width = cloud_msg.width
        out.fields = cloud_msg.fields
        out.is_bigendian = cloud_msg.is_bigendian
        out.point_step = cloud_msg.point_step
        out.row_step = cloud_msg.row_step
        out.is_dense = cloud_msg.is_dense
        out.data = points.tobytes()
        self.corrected_pub.publish(out)

        # Stats
        self.clouds_processed += 1
        self.points_processed += len(points)
        now = self.get_clock().now()
        if (now - self.last_stats_time).nanoseconds > 5_000_000_000:
            angle_deg = np.rad2deg(np.mean(platform_angles))
            spread_deg = np.rad2deg(np.std(platform_angles))
            self.get_logger().info(
                f'{self.clouds_processed} clouds, {self.points_processed} pts | '
                f'buf={self._buf_len} | '
                f'angle={angle_deg:.1f} deg  spread={spread_deg:.3f} deg/cloud')
            self.last_stats_time = now


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PointCloudReconstructor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
