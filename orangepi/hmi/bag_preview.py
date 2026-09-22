#!/usr/bin/env python3
"""
Downsampled point-cloud preview for the HMI web dashboard.

Reads a rosbag2 MCAP, deskews a bounded subset of clouds using
offline_deskew.py (the verified deskew implementation for this rig),
and returns a packed Float32 XYZ buffer for the 3-D viewer in the browser.

Bounded reading: stops after target_rotations of platform rotation so
preview stays fast on large bags — a static scan repeats the same
geometry every rotation, so a handful of them is representative.
"""

import os
import sys
import time

import numpy as np

# offline_deskew.py lives alongside this file in ~/hmi/
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from offline_deskew import (          # noqa: E402
    parse_pointcloud2,
    deskew_cloud,
    resolve_mcap_path,
)

TOPIC_POINTS      = '/velodyne_points'
# New spin_controller (deployed 2026-09-21) publishes to absolute /rotating_platform/* topics.
# The joint_state topic is preferred (header.stamp = encoder HW time, precise).
# The platform_angle topic is a Float64 fallback (uses log_time, jitter ~10–30 ms).
TOPIC_JOINT_STATE = '/rotating_platform/joint_state'
TOPIC_ANGLE       = '/rotating_platform/angle'
# IMU topics tried in order; first match with data wins
IMU_TOPICS = ['/imu/data', '/imu/data_raw', '/imu']

DEFAULT_MAX_CLOUDS       = 25
DEFAULT_MAX_POINTS       = 150_000
DEFAULT_TARGET_ROTATIONS = 2.5

HARD_MSG_CAP = 40_000
TWO_PI = 2 * np.pi

# Range filter: discard points closer than MIN_RANGE or farther than MAX_RANGE
MIN_RANGE = 0.3   # metres — VLP-16 blind zone
MAX_RANGE = 100.0


def gravity_rotation(g_vec: np.ndarray) -> np.ndarray:
    """Return 3×3 rotation matrix that maps g_vec (sensor 'up') to world Y (0,1,0).
    Output is in WebGL convention: Y = up.
    """
    g = g_vec / (np.linalg.norm(g_vec) + 1e-12)
    y = np.array([0.0, 1.0, 0.0])
    v = np.cross(g, y)
    s = np.linalg.norm(v)
    c = float(np.dot(g, y))
    if s < 1e-6:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / (s * s))


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    """Quaternion [x, y, z, w] → 3×3 rotation matrix (active, row-major)."""
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [  2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w)],
        [  2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def filter_points(xyz: np.ndarray) -> np.ndarray:
    """Remove NaN/Inf and out-of-range points from an (N, 3) float32 array."""
    if len(xyz) == 0:
        return xyz
    finite = np.isfinite(xyz).all(axis=1)
    r2 = (xyz[:, 0]**2 + xyz[:, 1]**2 + xyz[:, 2]**2)
    in_range = (r2 >= MIN_RANGE**2) & (r2 <= MAX_RANGE**2)
    return xyz[finite & in_range]


class _SpanTracker:
    """O(1)-per-sample running span of an unwrapped angle series."""
    def __init__(self):
        self._last_raw = None
        self._unwrapped = 0.0
        self._min = None
        self._max = None

    def push(self, raw_angle):
        if self._last_raw is None:
            self._unwrapped = raw_angle
        else:
            d = raw_angle - self._last_raw
            d = (d + np.pi) % TWO_PI - np.pi
            self._unwrapped += d
        self._last_raw = raw_angle
        u = self._unwrapped
        self._min = u if self._min is None else min(self._min, u)
        self._max = u if self._max is None else max(self._max, u)

    @property
    def span(self):
        return 0.0 if self._min is None else (self._max - self._min)


def _finalize_encoder_series(state_t, state_a, angle_t, angle_a):
    if len(state_t) >= 2:
        t, a = np.array(state_t, np.float64), np.array(state_a, np.float64)
        src = f'{TOPIC_JOINT_STATE} (jitter-free)'
    elif len(angle_t) >= 2:
        t, a = np.array(angle_t, np.float64), np.array(angle_a, np.float64)
        src = f'{TOPIC_ANGLE} (log_time - jittery)'
    else:
        return None, None, 'no usable encoder topic (need joint_state or angle, 2+ samples)'
    order = np.argsort(t)
    t, a = t[order], a[order]
    a = np.unwrap(a)
    return t, a, src


def build_bag_preview(bag_dir, max_clouds=DEFAULT_MAX_CLOUDS,
                      max_points=DEFAULT_MAX_POINTS,
                      target_rotations=DEFAULT_TARGET_ROTATIONS,
                      mount_pitch_deg=None, mount_yaw_deg=None,
                      angle_offset_deg=0.0, center_z=None):
    """Returns a dict with a raw Float32 xyz point buffer and preview stats.

    Raises FileNotFoundError / ValueError on bad input.
    """
    from mcap_ros2.reader import read_ros2_messages
    from offline_deskew import ROTATION_CENTER as _DEFAULT_CENTER, MOUNT_RPY_DEG as _DEFAULT_RPY

    t0 = time.monotonic()
    mcap_path = resolve_mcap_path(bag_dir)

    _mount_rpy = [
        0.0,
        float(mount_pitch_deg) if mount_pitch_deg is not None else _DEFAULT_RPY[1],
        float(mount_yaw_deg)   if mount_yaw_deg   is not None else _DEFAULT_RPY[2],
    ]
    _angle_offset = float(angle_offset_deg)
    _center = _DEFAULT_CENTER.copy()
    if center_z is not None:
        _center = _center.copy()
        _center[2] = float(center_z)
    target_span = target_rotations * TWO_PI

    state_t, state_a, angle_t, angle_a = [], [], [], []
    raw_clouds = []
    imu_accels = []
    tf_quats = []   # world→imu_link quaternions from /tf
    have_state = False
    state_tracker = _SpanTracker()
    angle_tracker = _SpanTracker()
    hard_cloud_cap = max(max_clouds * 20, 200)

    # Discover which IMU topic is present in this bag
    from mcap.reader import make_reader as _mcap_reader
    with open(str(mcap_path), 'rb') as _f:
        _topics_in_bag = {ch.topic for ch in _mcap_reader(_f).get_summary().channels.values()}
    imu_topic = next((t for t in IMU_TOPICS if t in _topics_in_bag), None)

    read_topics = [TOPIC_POINTS, TOPIC_JOINT_STATE, TOPIC_ANGLE]
    if imu_topic:
        read_topics.append(imu_topic)
    has_tf = '/tf' in _topics_in_bag
    if has_tf:
        read_topics.append('/tf')

    for seen, m in enumerate(read_ros2_messages(
            str(mcap_path),
            topics=read_topics), 1):
        topic = m.channel.topic
        if topic == TOPIC_JOINT_STATE:
            have_state = True
            h = m.ros_msg.header.stamp
            state_t.append(h.sec * 1_000_000_000 + h.nanosec)
            a = float(m.ros_msg.position[0])
            state_a.append(a)
            state_tracker.push(a)
        elif topic == TOPIC_ANGLE:
            angle_t.append(m.log_time_ns)
            a = np.deg2rad(float(m.ros_msg.data))
            angle_a.append(a)
            angle_tracker.push(a)
        elif imu_topic and topic == imu_topic:
            a = m.ros_msg.linear_acceleration
            imu_accels.append([a.x, a.y, a.z])
        elif has_tf and topic == '/tf':
            for tr in m.ros_msg.transforms:
                if tr.header.frame_id == 'world' and tr.child_frame_id == 'imu_link':
                    q = tr.transform.rotation
                    tf_quats.append([q.x, q.y, q.z, q.w])
        elif topic == TOPIC_POINTS:
            h = m.ros_msg.header.stamp
            raw_clouds.append((h.sec * 1_000_000_000 + h.nanosec, m.ros_msg))
            span = state_tracker.span if have_state else angle_tracker.span
            if span >= target_span and len(raw_clouds) > 0:
                break
            if len(raw_clouds) >= hard_cloud_cap:
                break

        if seen >= HARD_MSG_CAP:
            break

    rotations_captured = (state_tracker.span if have_state else angle_tracker.span) / TWO_PI

    angle_t_arr, angle_a_arr, enc_src = _finalize_encoder_series(
        state_t, state_a, angle_t, angle_a)
    no_encoder = angle_t_arr is None
    if no_encoder:
        enc_src = 'none (raw, no deskew)'

    clouds_seen = len(raw_clouds)
    stride = max(1, clouds_seen // max_clouds)
    selected = raw_clouds[::stride][:max_clouds]
    per_cloud_budget = max(1, max_points // max_clouds)

    blocks = []
    kept = low_cov = 0
    for cloud_ns, msg in selected:
        pts = parse_pointcloud2(msg)
        if len(pts) == 0:
            continue

        if no_encoder:
            # No encoder data — stack raw xyz without deskewing
            try:
                xyz = np.column_stack([
                    pts['x'].astype(np.float32),
                    pts['y'].astype(np.float32),
                    pts['z'].astype(np.float32),
                ])
            except (ValueError, KeyError):
                continue
            corrected = filter_points(xyz)
        else:
            clipped = np.clip(
                (pts['timestamp'].astype(np.float64) * 1e9
                 if 'timestamp' in pts.dtype.names
                 else np.full(len(pts), float(cloud_ns))),
                angle_t_arr[0], angle_t_arr[-1])
            covered = float(
                ((clipped >= angle_t_arr[0]) & (clipped <= angle_t_arr[-1])).mean())
            if covered < 0.99:
                low_cov += 1
            corrected = deskew_cloud(pts, cloud_ns, angle_t_arr, angle_a_arr,
                                    mount_rpy_deg=_mount_rpy,
                                    angle_offset_deg=_angle_offset,
                                    rotation_center=_center)
            corrected = filter_points(corrected.astype(np.float32))

        if len(corrected) == 0:
            continue

        if len(corrected) > per_cloud_budget:
            dec_stride = int(np.ceil(len(corrected) / per_cloud_budget))
            corrected = corrected[::dec_stride]

        blocks.append(corrected)
        kept += 1

    if not blocks:
        raise ValueError(
            'No usable point-cloud data in this bag (empty or unrecognised layout)')

    all_pts = np.concatenate(blocks, axis=0).astype(np.float32)

    # Orientation alignment: rotate point cloud to world frame (Z up)
    # Priority: /tf world→imu_link  >  IMU linear_acceleration fallback
    # Z-up (ROS/TF convention) → Y-up (WebGL convention): worldZ→Y, worldY→-Z
    _R_zup_to_yup = np.array([[1,0,0],[0,0,1],[0,-1,0]], dtype=np.float32)

    grav_info = 'none'
    if len(tf_quats) >= 1:
        # Average quaternion: simple mean + normalize (valid for small spread)
        q_arr = np.array(tf_quats)
        # Flip quats that differ in sign from the first (same rotation, opposite sign)
        q0 = q_arr[0]
        signs = np.sign(q_arr @ q0)
        q_arr *= signs[:, None]
        q_mean = q_arr.mean(axis=0)
        q_mean /= np.linalg.norm(q_mean)
        # quat_to_rot gives imu_link→world(Z=up); combine with Z→Y swap for WebGL
        R_tf = quat_to_rot(q_mean).astype(np.float32)
        R = _R_zup_to_yup @ R_tf
        all_pts = (all_pts @ R.T)
        grav_info = f'/tf world->imu_link n={len(tf_quats)} q=[{q_mean[0]:.3f},{q_mean[1]:.3f},{q_mean[2]:.3f},{q_mean[3]:.3f}]'
    elif len(imu_accels) >= 3:
        # Fallback: estimate orientation from static gravity vector
        g_mean = np.mean(imu_accels, axis=0)
        g_norm = np.linalg.norm(g_mean)
        if 0.5 < g_norm < 2.0:
            R = gravity_rotation(g_mean).astype(np.float32)
            all_pts = (all_pts @ R.T)
            tilt_deg = round(float(np.degrees(np.arccos(
                np.clip(np.dot(g_mean / g_norm, [0, 1, 0]), -1, 1)))), 2)
            grav_info = f'{imu_topic} accel fallback n={len(imu_accels)} tilt={tilt_deg}deg'

    return {
        'buf':                 all_pts.tobytes(),
        'points':              int(len(all_pts)),
        'clouds':              kept,
        'clouds_total':        clouds_seen,
        'low_coverage_clouds': low_cov,
        'encoder_source':      enc_src,
        'rotations_captured':  round(rotations_captured, 2),
        'compute_s':           round(time.monotonic() - t0, 2),
        'gravity_source':      grav_info,
    }
