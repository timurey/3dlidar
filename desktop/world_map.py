#!/usr/bin/env python3
"""
Построение 3D карты мира из bag-файла.

Для каждого фрейма лидара:
  1. Deskew — коррекция внутри фрейма (per-point time × скорость платформы)
  2. World rotation — поворот фрейма в мировые координаты по углу платформы

Формула Rx (поворот вокруг X, z→-z для правильного "вверх"):
  x_out = x
  y_out =  y·cos(θ) - z·sin(θ)
  z_out = -(y·sin(θ) + z·cos(θ))

Usage:
    python world_map.py <bag_dir> [-o output.e57] [--voxel 0.01]
"""

import sys
import argparse
import numpy as np
from pathlib import Path

from offline_deskew import (
    read_bag, parse_pointcloud2, deskew_cloud, resolve_mcap_path,
    ROTATION_AXIS, ANGLE_OFFSET_DEG, ROTATION_CENTER,
    INVERT_ROTATION, ENCODER_TIME_OFFSET_MS,
    MOUNT_RPY_DEG, MOUNT_AXES,
)


def world_rotate_x(pts, theta):
    """Поворот вокруг X на угол theta, z → -z (правильный 'вверх')."""
    c, s = np.cos(theta), np.sin(theta)
    x = pts[:, 0].copy()
    y =  pts[:, 1] * c - pts[:, 2] * s
    z = -(pts[:, 1] * s + pts[:, 2] * c)
    return np.column_stack([x, y, z])


def read_imu_tilt(bag_path):
    """
    Читает /imu (или /imu/data_raw), усредняет accel → вычисляет матрицу коррекции наклона.

    IMU смонтирован X-вверх: вектор гравитации в сенсоре ≈ [+g, 0, 0].
    Если основание платформы не горизонтально, ось вращения (sensor-Z = world-X)
    будет слегка наклонена → воронка в реконструкции.
    Коррекция: поворот всего облака так, чтобы измеренная гравитация совпала
    с ожидаемой (purely along +sensor-X).
    """
    from mcap_ros2.reader import read_ros2_messages
    mcap = resolve_mcap_path(bag_path)
    samples = []
    for msg in read_ros2_messages(str(mcap)):
        if msg.channel.topic == '/imu':
            a = msg.ros_msg.linear_acceleration
            samples.append([a.x, a.y, a.z])
            if len(samples) >= 500:
                break

    if len(samples) < 10:
        print("  WARNING: нет данных /imu — коррекция наклона отключена")
        return np.eye(3), 0.0

    g_vec = np.mean(samples, axis=0)
    g_norm = np.linalg.norm(g_vec)
    g_unit = g_vec / g_norm

    # Ожидаемая гравитация в сенсоре (X-вверх → g должна быть [+1, 0, 0])
    g_expected = np.array([1.0, 0.0, 0.0])

    # Угол и ось поворота: g_unit → g_expected
    cos_a = np.clip(np.dot(g_unit, g_expected), -1.0, 1.0)
    tilt_deg = np.degrees(np.arccos(cos_a))

    if tilt_deg < 0.1:
        return np.eye(3), 0.0

    axis = np.cross(g_unit, g_expected)
    axis_norm = np.linalg.norm(axis)
    if axis_norm < 1e-9:
        return np.eye(3), 0.0
    axis /= axis_norm

    # Матрица поворота Родрига: axis, angle=tilt_deg
    a = np.radians(tilt_deg)
    c, s = np.cos(a), np.sin(a)
    t = 1 - c
    x, y, z = axis
    R = np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])
    print(f"  Наклон платформы: {tilt_deg:.2f}°  ось={np.round(axis, 3)}")
    return R, tilt_deg


def voxel_downsample(pts, voxel):
    """Простая вокселизация через numpy (без open3d зависимости)."""
    if voxel <= 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int32)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def save_e57(pts, path):
    import pye57
    e57 = pye57.E57(str(path), mode='w')
    data = {
        'cartesianX': pts[:, 0].astype(np.float64),
        'cartesianY': pts[:, 1].astype(np.float64),
        'cartesianZ': pts[:, 2].astype(np.float64),
    }
    e57.write_scan_raw(data)
    e57.close()
    print(f"E57  → {path}  ({len(pts):,} pts)")


def save_pcd(pts, path):
    """Binary PCD (float32 xyz)."""
    pts32 = pts.astype(np.float32)
    header = (
        f"VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n"
        f"COUNT 1 1 1\nWIDTH {len(pts32)}\nHEIGHT 1\n"
        f"VIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(pts32)}\nDATA binary\n"
    ).encode()
    with open(path, 'wb') as f:
        f.write(header)
        f.write(pts32.tobytes())
    print(f"PCD  → {path}  ({len(pts32):,} pts)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('bag_path')
    parser.add_argument('-o', '--output', default=None,
                        help='Output file (.e57 or .pcd, default: <bag_name>.e57)')
    parser.add_argument('--voxel', type=float, default=0.01,
                        help='Voxel size for downsampling (m), 0=off (default: 0.01)')
    parser.add_argument('--max-range', type=float, default=20.0)
    parser.add_argument('--min-range', type=float, default=0.3)
    args = parser.parse_args()

    bag_path = Path(args.bag_path)
    if args.output:
        out_path = Path(args.output)
    else:
        out_path = Path(bag_path.name).with_suffix('.e57')

    # ── Читаем bag ────────────────────────────────────────────────
    angle_times, angle_values, clouds = read_bag(bag_path)

    if len(angle_times) < 2:
        print("ERROR: недостаточно данных энкодера")
        sys.exit(1)

    angles_unwrapped = np.unwrap(angle_values)

    # ── Коррекция наклона платформы из акселерометра ──────────────
    print("\nЧитаем IMU для коррекции наклона...")
    R_tilt, tilt_deg = read_imu_tilt(bag_path)
    use_tilt = tilt_deg > 0.1

    # ── Обрабатываем каждый фрейм ─────────────────────────────────
    print(f"\nОбработка {len(clouds)} фреймов...")
    all_pts = []

    for i, (cloud_time_ns, cloud_msg) in enumerate(clouds):
        pts = parse_pointcloud2(cloud_msg)
        if len(pts) == 0:
            continue

        # Угол платформы в момент фрейма
        t_clip = np.clip(cloud_time_ns, angle_times[0], angle_times[-1])
        theta = np.interp(t_clip, angle_times, angles_unwrapped)

        # Пропускаем кадры с малым угловым перекрытием (плохой дескью):
        # проверяем, что за 100мс перед фреймом есть достаточно энкодерных точек
        t_start = cloud_time_ns - 110_000_000  # 110ms назад
        enc_in_window = np.sum((angle_times >= t_start) & (angle_times <= cloud_time_ns))
        if enc_in_window < 5:
            continue

        # 1. Deskew (коррекция внутри фрейма)
        corrected = deskew_cloud(
            pts, cloud_time_ns, angle_times, angle_values,
            rotation_axis=ROTATION_AXIS,
            angle_offset_deg=ANGLE_OFFSET_DEG,
            rotation_center=ROTATION_CENTER,
            invert_rotation=INVERT_ROTATION,
            encoder_time_offset_ms=ENCODER_TIME_OFFSET_MS,
            mount_rpy_deg=MOUNT_RPY_DEG,
            mount_axes=MOUNT_AXES,
        )

        # Фильтрация по дальности
        r = np.sqrt(corrected[:, 0]**2 + corrected[:, 1]**2 + corrected[:, 2]**2)
        mask = (r >= args.min_range) & (r <= args.max_range)
        corrected = corrected[mask]
        if len(corrected) == 0:
            continue

        # Deskew уже поместил точки в мировые координаты (θ=0 референс).
        world_pts = corrected

        # 2. Коррекция наклона платформы (из акселерометра)
        if use_tilt:
            world_pts = (R_tilt @ world_pts.T).T

        # Flip не нужен: VLP-16 крутится в правильном направлении (+RPM)

        all_pts.append(world_pts.astype(np.float32))

        if (i + 1) % 50 == 0:
            print(f"  {i+1}/{len(clouds)}")

    if not all_pts:
        print("ERROR: нет точек после обработки")
        sys.exit(1)

    pts_all = np.vstack(all_pts)
    print(f"\nВсего точек: {len(pts_all):,}")

    # ── Вокселизация ──────────────────────────────────────────────
    if args.voxel > 0:
        pts_all = voxel_downsample(pts_all, args.voxel)
        print(f"После вокселизации ({args.voxel}m): {len(pts_all):,}")

    # ── Сохраняем ────────────────────────────────────────────────
    suffix = out_path.suffix.lower()
    if suffix == '.e57':
        save_e57(pts_all, out_path)
    else:
        save_pcd(pts_all, out_path)

    print(f"\nГотово. Открыть:\n  .venv/bin/python flythrough.py {out_path}")


if __name__ == '__main__':
    main()
