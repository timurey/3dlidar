#!/usr/bin/env python3
"""
Оценка точности и повторяемости 3D-реконструкции.

Метрики:
  1. Planar RMS  — отклонение точек пола/стены от лучшей плоскости (RANSAC).
                   Отражает комбинацию шума лидара + ошибки интерполяции угла.
  2. ICP residual — среднее расстояние между соответствующими точками двух
                   сканов одной сцены после выравнивания ICP (повторяемость).

Usage:
    python evaluate_accuracy.py [e57_dir]
    python evaluate_accuracy.py bag_1918a.e57 bag_1918b.e57   # только ICP между двумя
"""

import sys
import numpy as np
import open3d as o3d
from pathlib import Path


# ── параметры ──────────────────────────────────────────────────────────────
RANSAC_DIST   = 0.03      # м — порог RANSAC плоскости (3 см)
RANSAC_ITER   = 3000
MIN_PLANE_PTS = 5_000     # минимум точек на плоскость чтобы считать её надёжной
ICP_MAX_DIST  = 0.10      # м — максимальное соответствие при ICP
ICP_VOXEL     = 0.02      # м — вокселизация перед ICP (ускорение)
EVAL_VOXEL    = 0.005     # м — вокселизация для финальной оценки RMS плоскости


# ── загрузка E57 → open3d PointCloud ──────────────────────────────────────
def load_e57(path: Path) -> o3d.geometry.PointCloud:
    import pye57
    e57 = pye57.E57(str(path), mode='r')
    pts = []
    for i in range(e57.scan_count):
        raw = e57.read_scan_raw(i)
        x = np.asarray(raw['cartesianX'], dtype=np.float64)
        y = np.asarray(raw['cartesianY'], dtype=np.float64)
        z = np.asarray(raw['cartesianZ'], dtype=np.float64)
        pts.append(np.column_stack([x, y, z]))
    e57.close()
    pts = np.concatenate(pts)
    # убираем NaN/Inf
    mask = np.isfinite(pts).all(axis=1)
    pts = pts[mask]
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    return pcd


# ── поиск плоскостей RANSAC и оценка RMS ──────────────────────────────────
def find_planes(pcd: o3d.geometry.PointCloud, name: str):
    """
    Ищет до 3 крупных плоскостей (пол, стены) и считает RMS отклонения точек.
    Возвращает список (normal, d, rms, n_pts).
    """
    remaining = pcd.voxel_down_sample(EVAL_VOXEL)
    results = []

    for plane_idx in range(3):
        if len(remaining.points) < MIN_PLANE_PTS:
            break

        plane, inliers = remaining.segment_plane(
            distance_threshold=RANSAC_DIST,
            ransac_n=3,
            num_iterations=RANSAC_ITER,
        )
        a, b, c, d = plane
        normal = np.array([a, b, c])
        n_pts = len(inliers)

        if n_pts < MIN_PLANE_PTS:
            break

        pts_arr = np.asarray(remaining.points)[inliers]
        dist = pts_arr @ normal + d          # знаковое расстояние до плоскости
        rms = float(np.sqrt(np.mean(dist**2)))
        p95 = float(np.percentile(np.abs(dist), 95))

        # определяем тип плоскости по нормали
        nx, ny, nz = np.abs(normal)
        if nz > 0.7:
            plane_type = 'пол/потолок'
        elif nx > ny:
            plane_type = 'стена X'
        else:
            plane_type = 'стена Y'

        results.append({
            'type':  plane_type,
            'rms':   rms,
            'p95':   p95,
            'n_pts': n_pts,
            'normal': normal,
        })

        # убираем inliers — ищем следующую плоскость
        remaining = remaining.select_by_index(inliers, invert=True)

    return results


# ── ICP между двумя облаками ──────────────────────────────────────────────
def icp_repeatability(pcd_a: o3d.geometry.PointCloud,
                      pcd_b: o3d.geometry.PointCloud):
    """
    Выравниваем pcd_b → pcd_a через ICP.
    Возвращает (fitness, rms, transform).
    """
    a_ds = pcd_a.voxel_down_sample(ICP_VOXEL)
    b_ds = pcd_b.voxel_down_sample(ICP_VOXEL)

    a_ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    b_ds.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))

    # грубая инициализация — identity (сканы сняты с одной позиции)
    result = o3d.pipelines.registration.registration_icp(
        b_ds, a_ds,
        max_correspondence_distance=ICP_MAX_DIST,
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=100, relative_rmse=1e-6),
    )
    return result.fitness, result.inlier_rmse, result.transformation


# ── вывод таблицы ─────────────────────────────────────────────────────────
def print_plane_results(name: str, planes):
    print(f"\n  {'Плоскость':<16} {'Точек':>8}  {'RMS':>8}  {'P95':>8}")
    print(f"  {'-'*50}")
    for p in planes:
        print(f"  {p['type']:<16} {p['n_pts']:>8,}  {p['rms']*1000:>7.1f}мм  {p['p95']*1000:>7.1f}мм")
    if not planes:
        print("  (плоскостей не найдено)")


# ── main ──────────────────────────────────────────────────────────────────
def main():
    args = sys.argv[1:]

    # Определяем список файлов
    if not args:
        d = Path(__file__).parent
        files = sorted(d.glob('*.e57'))
    elif len(args) == 1 and Path(args[0]).is_dir():
        files = sorted(Path(args[0]).glob('*.e57'))
    else:
        files = [Path(a) for a in args]

    if not files:
        print("E57 файлы не найдены.")
        sys.exit(1)

    print(f"\n{'='*60}")
    print(f"  Оценка точности реконструкции VLP-16")
    print(f"{'='*60}")
    print(f"  Файлов: {len(files)}")
    print(f"  RANSAC порог: {RANSAC_DIST*1000:.0f} мм")
    print(f"  Вокселизация оценки: {EVAL_VOXEL*1000:.0f} мм")

    clouds = {}

    # ── 1. Planar RMS для каждого файла ───────────────────────────────────
    print(f"\n{'─'*60}")
    print("  1. ТОЧНОСТЬ СРЕЗА (planar RMS)")
    print(f"{'─'*60}")

    all_rms = []
    for f in files:
        print(f"\n  [{f.stem}]  загрузка...", end='', flush=True)
        pcd = load_e57(f)
        clouds[f.stem] = pcd
        n = len(pcd.points)
        print(f" {n:,} точек")

        planes = find_planes(pcd, f.stem)
        print_plane_results(f.stem, planes)

        for p in planes:
            all_rms.append(p['rms'])

    if all_rms:
        print(f"\n  Средний RMS по всем плоскостям: {np.mean(all_rms)*1000:.1f} мм")
        print(f"  Лучший RMS:                     {np.min(all_rms)*1000:.1f} мм")
        print(f"  Худший RMS:                     {np.max(all_rms)*1000:.1f} мм")

    # ── 2. ICP повторяемость между парами ─────────────────────────────────
    names = list(clouds.keys())
    if len(names) >= 2:
        print(f"\n{'─'*60}")
        print("  2. ПОВТОРЯЕМОСТЬ (ICP residual)")
        print(f"{'─'*60}")
        print(f"  Вокселизация ICP: {ICP_VOXEL*1000:.0f} мм, max_dist: {ICP_MAX_DIST*1000:.0f} мм\n")

        icp_results = []

        # сравниваем каждый с первым (базовый)
        base_name = names[0]
        base_pcd  = clouds[base_name]

        for other_name in names[1:]:
            other_pcd = clouds[other_name]
            print(f"  ICP  {other_name} → {base_name} ...", end='', flush=True)
            fitness, rmse, T = icp_repeatability(base_pcd, other_pcd)

            # извлекаем translation и rotation из матрицы трансформации
            t = T[:3, 3]
            R = T[:3, :3]
            angle_deg = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))

            print(f"  fitness={fitness:.3f}  RMSE={rmse*1000:.1f}мм  "
                  f"Δt=[{t[0]:.3f},{t[1]:.3f},{t[2]:.3f}]м  Δθ={angle_deg:.2f}°")
            icp_results.append(rmse)

        if icp_results:
            valid = [r for r in icp_results if r > 0]
            if valid:
                print(f"\n  Средний ICP RMSE: {np.mean(valid)*1000:.1f} мм")
                print(f"  Лучший:           {np.min(valid)*1000:.1f} мм")
                print(f"  Худший:           {np.max(valid)*1000:.1f} мм")

    # ── итог ──────────────────────────────────────────────────────────────
    print(f"\n{'='*60}")
    print("  ИТОГ")
    print(f"{'='*60}")
    print(f"  VLP-16 spec:         ±30 мм (дальность)")
    if all_rms:
        mean_rms = np.mean(all_rms) * 1000
        print(f"  Planar RMS (факт):   ±{mean_rms:.0f} мм  "
              f"({'хорошо' if mean_rms < 20 else 'приемлемо' if mean_rms < 40 else 'много — проверь интерполяцию угла'})")
    if 'icp_results' in dir() and icp_results:
        valid = [r for r in icp_results if r > 0]
        if valid:
            mean_icp = np.mean(valid) * 1000
            print(f"  ICP RMSE (повтор.):  ±{mean_icp:.0f} мм  "
                  f"({'хорошо' if mean_icp < 30 else 'приемлемо' if mean_icp < 60 else 'много — проверь наклон/калибровку'})")
    print()


if __name__ == '__main__':
    main()
