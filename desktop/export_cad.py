#!/usr/bin/env python3
"""
Export PCD point cloud to CAD-compatible formats:
  E57  → Autodesk ReCap → AutoCAD / Revit / Fusion 360
  LAS  → Autodesk ReCap → AutoCAD / Revit / Fusion 360
  XYZ  → Autodesk ReCap (ASCII import)
  PTS  → AutoCAD / Leica (ASCII)

Usage:
    python export_cad.py input.pcd [--format e57|las|xyz|pts|all] [--out output]
"""

import sys, argparse
import numpy as np
from pathlib import Path


def read_pcd(path):
    with open(path, 'rb') as f:
        header = {}
        while True:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            if line == 'DATA binary':
                break
            if line == 'DATA ascii':
                header['ascii'] = True
                break
            if ' ' in line:
                k, v = line.split(None, 1)
                header[k] = v
        if header.get('ascii'):
            data = np.loadtxt(f)
            return data[:, :3].astype(np.float32)
        raw = f.read()
    n = int(header.get('POINTS', 0))
    pts = np.frombuffer(raw, dtype=np.float32).reshape(n, -1)
    return pts[:, :3]


def export_xyz(pts, out_path):
    """ASCII XYZ — works in ReCap, FreeCAD, CloudCompare."""
    np.savetxt(out_path, pts, fmt='%.4f', delimiter=' ')
    print(f"XYZ  → {out_path}  ({len(pts):,} pts)")


def export_pts(pts, out_path):
    """PTS (Leica/AutoCAD ASCII) — first line is point count."""
    with open(out_path, 'w') as f:
        f.write(f"{len(pts)}\n")
        for x, y, z in pts:
            f.write(f"{x:.4f} {y:.4f} {z:.4f}\n")
    print(f"PTS  → {out_path}  ({len(pts):,} pts)")


def export_e57(pts, out_path):
    """E57 — standard for ReCap, Revit, AutoCAD, Bentley."""
    import pye57
    e57 = pye57.E57(str(out_path), mode='w')
    data = {
        'cartesianX': pts[:, 0].astype(np.float64),
        'cartesianY': pts[:, 1].astype(np.float64),
        'cartesianZ': pts[:, 2].astype(np.float64),
    }
    e57.write_scan_raw(data)
    e57.close()
    print(f"E57  → {out_path}  ({len(pts):,} pts)")


def export_las(pts, out_path):
    """LAS 1.4 — works in ReCap, Civil 3D, Trimble."""
    import laspy
    header = laspy.LasHeader(point_format=0, version="1.4")
    # Scale: 1mm precision, offset at centroid
    offset = pts.mean(axis=0).astype(np.float64)
    scale = 0.001
    header.offsets = offset
    header.scales = np.array([scale, scale, scale])
    las = laspy.LasData(header=header)
    las.x = pts[:, 0].astype(np.float64)
    las.y = pts[:, 1].astype(np.float64)
    las.z = pts[:, 2].astype(np.float64)
    las.write(str(out_path))
    print(f"LAS  → {out_path}  ({len(pts):,} pts)")


def main():
    parser = argparse.ArgumentParser(description='Export PCD to CAD formats')
    parser.add_argument('input', help='Input .pcd file')
    parser.add_argument('--format', default='all',
                        choices=['e57', 'las', 'xyz', 'pts', 'all'],
                        help='Output format (default: all)')
    parser.add_argument('--out', help='Output path/stem (default: same as input)')
    args = parser.parse_args()

    src = Path(args.input)
    stem = Path(args.out) if args.out else src.with_suffix('')

    print(f"Reading {src}...")
    pts = read_pcd(src)
    print(f"  {len(pts):,} points  "
          f"X={pts[:,0].min():.2f}..{pts[:,0].max():.2f}  "
          f"Y={pts[:,1].min():.2f}..{pts[:,1].max():.2f}  "
          f"Z={pts[:,2].min():.2f}..{pts[:,2].max():.2f} m")

    fmt = args.format
    if fmt in ('xyz', 'all'):
        export_xyz(pts, str(stem) + '.xyz')
    if fmt in ('pts', 'all'):
        export_pts(pts, str(stem) + '.pts')
    if fmt in ('e57', 'all'):
        export_e57(pts, str(stem) + '.e57')
    if fmt in ('las', 'all'):
        export_las(pts, str(stem) + '.las')

    print("\nДля AutoCAD / Revit / Fusion 360:")
    print("  1. Открой Autodesk ReCap (бесплатно)")
    print("  2. Импортируй .e57 или .las файл")
    print("  3. ReCap создаст .rcs — его можно вставить в AutoCAD/Revit/Fusion")


if __name__ == '__main__':
    main()
