#!/usr/bin/env python3
"""
Convert PCD point cloud → mesh (OBJ/STL) or DXF for CAD without ReCap.

  OBJ / STL  → Fusion 360, AutoCAD (IMPORT), Revit (link SAT/DWG)
  DXF        → AutoCAD natively (3D POINT entities)

Usage:
    python export_mesh.py input.pcd [--format obj|stl|dxf|all]
                                    [--out stem]
                                    [--depth 9]       # Poisson depth (7-11)
                                    [--samples 500000] # points to use
"""
import sys, argparse
import numpy as np
from pathlib import Path


def read_pcd(path):
    with open(path, 'rb') as f:
        while True:
            line = f.readline().strip()
            if line == b"DATA binary":
                break
        return np.frombuffer(f.read(), dtype=np.float32).reshape(-1, 3)


def build_mesh(pts, depth=9):
    import open3d as o3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
    print("  Estimating normals...")
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.15, max_nn=30))
    pcd.orient_normals_consistent_tangent_plane(10)
    print(f"  Poisson reconstruction (depth={depth})...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth)
    dens = np.asarray(densities)
    mesh.remove_vertices_by_mask(dens < np.percentile(dens, 10))
    mesh.compute_vertex_normals()
    print(f"  Mesh: {len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris")
    return mesh


def export_obj(mesh, path):
    import open3d as o3d
    o3d.io.write_triangle_mesh(str(path), mesh)
    print(f"OBJ  → {path}")


def export_stl(mesh, path):
    import open3d as o3d
    mesh.compute_triangle_normals()
    o3d.io.write_triangle_mesh(str(path), mesh)
    print(f"STL  → {path}")


def export_dxf(pts, path, max_pts=200_000):
    """Write AutoCAD DXF with 3D POINT entities (no libraries needed)."""
    if len(pts) > max_pts:
        idx = np.random.choice(len(pts), max_pts, replace=False)
        pts = pts[idx]
        print(f"  DXF: downsampled to {max_pts:,} pts (AutoCAD limit)")
    with open(path, 'w') as f:
        f.write("0\nSECTION\n2\nHEADER\n0\nENDSEC\n")
        f.write("0\nSECTION\n2\nENTITIES\n")
        for x, y, z in pts:
            f.write(f"0\nPOINT\n8\nPointCloud\n10\n{x:.4f}\n20\n{y:.4f}\n30\n{z:.4f}\n")
        f.write("0\nENDSEC\n0\nEOF\n")
    print(f"DXF  → {path}  ({len(pts):,} pts)")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('input')
    parser.add_argument('--format', default='all',
                        choices=['obj', 'stl', 'dxf', 'all'])
    parser.add_argument('--out', default=None)
    parser.add_argument('--depth', type=int, default=9,
                        help='Poisson depth 7=fast/coarse, 11=slow/fine')
    parser.add_argument('--samples', type=int, default=500_000,
                        help='Points to use for mesh (more=slower)')
    args = parser.parse_args()

    src = Path(args.input)
    stem = Path(args.out) if args.out else src.with_suffix('')

    print(f"Reading {src.name}...")
    pts = read_pcd(src)
    print(f"  {len(pts):,} points")

    fmt = args.format
    need_mesh = fmt in ('obj', 'stl', 'all')

    if need_mesh:
        n = min(args.samples, len(pts))
        idx = np.random.choice(len(pts), n, replace=False)
        print(f"  Using {n:,} pts for mesh...")
        mesh = build_mesh(pts[idx], depth=args.depth)

    if fmt in ('obj', 'all'):
        export_obj(mesh, str(stem) + '.obj')
    if fmt in ('stl', 'all'):
        export_stl(mesh, str(stem) + '.stl')
    if fmt in ('dxf', 'all'):
        export_dxf(pts, str(stem) + '.dxf')

    print("\n--- Как использовать ---")
    print("Fusion 360 : File → Open → выбери .obj или .stl")
    print("AutoCAD    : команда IMPORT → .obj/.stl, или DXFATTACH → .dxf")
    print("Revit      : Insert → Import CAD → .dxf  (только для 2D контуров)")
    print("             Insert → Link CAD → .dwg  (конвертируй .dxf → .dwg в AutoCAD)")


if __name__ == '__main__':
    main()
