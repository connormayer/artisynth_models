#!/usr/bin/env python3
"""
find_self_intersections.py
---------------------------
Locate self-intersecting faces in a repaired airway mesh (the *_clean.stl
output from frank-airway-mesh-processing.py / repair_mesh.py).

For each input file, this:
  1. Loads the mesh
  2. Selects the faces flagged as self-intersecting
  3. Prints how many there are and their approximate 3D location
     (so you can judge, without opening Blender, whether they cluster
     around a particular region like the neck/glottis)
  4. Saves a copy with those faces colored red, so you can visually
     inspect them in Blender / MeshLab

Usage:
    python find_self_intersections.py path\to\frank_airway_processed_mm_clean.stl

You can pass multiple files at once:
    python find_self_intersections.py outputs\a_clean.stl outputs\i_clean.stl
"""
import sys
import os
import pymeshlab
import numpy as np


def find_self_intersections(path):
    print(f"\n{'='*60}")
    print(f"Checking: {path}")
    print('='*60)

    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(path)

    # Flag every face that participates in a self-intersection
    ms.compute_selection_by_self_intersections_per_face()

    m = ms.current_mesh()
    selected = m.face_selection_array()  # boolean array, one entry per face
    n_flagged = int(selected.sum())
    print(f"Flagged faces: {n_flagged}")

    if n_flagged == 0:
        print("No self-intersections found. Nothing more to do for this file.")
        return

    # Compute the centroid (average 3D position) of each flagged face,
    # so we can report roughly *where* the problem is without opening a viewer.
    faces = m.face_matrix()
    verts = m.vertex_matrix()
    flagged_face_ids = np.where(selected)[0]

    print("\nApproximate location of each flagged face (centroid, mm):")
    centroids = []
    for fid in flagged_face_ids:
        tri = faces[fid]
        centroid = verts[tri].mean(axis=0)
        centroids.append(centroid)
        print(f"  face {fid:5d}:  x={centroid[0]:8.2f}  y={centroid[1]:8.2f}  z={centroid[2]:8.2f}")

    centroids = np.array(centroids)
    print(f"\nOverall spread of flagged faces:")
    print(f"  x range: {centroids[:,0].min():.2f} to {centroids[:,0].max():.2f}")
    print(f"  y range: {centroids[:,1].min():.2f} to {centroids[:,1].max():.2f}")
    print(f"  z range: {centroids[:,2].min():.2f} to {centroids[:,2].max():.2f}")
    print("  (compare these to the glottis-at-origin / tube-along-+Z convention")
    print("   from the main script: small z = near the glottis, large z = near the lips)")

    # Save a colored copy: flagged faces marked red, everything else white,
    # so you can see them directly in Blender / MeshLab. pymeshlab has no
    # direct "color from selection" filter, so build the colors ourselves
    # and export via trimesh, which reliably writes per-face color in PLY.
    try:
        import trimesh
        face_colors = np.tile([255, 255, 255, 255], (len(faces), 1))  # white
        face_colors[flagged_face_ids] = [255, 0, 0, 255]               # red
        tm = trimesh.Trimesh(vertices=verts, faces=faces, process=False)
        tm.visual.face_colors = face_colors
        out_path = os.path.splitext(path)[0] + "_marked.ply"
        tm.export(out_path)
        print(f"\nSaved marked mesh (flagged faces in red): {out_path}")
        print("Open this in Blender or MeshLab (with vertex/face color display on) to see exactly where the problem faces are.")
    except Exception as ex:
        print(f"\nCould not save colored mesh ({ex}); face coordinates above are still valid.")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python find_self_intersections.py file1.stl [file2.stl ...]")
        sys.exit(1)

    for path in sys.argv[1:]:
        if not os.path.exists(path):
            print(f"File not found, skipping: {path}")
            continue
        find_self_intersections(path)