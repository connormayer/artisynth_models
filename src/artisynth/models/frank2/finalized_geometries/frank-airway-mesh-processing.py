#!/usr/bin/env python3
"""
frank-airway-mesh-processing.py
Prepare a vocal-tract airway surface mesh for acoustic simulation.

Pipeline
--------
1. Clean & repair  -> remove junk fragments, non-manifold edges
2. Scale meters -> millimetres (x1000) to match the bahn reference mesh
3. Isotropic remesh (open walls) -> uniform triangles, edge -> REMESH_TARGET
4. FLAT PLANAR CAPS -> both tube ends closed with a genuinely flat surface:
      fit a plane to each open rim, snap the rim onto it, and fill the disk
      with a quality (Delaunay) triangulation lying exactly in that plane.
5. Targeted split -> every edge <= MAX_EDGE_MM (2.5 mm)
6. Reorient       -> glottal cap centroid at the ORIGIN, cap lying in the
                     XY plane (Z=0), tube extending along +Z
7. Verify watertight and export
8. Write a self-intersection-free copy (CGAL alpha wrap) for volume meshing:
      <output>_clean.stl   (disable via WRITE_REPAIRED = False)

Dependencies:  pip install trimesh pymeshlab scipy numpy matplotlib  # pymeshlab>=2023.12
Usage:         python vocal_tract_prep.py input.stl output.stl
"""

import sys
import numpy as np
import trimesh
import pymeshlab
import networkx as nx
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra
from scipy.spatial import Delaunay, cKDTree
from matplotlib.path import Path
from trimesh.grouping import group_rows

# --------------------------- CONFIG ---------------------------------------
SCALE_TO_MM      = 1000.0    # input is in metres; x1000 -> mm. Set 1.0 if already mm.
MAX_EDGE_MM      = 2.5       # hard ceiling on edge length
REMESH_TARGET_MM = 1.9       # isotropic target (< MAX so the spread stays under it)
REMESH_ITERS     = 10
MIN_COMPONENT    = 25        # drop connected components smaller than this many faces

# --- Surface-fidelity controls (prevent the remesh from bridging/deepening the
#     airway's real folds, which otherwise shows up as spurious constrictions) ---
FEATURE_DEG      = 180.0     # 180 = uniform triangles (no crease-forced slivers)
CHECK_SURF_DIST  = True      # bound how far the remesh may drift from the input
MAX_SURF_DIST_MM = 0.3       # that bound (mm). This is the key fix vs bridging folds.
WELD_MM          = 0.15      # weld vertices closer than this to kill sliver/degenerate tris.
                              # Must be bigger than the sliver edges the cap-plane slice in
                              # reflatten_caps() leaves behind (observed ~0.055-0.10 mm) or
                              # GMsh's volume mesher reports "overlapping facets" on the wall.

# Which end is the glottis?  'auto' picks the narrower cap (the glottis is the
# constriction).  Override with 'A' or 'B' after reading the printed coordinates
# and matching them to what you see in Blender.
GLOTTIS_END      = 'A'

# If True, additionally shift X and Y so the whole mesh sits in the positive
# octant.  NOTE: this moves the glottal centroid off (0,0,0) in X and Y while
# keeping it on the Z=0 plane.  Default False = glottal centroid exactly at origin.
POSITIVE_OCTANT  = False

# After prep, also write a self-intersection-free copy (via CGAL alpha wrap) suitable
# for volume meshing. The clean file is named <output>_clean.stl.
WRITE_REPAIRED   = True
# --------------------------------------------------------------------------


def clean_and_repair(path):
    """Load, strip junk fragments, repair non-manifold. Ends are left OPEN on
    purpose -- they are closed later with flat planar caps (see flat_caps)."""
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(path)
    ms.meshing_remove_duplicate_vertices()
    ms.meshing_remove_duplicate_faces()
    ms.meshing_remove_connected_component_by_face_number(mincomponentsize=MIN_COMPONENT)
    ms.meshing_repair_non_manifold_edges()
    ms.meshing_remove_unreferenced_vertices()
    ms.compute_matrix_from_scaling_or_normalization(
        axisx=SCALE_TO_MM, axisy=SCALE_TO_MM, axisz=SCALE_TO_MM)
    return ms


def remesh_walls(ms):
    """Isotropic remesh of the (open) tube walls to REMESH_TARGET.
    Open boundaries are preserved, so the two end rims stay put.
    checksurfdist + feature preservation keep the remesh on the real surface so
    the airway's folds are reproduced faithfully instead of being bridged over
    (which previously deepened them into spurious constrictions)."""
    kw = dict(iterations=REMESH_ITERS, adaptive=False,
              targetlen=pymeshlab.PureValue(REMESH_TARGET_MM),
              featuredeg=FEATURE_DEG, checksurfdist=CHECK_SURF_DIST)
    if CHECK_SURF_DIST:
        kw['maxsurfdist'] = pymeshlab.PureValue(MAX_SURF_DIST_MM)
    ms.meshing_isotropic_explicit_remeshing(**kw)
    return ms


def split_long_edges(ms):
    """Midpoint-split until every edge <= MAX_EDGE_MM. Splitting coplanar cap
    triangles keeps them exactly coplanar, so the flat caps stay flat."""
    for _ in range(8):
        m = ms.current_mesh()
        el = trimesh.Trimesh(m.vertex_matrix(), m.face_matrix(),
                             process=True).edges_unique_length
        if el.max() <= MAX_EDGE_MM:
            break
        ms.meshing_surface_subdivision_midpoint(
            threshold=pymeshlab.PureValue(MAX_EDGE_MM), iterations=1)
    # weld any near-coincident vertices created along creases -> removes slivers
    ms.meshing_merge_close_vertices(threshold=pymeshlab.PureValue(WELD_MM))
    return ms


def to_trimesh(ms):
    m = ms.current_mesh()
    tm = trimesh.Trimesh(m.vertex_matrix(), m.face_matrix(), process=True)
    tm.merge_vertices()
    return tm


def to_meshset(tm):
    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(tm.vertices, tm.faces))
    return ms


# --------------------------- capping --------------------------------------
def boundary_loops(tm):
    """Ordered vertex-index loops for every open boundary (each tube end)."""
    edges = tm.edges_sorted
    open_edges = edges[group_rows(edges, require_count=1)]
    G = nx.Graph()
    G.add_edges_from(map(tuple, open_edges))
    loops = []
    for comp in nx.connected_components(G):
        sub = G.subgraph(comp)
        start = next(iter(comp)); order = [start]; prev = None; cur = start
        while True:
            nbrs = [n for n in sub.neighbors(cur) if n != prev]
            if not nbrs or nbrs[0] == start:
                break
            prev, cur = cur, nbrs[0]
            order.append(cur)
        loops.append(np.array(order))
    return loops


def plane_fit(pts):
    """Best-fit plane (centroid, unit normal) via PCA."""
    c = pts.mean(0)
    _, _, vt = np.linalg.svd(pts - c, full_matrices=False)
    return c, vt[2]


def _planar_cap(tm, loop, target):
    """Build a flat, quality-triangulated cap for one boundary loop.
    Returns (rim_projected_xyz, interior_xyz, cap_faces) with cap_faces indexing
    rim vertices by their *global* ids and interior vertices by (base + i)."""
    rim = tm.vertices[loop]
    c, n = plane_fit(rim)
    # in-plane orthonormal basis (e1, e2) spanning the cap plane
    seed = np.array([1., 0., 0.]) if abs(n[0]) < 0.9 else np.array([0., 1., 0.])
    e1 = np.cross(n, seed); e1 /= np.linalg.norm(e1)
    e2 = np.cross(n, e1)
    # snap rim exactly onto the plane, express as 2D coords
    rim_p = rim - np.outer((rim - c) @ n, n)
    P2 = np.c_[(rim_p - c) @ e1, (rim_p - c) @ e2]
    poly = Path(np.vstack([P2, P2[:1]]))
    # interior grid points, kept comfortably inside the rim polygon
    lo, hi = P2.min(0) - target, P2.max(0) + target
    gx = np.arange(lo[0], hi[0], target * 0.87)
    gy = np.arange(lo[1], hi[1], target * 0.87)
    grid = np.c_[np.repeat(gx, len(gy)), np.tile(gy, len(gx))]
    keep = poly.contains_points(grid)
    keep &= cKDTree(P2).query(grid)[0] > 0.55 * target
    interior2 = grid[keep]
    pts2 = np.vstack([P2, interior2])
    tri = Delaunay(pts2)
    good = poly.contains_points(pts2[tri.simplices].mean(1))   # drop outside tris
    simp = tri.simplices[good]
    base = len(tm.vertices)
    idmap = np.concatenate([loop, base + np.arange(len(interior2))])
    cap_faces = idmap[simp]
    interior3 = c + np.outer(interior2[:, 0], e1) + np.outer(interior2[:, 1], e2)
    return rim_p, interior3, cap_faces


def drop_spanning_triangles(tm, max_iter=6):
    """Remove wall triangles whose three vertices all lie on a boundary loop.
    These are webs across a notched/flared opening; if left in, they fold flat
    against the cap after capping and create zero-thickness flaps (self-
    intersections that stop GMSH volume meshing). Iterated because removing one
    can expose another."""
    for _ in range(max_iter):
        loops = boundary_loops(tm)
        onbound = set()
        for l in loops:
            onbound.update(l.tolist())
        span = np.array([all(v in onbound for v in f) for f in tm.faces])
        if not span.any():
            break
        tm.update_faces(~span)
        tm.remove_unreferenced_vertices()
    return tm


def flat_caps(tm, target=REMESH_TARGET_MM):
    """Close every open end with a genuinely flat planar cap.
    Also returns each cap's exact plane as (centroid, unit_normal, rim_radius),
    in the mesh's current coordinates, so orientation can align to it precisely."""
    tm = drop_spanning_triangles(tm)                     # kill opening-webs -> no cap flaps
    loops = boundary_loops(tm)
    V = tm.vertices.copy()
    F = list(tm.faces.copy())
    caps = []
    for loop in loops:
        rim_p, interior3, cap_faces = _planar_cap(tm, loop, target)
        c, n = plane_fit(rim_p)
        rim_radius = np.linalg.norm(rim_p - c, axis=1).max()
        caps.append((c, n, rim_radius))
        V[loop] = rim_p                                   # snap rim onto plane
        off = len(V)
        # remap interior ids (built assuming base=len(orig V)) to current offset
        cap_faces = np.where(cap_faces >= len(tm.vertices),
                             cap_faces - len(tm.vertices) + off, cap_faces)
        V = np.vstack([V, interior3])
        F.extend(cap_faces.tolist())
    out = trimesh.Trimesh(V, np.array(F), process=True)
    out.merge_vertices()
    out.update_faces(out.nondegenerate_faces())
    out.update_faces(out.unique_faces())
    trimesh.repair.fix_normals(out)                       # consistent outward normals
    return out, caps


def reflatten_caps(tm, planes, target=REMESH_TARGET_MM):
    """Slice off the (wrapped/rounded) end regions at the given planes and
    re-cap them flat. Used to restore genuinely flat, sharp-edged caps after the
    alpha-wrap repair, which otherwise rounds the rim and bulges the cap by the
    wrap offset. `planes` = list of (centroid, outward_unit_normal)."""
    from trimesh.intersections import slice_faces_plane
    V, F = np.asarray(tm.vertices, float), np.asarray(tm.faces)
    for c, n in planes:
        c = np.asarray(c, float); n = np.asarray(n, float)
        V, F, _ = slice_faces_plane(V, F, plane_normal=-n, plane_origin=c)  # keep interior
    m = trimesh.Trimesh(V, F, process=True); m.merge_vertices()
    m, _ = flat_caps(m, target)                       # fresh flat planar caps
    m = to_trimesh(split_long_edges(to_meshset(m)))   # uniform edges on the new caps
    trimesh.repair.fix_normals(m)
    return m


# --------------------- endpoint / orientation helpers ---------------------
def geodesic_graph(tm):
    E = tm.edges_unique
    w = np.linalg.norm(tm.vertices[E[:, 0]] - tm.vertices[E[:, 1]], axis=1)
    n = len(tm.vertices)
    rows = np.r_[E[:, 0], E[:, 1]]
    cols = np.r_[E[:, 1], E[:, 0]]
    return csr_matrix((np.r_[w, w], (rows, cols)), shape=(n, n))


def find_endpoints(tm, G):
    """Two tube tips = the graph-diameter endpoints (double Dijkstra)."""
    a = int(np.argmax(dijkstra(G, indices=0)))
    da = dijkstra(G, indices=a)
    b = int(np.argmax(da))
    return a, b, da[b]


def rot_from_to(a, b):
    """Rotation matrix sending unit vector a onto unit vector b."""
    a = a / np.linalg.norm(a); b = b / np.linalg.norm(b)
    v = np.cross(a, b); c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1, -1, -1]).astype(float)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def build_transform(g_centroid, g_axis, lips_centroid):
    """glottal centroid -> origin; glottal axis -> +Z (cap lies in the XY plane);
    tube bend -> +X."""
    R1 = rot_from_to(g_axis, np.array([0., 0., 1.]))       # glottal axis -> +Z
    lips_local = R1 @ (lips_centroid - g_centroid)
    # align the in-plane chord (glottis->lips, projected to XY) with +X (rotate about Z)
    theta = np.arctan2(lips_local[1], lips_local[0])       # angle in XY plane
    ct, st = np.cos(theta), np.sin(theta)
    R2 = np.array([[ct, st, 0], [-st, ct, 0], [0, 0, 1]])  # sends chord dir -> +X
    R = R2 @ R1
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = -R @ g_centroid
    return T


def main(inp, outp):
    print("Loading & repairing:", inp)
    ms = clean_and_repair(inp)
    ms = remesh_walls(ms)
    tm = to_trimesh(ms)

    print("Closing tube ends with flat planar caps ...")
    tm, caps = flat_caps(tm)
    print(f"  capped {len(caps)} open end(s)")
    ms = split_long_edges(to_meshset(tm))
    tm = to_trimesh(ms)
    trimesh.repair.fix_normals(tm)

    # geodesic arc length (report only)
    G = geodesic_graph(tm)
    _, _, arc = find_endpoints(tm, G)

    # caps[i] = (centroid, unit_normal, rim_radius) in current coords
    (cA, nA, rA), (cB, nB, rB) = caps[0], caps[1]
    print("\nDetected tube ends (mm, pre-orientation):")
    print(f"  End A: centroid {np.round(cA,1)}  rim radius {rA:.1f}")
    print(f"  End B: centroid {np.round(cB,1)}  rim radius {rB:.1f}")
    print(f"  geodesic tube length (arc): {arc:.1f} mm")

    if GLOTTIS_END in ('A', 'B'):
        glottis = GLOTTIS_END
    else:
        glottis = 'A' if rA <= rB else 'B'        # narrower cap = glottis
    print(f"  --> glottis = End {glottis}  "
          f"({'set manually' if GLOTTIS_END!='auto' else 'auto: narrower cap'})")

    if glottis == 'A':
        g_c, g_n, l_c, l_n = cA, nA, cB, nB
    else:
        g_c, g_n, l_c, l_n = cB, nB, cA, nA

    # align the glottal cap's OWN plane normal (inward-pointing) to +Z, so the
    # cap lies exactly in the XY plane; centroid to origin; bend -> +X.
    g_axis = g_n * np.sign(np.dot(g_n, l_c - g_c))        # point into the tube
    T = build_transform(g_c, g_axis, l_c)
    tm.apply_transform(T)

    if POSITIVE_OCTANT:
        mn = tm.bounds[0]
        tm.apply_translation([-mn[0], -mn[1], 0.0])   # keep glottis on Z=0

    # ---- verification ----
    el = tm.edges_unique_length
    print("\nFinal mesh:")
    print(f"  vertices {len(tm.vertices)}  faces {len(tm.faces)}")
    print(f"  watertight: {tm.is_watertight}   euler: {tm.euler_number} (2 = closed genus-0)")
    print(f"  winding consistent: {tm.is_winding_consistent}")
    print(f"  edge length mm: min {el.min():.2f}  mean {el.mean():.2f}  "
          f"max {el.max():.2f}  (ceiling {MAX_EDGE_MM})")
    print(f"  bbox min mm: {np.round(tm.bounds[0],2)}")
    print(f"  bbox max mm: {np.round(tm.bounds[1],2)}")
    print(f"  volume: {tm.volume/1000.0:.2f} mL")

    # glottal cap should now lie in the XY plane (Z = 0): report residual
    g_rad = rA if glottis == 'A' else rB
    xy = np.linalg.norm(tm.vertices[:, :2], axis=1)
    cap_v = tm.vertices[(np.abs(tm.vertices[:, 2]) < 1e-3) & (xy < g_rad + 0.5)]
    zmax = np.abs(cap_v[:, 2]).max() if len(cap_v) else float('nan')
    print(f"  glottal cap centroid now at: {np.round(T[:3,:3] @ g_c + T[:3,3], 4)}")
    print(f"  glottal cap lies in XY plane: max |Z| = {zmax:.2e} mm "
          f"over {len(cap_v)} cap verts")

    tm.export(outp)
    print("\nWrote:", outp)

    # ---- self-intersection-free copy for volume meshing ----
    if WRITE_REPAIRED:
        import os
        from repair_mesh import repair_self_intersections
        clean_path = os.path.splitext(outp)[0] + "_clean.stl"
        print("\nRepairing self-intersections (alpha wrap) ...")
        clean = repair_self_intersections(tm)

        # alpha wrap rounds/bulges the caps; restore flat, sharp-edged caps by
        # slicing at the (transformed) cap planes and re-capping.
        R, t = T[:3, :3], T[:3, 3]
        g_plane = (np.array([0., 0., 0.]), np.array([0., 0., -1.]))   # glottis: XY plane, outward -Z
        l_n_out = l_n * np.sign(np.dot(l_n, l_c - g_c))               # lips outward normal
        l_plane = (R @ l_c + t, R @ l_n_out)
        clean = reflatten_caps(clean, [g_plane, l_plane])
        clean.export(clean_path)
        print("Wrote:", clean_path)

    return tm


if __name__ == "__main__":
    inp  = sys.argv[1] if len(sys.argv) > 1 else "frank_airway_a_0.400.stl"
    outp = sys.argv[2] if len(sys.argv) > 2 else "frank_airway_a_processed_mm.stl"
    main(inp, outp)