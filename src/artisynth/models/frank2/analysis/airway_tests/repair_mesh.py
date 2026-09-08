#!/usr/bin/env python3
"""
repair_mesh.py
--------------
Remove self-intersections from a surface mesh, producing a clean, watertight,
self-intersection-free manifold STL that is safe to volume-mesh (GMSH, FEM, ...).

METHOD: CGAL alpha wrapping (pymeshlab.generate_alpha_wrap), then a uniform
isotropic remesh so the whole surface -- including the flat caps -- carries the
same small, regular triangles as the prep output.

Alpha wrapping shrink-wraps a tight OUTER envelope around the surface. Unlike
"remove intersecting faces + refill" repairs (PyMeshFix, mesh booleans, winding-
number extraction), it does NOT pinch the lumen where the surface self-intersects.
That matters here: the ArtiSynth airway self-intersects at the neck, and every
enclosed-volume method collapses the neck cross-section by ~25-30% (a big shift in
acoustic response). Alpha wrapping keeps the neck open.

TRADEOFF: the envelope sits ~`offset_mm` OUTSIDE the input, so the lumen is
inflated slightly and roughly uniformly (a few percent at the default settings).
Lower `offset_mm` -> less inflation, but more triangles and memory during wrapping.
The proper way to avoid both pinch and inflation is a clean (non-self-intersecting)
airway extraction upstream in ArtiSynth.

REQUIRES: pymeshlab >= 2023.12 (CGAL alpha wrap). Also trimesh, numpy.

USE AS A LIBRARY
----------------
    from repair_mesh import repair_self_intersections
    clean = repair_self_intersections(mesh)          # mesh, clean : trimesh.Trimesh

USE FROM THE COMMAND LINE
-------------------------
    python3 repair_mesh.py  in_surface.stl  out_clean.stl
"""
import sys
import numpy as np
import trimesh


def repair_self_intersections(mesh, alpha_mm=0.6, offset_mm=0.06,
                              remesh_target_mm=1.9, max_edge_mm=2.5, report=True):
    """Return a clean, watertight, self-intersection-free copy of `mesh`.

    Parameters
    ----------
    mesh : trimesh.Trimesh
        Input surface (assumed to be in millimetres). May self-intersect.
    alpha_mm : float
        Alpha-wrap carving radius (mm). Should be smaller than the smallest
        lumen feature to resolve (the neck radius here is ~3 mm), or that
        feature gets rounded off. Smaller alpha = finer detail, more faces.
    offset_mm : float
        Outward wrap distance (mm) = the (roughly uniform) inflation. Smaller is
        more faithful but costs memory/time during wrapping.
    remesh_target_mm : float
        Target edge length for the post-wrap isotropic remesh. Match the prep's
        REMESH_TARGET_MM so both meshes have the same triangle density (this is
        what gives the caps their uniform small triangles).
    max_edge_mm : float
        Hard ceiling on edge length (midpoint-split any longer edges).
    report : bool
        Print topology, self-intersection, fidelity and inflation stats.

    Returns
    -------
    trimesh.Trimesh
        Clean manifold (watertight, genus 0, 0 self-intersections), uniformly
        triangulated. The caps are reshaped by wrapping, so if a boundary
        condition needs the glottis exactly on a plane, re-establish it here.
    """
    import pymeshlab

    ms = pymeshlab.MeshSet()
    ms.add_mesh(pymeshlab.Mesh(mesh.vertices, mesh.faces))

    # 1) tight outer envelope -> removes self-intersections without pinching
    ms.generate_alpha_wrap(alpha=pymeshlab.PureValue(alpha_mm),
                           offset=pymeshlab.PureValue(offset_mm))

    # 2) uniform triangulation everywhere (incl. caps), matching the prep density
    ms.meshing_isotropic_explicit_remeshing(
        iterations=10, adaptive=False,
        targetlen=pymeshlab.PureValue(remesh_target_mm),
        featuredeg=180, checksurfdist=True,
        maxsurfdist=pymeshlab.PureValue(0.3))

    # 3) enforce the edge-length ceiling
    for _ in range(6):
        m = ms.current_mesh()
        el = trimesh.Trimesh(m.vertex_matrix(), m.face_matrix(),
                             process=True).edges_unique_length
        if el.max() <= max_edge_mm:
            break
        ms.meshing_surface_subdivision_midpoint(
            threshold=pymeshlab.PureValue(max_edge_mm), iterations=1)

    m = ms.current_mesh()
    clean = trimesh.Trimesh(m.vertex_matrix(), m.face_matrix(), process=True)
    clean.merge_vertices()
    trimesh.repair.fix_normals(clean)

    if report:
        try:
            ms.compute_selection_by_self_intersections_per_face()
            si = int(ms.current_mesh().face_selection_array().sum())
        except Exception:
            si = -1
        try:
            from trimesh.proximity import closest_point
            po, _ = trimesh.sample.sample_surface(mesh, 30000)
            _, d, _ = closest_point(clean, po)
            dev = "  fidelity vs input: p95 %.2f  max %.2f mm" % (
                np.percentile(d, 95), d.max())
        except Exception:
            dev = ""
        el = clean.edges_unique_length
        infl = (abs(clean.volume) / abs(mesh.volume) - 1.0) * 100 if mesh.volume else float('nan')
        print("  repaired (alpha wrap + remesh): faces %d  watertight %s  euler %d  "
              "self-intersections %d" % (len(clean.faces), clean.is_watertight,
                                         clean.euler_number, si))
        print("  edge length mm: min %.2f  mean %.2f  max %.2f" % (el.min(), el.mean(), el.max()))
        print("  volume %.2f mL (input %.2f mL, +%.1f%% from wrap offset)"
              % (abs(clean.volume) / 1000.0, abs(mesh.volume) / 1000.0, infl))
        if dev:
            print(dev)
    return clean


def repair_stl(in_path, out_path, **kw):
    """Load an STL, repair self-intersections, write the clean STL."""
    mesh = trimesh.load(in_path, process=True)
    mesh.merge_vertices()
    print("input :", in_path, "| faces", len(mesh.faces),
          "| watertight", mesh.is_watertight)
    clean = repair_self_intersections(mesh, **kw)
    clean.export(out_path)
    print("wrote :", out_path)
    return clean


if __name__ == "__main__":
    src = sys.argv[1] if len(sys.argv) > 1 else "frank_airway_i_processed_mm.stl"
    out = sys.argv[2] if len(sys.argv) > 2 else "frank_airway_i_clean_mm.stl"
    repair_stl(src, out)