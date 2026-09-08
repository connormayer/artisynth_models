import bmesh
import bpy
import mathutils

obj = bpy.context.active_object
bpy.ops.object.mode_set(mode="EDIT")
bm = bmesh.from_edit_mesh(obj.data)

max_edge_length = 0.002  # 2mm (Blender units are assumed to be meters)

# Step 1: collect all boundary edges
boundary_edges = [e for e in bm.edges if not e.is_manifold]
print(f"Found {len(boundary_edges)} boundary edges in total")


def split_into_loops(edges):
    """Group boundary edges by connectivity into separate holes (each hole = a set of edges)"""
    remaining = set(edges)
    loops = []
    while remaining:
        seed = next(iter(remaining))
        group = {seed}
        frontier = [seed]
        remaining.discard(seed)
        while frontier:
            e = frontier.pop()
            for v in e.verts:
                for e2 in v.link_edges:
                    if e2 in remaining:
                        group.add(e2)
                        remaining.discard(e2)
                        frontier.append(e2)
        loops.append(list(group))
    return loops


loops = split_into_loops(boundary_edges)
print(f"Detected {len(loops)} independent holes")


def order_boundary_loop(edges):
    edge_map = {}
    for e in edges:
        v1, v2 = e.verts
        edge_map.setdefault(v1, []).append(v2)
        edge_map.setdefault(v2, []).append(v1)
    start = next(iter(edge_map))
    loop = [start]
    prev = None
    current = start
    while True:
        neighbors = edge_map[current]
        next_v = neighbors[0] if neighbors[0] != prev else neighbors[1]
        if next_v == start:
            break
        loop.append(next_v)
        prev, current = current, next_v
    return loop


def polygon_area_newell(points):
    """Newell's method: works even if the polygon is not perfectly planar."""
    n = len(points)
    normal = mathutils.Vector((0.0, 0.0, 0.0))
    for i in range(n):
        p1 = points[i]
        p2 = points[(i + 1) % n]
        normal.x += (p1.y - p2.y) * (p1.z + p2.z)
        normal.y += (p1.z - p2.z) * (p1.x + p2.x)
        normal.z += (p1.x - p2.x) * (p1.y + p2.y)
    return normal.length / 2.0


mat_world = obj.matrix_world

# Unit conversion: assumes 1 Blender unit = 1 meter (default metric scale).
# area (m^2) * M2_TO_CM2 = area (cm^2)
M2_TO_CM2 = 10_000.0


# Step 1.5: fix a stable ordering for the holes
# split_into_loops groups edges using a Python set internally, so the
# traversal order depends on bmesh element memory addresses and is NOT
# guaranteed to be stable across runs on the same model. That means
# "Hole 1" might be the left hole one run and the right hole the next,
# which is bad for handoffs and comparing logs over time.
#
# Here we sort based on a known relationship between the two holes in
# this particular model (a *relative* comparison between hole 1 and
# hole 2, not an absolute comparison of one hole's own x vs z):
#   hole_1.x < hole_2.x   (hole 1's x coordinate is smaller)
#   hole_1.z > hole_2.z   (hole 1's z coordinate is larger)
#
# Sorting by (x - z) satisfies both relations at once. Proof:
#   Given x1 < x2 and z1 > z2 (i.e. z1 - z2 > 0)
#   (x1 - z1) - (x2 - z2) = (x1 - x2) - (z1 - z2) = negative - positive = more negative
#   => (x1 - z1) < (x2 - z2) always holds, regardless of the absolute
#      values of x and z, as long as both relative relations hold.
# Sorting ascending, the entry with the smaller (x - z) naturally comes
# first and will always correspond to "Hole 1".
# If the number of holes changes later, or these two relative
# relations no longer both hold, this sort key will need to be revisited.
def loop_sort_key(loop_edges):
    ordered_verts = order_boundary_loop(loop_edges)
    world_coords = [mat_world @ v.co for v in ordered_verts]
    center = sum(world_coords, mathutils.Vector((0.0, 0.0, 0.0))) / len(world_coords)
    # Round to 5 decimal places to avoid floating point noise destabilizing the sort
    return round(center.x - center.z, 5)


loops.sort(key=loop_sort_key)


# Step 2: compute each hole's area and location (record this BEFORE filling)
for i, loop_edges in enumerate(loops):
    ordered_verts = order_boundary_loop(loop_edges)
    coords = [v.co.copy() for v in ordered_verts]
    area_m2 = polygon_area_newell(coords)
    area_cm2 = area_m2 * M2_TO_CM2

    # World-space center (average of vertex coordinates), for direct
    # comparison against the object's position in the Blender scene
    world_coords = [mat_world @ co for co in coords]
    center = sum(world_coords, mathutils.Vector((0.0, 0.0, 0.0))) / len(world_coords)

    # Bounding box, to give a rough sense of the hole's extent/size
    xs = [c.x for c in world_coords]
    ys = [c.y for c in world_coords]
    zs = [c.z for c in world_coords]
    bbox_min = mathutils.Vector((min(xs), min(ys), min(zs)))
    bbox_max = mathutils.Vector((max(xs), max(ys), max(zs)))

    print(f"Hole {i+1}: {len(coords)} vertices, area ≈ {area_cm2:.4f} cm^2")
    print(f"    Center (world space) ≈ ({center.x:.5f}, {center.y:.5f}, {center.z:.5f})")
    print(f"    Bounding box min=({bbox_min.x:.5f}, {bbox_min.y:.5f}, {bbox_min.z:.5f})  "
          f"max=({bbox_max.x:.5f}, {bbox_max.y:.5f}, {bbox_max.z:.5f})")

# Step 3: fill each hole one at a time (processed separately, and verified to actually fill)
# Key fixes:
#   1. grid_fill works well on rectangular/grid-like boundaries, but
#      often "silently fails" on circular or irregular boundaries -
#      it doesn't raise an exception, but also produces no new faces.
#      A plain try/except won't catch this; we must check whether the
#      returned faces list is actually non-empty.
#   2. Each hole's edges are passed to grid_fill separately - never
#      merge multiple holes' edges into a single grid_fill call.
filled_count = 0
for i, loop_edges in enumerate(loops):
    new_faces = []
    method = None

    try:
        res = bmesh.ops.grid_fill(bm, edges=loop_edges)
        new_faces = res.get("faces", [])
        if new_faces:
            method = "grid_fill"
    except Exception as ex:
        print(f"Hole {i+1}: grid_fill raised an exception: {ex}")

    if not new_faces:
        # grid_fill produced no faces - fall back to holes_fill (n-gon
        # filling), which is more robust for arbitrary shapes
        # (especially circular holes)
        res2 = bmesh.ops.holes_fill(bm, edges=loop_edges)
        new_faces = res2.get("faces", [])
        if new_faces:
            method = "holes_fill"

    if new_faces:
        filled_count += 1
        print(f"Hole {i+1}: filled successfully ({method}), added {len(new_faces)} faces")

        # Key fix:
        # holes_fill produces a single large n-gon with no internal
        # edges or vertices at all. The later "subdivide long edges"
        # logic scans bm.edges for edges over the threshold, but this
        # large face only has boundary edges (already dense, never
        # over threshold), so the face's interior would never get
        # subdivided - even though the actual span from hole center to
        # boundary could be far greater than the 2.5mm threshold and
        # go completely undetected.
        # Triangulating the new faces here creates real diagonal edges
        # that cross the hole interior; these diagonals will typically
        # exceed the threshold and get picked up (and recursively
        # subdivided) by the loop below.
        ngon_or_quad = [f for f in new_faces if len(f.verts) > 3]
        if ngon_or_quad:
            bmesh.ops.triangulate(
                bm, faces=ngon_or_quad, ngon_method="BEAUTY", quad_method="BEAUTY"
            )
    else:
        print(f"⚠️ Hole {i+1}: fill FAILED! Neither grid_fill nor holes_fill produced any "
              f"faces - check whether this hole's boundary is valid (e.g. edges may not "
              f"actually form a closed loop)")

print(f"Filled {filled_count} / {len(loops)} holes")

# New faces' normals may not match the surrounding geometry - recalculate all
bmesh.ops.recalc_face_normals(bm, faces=bm.faces[:])

bmesh.update_edit_mesh(obj.data)

# Step 4: repeatedly subdivide long edges until they meet the 2mm threshold
for i in range(20):
    bm = bmesh.from_edit_mesh(obj.data)
    long_edges = [e for e in bm.edges if e.calc_length() > max_edge_length]
    print(f"Subdivision pass {i+1}: {len(long_edges)} edges over threshold")
    if not long_edges:
        print("✅ All edges meet the 2mm threshold")
        break
    bmesh.ops.subdivide_edges(bm, edges=long_edges, cuts=1, use_grid_fill=True)
    bmesh.update_edit_mesh(obj.data)

# Final check: any boundary edges still remaining (i.e. holes not fully closed)?
bm_final = bmesh.from_edit_mesh(obj.data)
remaining_boundary = [e for e in bm_final.edges if not e.is_manifold]
print(f"Remaining boundary edges after filling: {len(remaining_boundary)} (0 means fully closed)")

bpy.ops.object.mode_set(mode="OBJECT")