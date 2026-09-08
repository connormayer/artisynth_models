import os
import bmesh
import bpy
import mathutils

# ==================== 設定區 ====================
ply_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\geometry\airway_complete_tube_clipped.ply"
cut_point = (0.048417, -0.007758, 0.098242)
cut_normal = (0.58859116, 0.00986789, 0.80837063)
suffix = "_z_parallel_cut"

# 【新增】只切割 cut_point 附近的局部區域，避免切到下面彎曲的部分
# 這個半徑要涵蓋你想切的整個範圍（含切口所有邊緣），但別大到碰到下方彎曲處
cut_radius = 0.09   # 單位跟你的模型座標一致（這裡假設是公尺，先抓 2cm 試試）
# ===============================================

# 步驟一：清空場景
for obj in list(bpy.data.objects):
    bpy.data.objects.remove(obj, do_unlink=True)
print("場景已清空")

# 步驟二：匯入 PLY
if not os.path.exists(ply_path):
    raise FileNotFoundError(f"找不到 PLY 檔案：{ply_path}")

try:
    bpy.ops.wm.ply_import(filepath=ply_path)
except AttributeError:
    bpy.ops.import_mesh.ply(filepath=ply_path)
print(f"成功匯入：{ply_path}")

orig_obj = bpy.context.active_object
if not orig_obj and bpy.context.selected_objects:
    orig_obj = bpy.context.selected_objects[0]
    bpy.context.view_layer.objects.active = orig_obj

if not (orig_obj and orig_obj.type == "MESH"):
    raise RuntimeError("未能辨識匯入的 Mesh 物件")

# 步驟三：套用變換，讓 local 座標 = world 座標
bpy.ops.object.select_all(action="DESELECT")
orig_obj.select_set(True)
bpy.context.view_layer.objects.active = orig_obj
bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
print("已套用變換")

base_dir = os.path.dirname(ply_path)
ply_name = os.path.splitext(os.path.basename(ply_path))[0]


def make_half(keep_outer: bool, part_index: int):
    bpy.ops.object.select_all(action="DESELECT")
    orig_obj.select_set(True)
    bpy.context.view_layer.objects.active = orig_obj
    bpy.ops.object.duplicate()
    half_obj = bpy.context.active_object

    bpy.ops.object.mode_set(mode="EDIT")
    bm = bmesh.from_edit_mesh(half_obj.data)

    center = mathutils.Vector(cut_point)

    # 【新增】只挑選在 cut_point 半徑範圍內的頂點/邊/面
    region_verts = [v for v in bm.verts if (v.co - center).length <= cut_radius]
    region_vert_set = set(region_verts)

    region_edges = [e for e in bm.edges
                     if e.verts[0] in region_vert_set and e.verts[1] in region_vert_set]

    region_faces = [f for f in bm.faces
                     if all(v in region_vert_set for v in f.verts)]

    geom = region_verts + region_edges + region_faces
    print(f"[part{part_index}] 區域內共 {len(region_verts)} 個頂點、"
          f"{len(region_faces)} 個面 參與切割")

    if not region_faces:
        print(f"警告：cut_radius={cut_radius} 範圍內沒有面，"
              f"請增加 cut_radius 數值")

    result = bmesh.ops.bisect_plane(
        bm,
        geom=geom,   # 只用局部區域，而不是整個模型
        plane_co=center,
        plane_no=mathutils.Vector(cut_normal),
        clear_outer=keep_outer,
        clear_inner=not keep_outer,
    )

    if not result.get("geom_cut"):
        print(f"警告：part{part_index} 切割平面沒有與局部區域產生交集")

    bmesh.update_edit_mesh(half_obj.data)
    bpy.ops.object.mode_set(mode="OBJECT")

    new_filepath = os.path.join(base_dir, f"{ply_name}{suffix}_part{part_index}.ply")
    bpy.ops.object.select_all(action="DESELECT")
    half_obj.select_set(True)
    bpy.context.view_layer.objects.active = half_obj

    try:
        bpy.ops.wm.ply_export(filepath=new_filepath, export_selected_objects=True)
    except (AttributeError, TypeError):
        try:
            bpy.ops.export_mesh.ply(filepath=new_filepath, use_selection=True)
        except AttributeError:
            bpy.ops.wm.ply_export(filepath=new_filepath)

    print(f"已匯出部件 {part_index}：{new_filepath}")


make_half(keep_outer=True, part_index=1)
make_half(keep_outer=False, part_index=2)

print("全部完成！只產生 2 個部件，且只切割局部區域。")