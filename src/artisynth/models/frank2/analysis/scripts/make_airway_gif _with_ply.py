"""
把每個時間點的 airway (.ply) + centerline (.csv) 疊圖,輸出成一支動畫 (gif)
視角:矢狀面側視 (x-z 平面),因為模型 y 值幾乎都是 0,從側面看才能看到聲道形狀變化
"""

import glob
import os
import re
import pandas as pd
import pyvista as pv

# ------------------------------------------------------------------
# 1. 設定資料夾路徑 (改成你自己電腦上, geometry 檔案所在的資料夾)
# ------------------------------------------------------------------
geometry_dir = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.03_11.30.45\geometry"

output_path = "airway_animation.gif"   # 想輸出 mp4 就改成 .mp4 並把 open_gif 換成 open_movie

# 畫面放大程度:數字越小、畫面看起來越大。先試 0.04,不夠大再往下調小 (例如 0.03、0.02)
PARALLEL_SCALE = 0.065

# ------------------------------------------------------------------
# 2. 找出所有時間點,並依時間排序
#    檔名格式假設是: frank_airway_tube_clipped_0.150.ply / frank_centerline_0.150.csv
# ------------------------------------------------------------------
ply_files = glob.glob(os.path.join(geometry_dir, "frank_airway_tube_clipped_*.ply"))

def extract_time(fname):
    match = re.search(r"clipped_([\d.]+)\.ply", fname)
    return float(match.group(1))

ply_files = sorted(ply_files, key=extract_time)
print(f"找到 {len(ply_files)} 個時間點")

if len(ply_files) == 0:
    raise FileNotFoundError(
        f"在 {geometry_dir} 裡找不到 frank_airway_tube_clipped_*.ply,"
        f"檢查路徑或檔名格式是否正確"
    )

# ------------------------------------------------------------------
# 3. 輔助函式:讀取 centerline csv
# ------------------------------------------------------------------
def load_centerline(csv_path):
    df = pd.read_csv(csv_path)
    df.columns = [c.strip() for c in df.columns]
    return df[df.columns[:3]].to_numpy()

# ------------------------------------------------------------------
# 4. 建立 plotter,逐個時間點畫格並存進動畫
# ------------------------------------------------------------------
plotter = pv.Plotter(window_size=(1000, 800), off_screen=True)
plotter.open_gif(output_path)

# 用平行投影 (orthographic),搭配固定的 parallel_scale,
# 才能確保整支動畫放大倍率一致、不會忽大忽小
plotter.camera.parallel_projection = True

for i, ply_path in enumerate(ply_files):
    t = extract_time(ply_path)
    centerline_path = os.path.join(geometry_dir, f"frank_centerline_{t:.3f}.csv")

    if not os.path.exists(centerline_path):
        print(f"找不到對應的 centerline: {centerline_path}, 跳過 t={t}")
        continue

    airway_mesh = pv.read(ply_path)
    centerline_pts = load_centerline(centerline_path)
    centerline_poly = pv.lines_from_points(centerline_pts)

    plotter.clear()
    plotter.add_mesh(airway_mesh, color="lightblue", opacity=0.35)
    plotter.add_mesh(centerline_poly, color="red", line_width=5)
    plotter.add_mesh(pv.PolyData(centerline_pts), color="red", point_size=8,
                      render_points_as_spheres=True)
    plotter.add_text(f"t = {t:.3f} s", position="upper_left", font_size=14)

    # --- 視角設定:側面 (x-z 平面),每一幀都重新對準,但放大倍率固定 ---
    plotter.camera_position = 'xz'
    plotter.camera.focal_point = centerline_pts.mean(axis=0)  # 鏡頭對準這一幀的中心
    plotter.camera.parallel_scale = PARALLEL_SCALE             # 固定縮放,不會忽大忽小

    plotter.write_frame()
    print(f"[{i+1}/{len(ply_files)}] 已寫入 t={t:.3f}")

plotter.close()
print(f"完成,動畫存在: {output_path}")