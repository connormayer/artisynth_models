import pandas as pd
import numpy as np
import pyvista as pv

# ------------------------------------------------------------------
# 1. 讀資料 (改成你自己的路徑)
# ------------------------------------------------------------------
airway_mesh = pv.read(r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.03_11.30.45\geometry\frank_airway_tube_clipped_0.000.ply")

df = pd.read_csv(r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.03_11.30.45\geometry\frank_centerline_0.000.csv")
df.columns = [c.strip() for c in df.columns]
centerline_pts = df[df.columns[:3]].to_numpy()

# ------------------------------------------------------------------
# 2. 對每個 centerline 點,切橫截面、算偏移量
# ------------------------------------------------------------------
results = []
n = len(centerline_pts)

for i in range(1, n - 1):   # 頭尾兩點沒有前後鄰點可算切線方向,跳過
    p_prev = centerline_pts[i - 1]
    p_curr = centerline_pts[i]
    p_next = centerline_pts[i + 1]

    # 用前後點估計局部切線方向 (path tangent)
    tangent = p_next - p_prev
    tangent = tangent / np.linalg.norm(tangent)

    # 用這個方向當法向量,垂直切一刀
    section = airway_mesh.slice(normal=tangent, origin=p_curr)

    if section.n_points == 0:
        print(f"第 {i} 點沒切到東西,可能路徑跑出mesh範圍,跳過")
        continue

    section_pts = section.points
    centroid = section_pts.mean(axis=0)          # 橫截面的幾何中心
    radius_est = np.linalg.norm(section_pts - centroid, axis=1).mean()  # 平均半徑

    offset = np.linalg.norm(p_curr - centroid)    # centerline點 跟 中心 的距離
    offset_ratio = offset / radius_est             # 偏移量相對半徑的比例

    results.append({
        "index": i,
        "offset_mm": offset,          # 注意單位跟你的模型座標單位一致
        "radius_mm": radius_est,
        "offset_ratio": offset_ratio  # 建議看這個數字:越接近0代表越置中
    })

# ------------------------------------------------------------------
# 3. 整理結果
# ------------------------------------------------------------------
result_df = pd.DataFrame(results)
print(result_df.describe())
result_df.to_csv("centerline_offset_check.csv", index=False)