# """
# 疊圖腳本:把某個時間點的 airway mesh (.ply) 跟 centerline (.csv) 疊在一起顯示
# 用途:視覺檢查 centerline 是否真的穿在 airway 管子的中軸上

# import pandas as pd
# import pyvista as pv

# # ------------------------------------------------------------------
# # 1. 設定檔案路徑 (改成你自己電腦上的路徑)
# # ------------------------------------------------------------------
# # 建議先只測一個時間點,例如 t=0 (rest pose),確定疊圖邏輯沒問題,
# # 之後再寫迴圈批次處理多個時間點。
# airway_ply_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.03_11.30.45\geometry\frank_airway_tube_clipped_0.000.ply"
# centerline_csv_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.03_11.30.45\geometry\frank_centerline_0.000.csv"

# # ------------------------------------------------------------------
# # 2. 讀取 airway mesh (.ply 是完整的 3D 表面網格,直接用 pyvista 讀)
# # ------------------------------------------------------------------
# airway_mesh = pv.read(airway_ply_path)
# print("airway mesh:", airway_mesh.n_points, "points,", airway_mesh.n_cells, "faces")

# # ------------------------------------------------------------------
# # 3. 讀取 centerline csv (只是 x,y,z 座標的表格,不是 pyvista 原生格式,
# #    所以要自己組成一條 PolyLine)
# # ------------------------------------------------------------------
# df = pd.read_csv(centerline_csv_path)
# df.columns = [c.strip() for c in df.columns]  # 避免欄位名稱有多餘空白
# centerline_pts = df[df.columns[:3]].to_numpy()  # 取前三欄當 x,y,z
# centerline_poly = pv.lines_from_points(centerline_pts)  # 把散點串成折線
# print("centerline:", centerline_pts.shape[0], "points")

# # ------------------------------------------------------------------
# # 4. 疊圖
# # ------------------------------------------------------------------
# plotter = pv.Plotter(window_size=(1200, 900))

# # airway mesh: 半透明,才看得到裡面的紅線
# plotter.add_mesh(
#     airway_mesh,
#     color="lightblue",
#     opacity=0.35,          # 0=全透明, 1=不透明。airway 一定要透明才看得到內部的線
#     label="airway_tube_clipped"
# )

# # centerline: 不透明的紅線 + 紅點,清楚標出每個取樣點
# plotter.add_mesh(centerline_poly, color="red", line_width=5, label="centerline")
# plotter.add_mesh(pv.PolyData(centerline_pts), color="red", point_size=10,
#                   render_points_as_spheres=True)

# plotter.add_legend(bcolor="white")
# plotter.add_axes()
# plotter.show_grid()

# plotter.show()   # 跳出互動視窗,可以拖曳旋轉檢查紅線是否真的在管子正中間
# """

# """
# 疊圖腳本:把某個時間點的 airway mesh (.stl) 跟 centerline (.csv) 疊在一起顯示
# 用途:視覺檢查 centerline 是否真的穿在 airway 管子的中軸上
# """

# """這邊是plyvista疊圖的範例程式,可以把 airway mesh (.stl) 跟 centerline (.csv) 疊在一起顯示
# """

import pandas as pd
import pyvista as pv

# ------------------------------------------------------------------
# 1. 設定檔案路徑 (改成你自己電腦上的路徑)
# ------------------------------------------------------------------
# 建議先只測一個時間點,例如 t=0 (rest pose),確定疊圖邏輯沒問題,
# 之後再寫迴圈批次處理多個時間點。
airway_stl_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.06_13.08.37\geometry\frank_airway_tube_clipped_0.000.stl"
centerline_csv_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.06_13.08.37\geometry\frank_centerline_0.000.csv"

# ------------------------------------------------------------------
# 2. 讀取 airway mesh (.stl 是完整的 3D 表面網格,直接用 pyvista 讀,
#    pyvista/VTK 原生支援 STL 格式,讀法跟 .ply 完全一樣)
# ------------------------------------------------------------------
airway_mesh = pv.read(airway_stl_path)
print("airway mesh:", airway_mesh.n_points, "points,", airway_mesh.n_cells, "faces")

# ------------------------------------------------------------------
# 3. 讀取 centerline csv (只是 x,y,z 座標的表格,不是 pyvista 原生格式,
#    所以要自己組成一條 PolyLine)
# ------------------------------------------------------------------
df = pd.read_csv(centerline_csv_path)
df.columns = [c.strip() for c in df.columns]  # 避免欄位名稱有多餘空白
centerline_pts = df[df.columns[:3]].to_numpy()  # 取前三欄當 x,y,z
centerline_poly = pv.lines_from_points(centerline_pts)  # 把散點串成折線
print("centerline:", centerline_pts.shape[0], "points")

# ------------------------------------------------------------------
# 4. 疊圖
# ------------------------------------------------------------------
plotter = pv.Plotter(window_size=(1200, 900))

# airway mesh: 半透明,才看得到裡面的紅線
plotter.add_mesh(
    airway_mesh,
    color="lightblue",
    opacity=0.35,          # 0=全透明, 1=不透明。airway 一定要透明才看得到內部的線
    label="airway_tube_clipped"
)

# centerline: 不透明的紅線 + 紅點,清楚標出每個取樣點
plotter.add_mesh(centerline_poly, color="red", line_width=5, label="centerline")
plotter.add_mesh(pv.PolyData(centerline_pts), color="red", point_size=10,
                  render_points_as_spheres=True)

plotter.add_legend(bcolor="white")
plotter.add_axes()
plotter.show_grid()

plotter.show()   # 跳出互動視窗,可以拖曳旋轉檢查紅線是否真的在管子正中間