import pandas as pd
import pyvista as pv

# 1. 設定檔案路徑
ply_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\geometry\airway_complete_tube_clipped.ply"
csv1_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\geometry\centerlinePoints.csv"
csv2_path = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.11_00.02.39\geometry\frank_centerline_0.000.csv"

# 2. 載入 PLY 網格模型
print("正在載入 PLY 模型...")
mesh = pv.read(ply_path)

# 3. 讀取第一組 CSV 點 (欄位為 Points:0, Points:1, Points:2)
print("正在載入 centerlinePoints...")
df1 = pd.read_csv(csv1_path)
points1 = df1[['Points:0', 'Points:1', 'Points:2']].values
poly_points1 = pv.PolyData(points1)

# 4. 讀取第二組 CSV 點 (欄位為 x, y, z)
print("正在載入 frank_centerline...")
df2 = pd.read_csv(csv2_path)
points2 = df2[['x', 'y', 'z']].values
poly_points2 = pv.PolyData(points2)

# 5. 建立 PyVista 繪圖視窗
plotter = pv.Plotter()

# 💡 加入 3D PLY 模型 (設定半透明度 opacity 方便觀察內部的點)
plotter.add_mesh(
    mesh, 
    color="lightgray", 
    opacity=0.4, 
    style="surface", 
    label="airway_complete_tube_clipped.ply"
)

# 💡 加入第一組點 (紅色)
plotter.add_mesh(
    poly_points1, 
    color="red", 
    point_size=8, 
    render_points_as_spheres=True, 
    label="Centerline at 0.000s (output)"
)

# 💡 加入第二組點 (藍色)
plotter.add_mesh(
    poly_points2, 
    color="blue", 
    point_size=8, 
    render_points_as_spheres=True, 
    label="Centerline Points (input)"
)

# 6. 優化視覺效果與輔助工具
plotter.add_legend(bcolor="white", size=(0.2, 0.2)) # 顯示右上角圖例
plotter.add_axes()                                 # 顯示左下角 XYZ 座標軸
plotter.show_grid()                                # 顯示背景空間網格面

# 7. 開啟互動視窗
print("啟動 3D 互動視窗...")
plotter.show()