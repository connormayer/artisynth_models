import pyvista as pv
import glob

# 1. 取得所有 vtk 檔案列表，並確保它們是排序好的
file_list = sorted(glob.glob("C:\\Users\\yuron\\git\\artisynth_models\\src\\artisynth\\models\\frank2\\output\\temp_2026.07.16_12.32.24\\geometry\\frank_tongue_*.vtk"))

# 2. 初始化 Plotter
plotter = pv.Plotter(off_screen=True)
plotter.open_gif("combined_animation.gif", fps=10) # 設定每秒幀數

# 3. 迴圈處理每個檔案
for filename in file_list:
    # 清空當前 Plotter 的內容
    plotter.clear()
    
    # 讀取並加入當前的網格
    mesh = pv.read(filename)
    plotter.add_mesh(mesh)
    
    # 強制重繪並寫入一個影格
    plotter.write_frame()

# 4. 關閉 Plotter
plotter.close()