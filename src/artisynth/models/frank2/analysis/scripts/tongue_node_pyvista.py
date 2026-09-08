import pyvista as pv

# 改成你實際的檔案路徑
filepath = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.07.02_14.37.16\geometry\frank_tongue_0.000.vtk"

mesh = pv.read(filepath)

target_nodes = [919, 873, 908, 927, 847]

print(f"這個 mesh 總共有 {mesh.n_points} 個 node\n")

for node_id in target_nodes:
    if node_id < mesh.n_points:
        pos = mesh.points[node_id]
        print(f"Node {node_id}: x={pos[0]:.5f}, y={pos[1]:.5f}, z={pos[2]:.5f}")
    else:
        print(f"⚠️ Node {node_id} 超出範圍(總共只有 {mesh.n_points} 個點)")

# 視覺化:把整個舌頭畫出來,並且把這幾個參考點標成紅色大球,方便你肉眼確認位置合不合理
plotter = pv.Plotter()
plotter.add_mesh(mesh, color="lightpink", opacity=0.6, label="Tongue mesh")

points_to_highlight = mesh.points[target_nodes]
plotter.add_points(points_to_highlight, color="red", point_size=20, render_points_as_spheres=True)

plotter.add_legend()
plotter.show()