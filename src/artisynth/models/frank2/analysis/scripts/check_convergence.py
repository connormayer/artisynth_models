import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import glob
import re
import os

geom_dir = r"C:\Users\yuron\git\artisynth_models\src\artisynth\models\frank2\output\temp_2026.09.01_22.21.42_r_snd\geometry"

# 抓出 0.300 到 0.395(或到你模擬的最後一步)之間所有 centerline csv
files = sorted(glob.glob(os.path.join(geom_dir, "frank_centerline_0.0*.csv")) + 
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.1*.csv")) +
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.2*.csv")) +
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.3*.csv")) +
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.4*.csv")) +
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.5*.csv")) +
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.6*.csv")) +
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.7*.csv"))+
               glob.glob(os.path.join(geom_dir, "frank_centerline_0.8*.csv")) )

# 從檔名擷取時間值,並排序
def get_time(f):
    m = re.search(r"frank_centerline_([\d.]+)\.csv", f)
    return float(m.group(1))

files = sorted(files, key=get_time)
times = [get_time(f) for f in files]

# 讀取每個時間點的頂點座標
all_pts = []
for f in files:
    df = pd.read_csv(f)
    df.columns = [c.strip() for c in df.columns]
    pts = df[df.columns[:3]].to_numpy()
    all_pts.append(pts)

# 計算相鄰兩個 timestep 之間,每個頂點移動的距離(RMS)
displacements = []
for i in range(1, len(all_pts)):
    diff = all_pts[i] - all_pts[i-1]
    rms = np.sqrt(np.mean(np.sum(diff**2, axis=1)))
    displacements.append(rms)

# 畫圖
plt.figure(figsize=(8,5))
plt.plot(times[1:], displacements, marker='o')
plt.xlabel("Time (s)")
plt.ylabel("RMS displacement between consecutive timesteps (m)")
plt.title("Centerline convergence check (0.0s - 0.895s)")
plt.grid(True)
plt.savefig("convergence_check.png", dpi=150)
plt.show()