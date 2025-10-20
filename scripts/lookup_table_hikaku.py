import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import os # ファイル名をラベルに使うためにインポート

# --- データ準備 ---
start_point = {'x': -0.3, 'y': -0.06, 'z': 0.9}
end_point   = {'x': 0.3,  'y': -0.06, 'z': 0.9}

# ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
# --- 1. 読み込むCSVファイルのリストを定義 ---
csv_files = [
    '/home/matsunaga-h/pickup_ws/rosbag/lookup_table/lookuptable_cleaned.csv',
    '/home/matsunaga-h/pickup_ws/rosbag/nearest_/nearest_cleaned.csv',  # 2つ目のファイルパス
]

# --- 2. 各グラフの色を定義 ---
colors = ['orange', 'purple', 'brown', 'pink', 'gray', 'cyan']
# ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★


# --- グラフの準備 ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')


# --- 元の直線と点をプロット ---
ax.plot(
    [start_point['x'], end_point['x']],
    [start_point['y'], end_point['y']],
    [start_point['z'], end_point['z']],
    color='blue', marker='o', label='Initial Line'
)
ax.scatter(start_point['x'], start_point['y'], start_point['z'], color='green', s=50, label='Start Point')
ax.scatter(end_point['x'], end_point['y'], end_point['z'], color='red', s=50, label='End Point')


# ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
# --- 3. forループを使って複数のCSVをプロット ---
for i, filepath in enumerate(csv_files):
    try:
        df = pd.read_csv(filepath, header=0)

        csv_x = df['x'].to_numpy()
        csv_y = df['y'].to_numpy()
        csv_z = df['z'].to_numpy()
        
        # ファイル名だけを抽出してラベルにする
        filename = os.path.basename(filepath)

        # 順番に色を変えながらプロット
        ax.plot(csv_x, csv_y, csv_z, color=colors[i % len(colors)], label=f'Trajectory: {filename}')
        print(f"'{filepath}' の読み込みと描画に成功しました。")

    except FileNotFoundError:
        print(f"エラー: ファイル '{filepath}' が見つかりません。スキップします。")
    except Exception as e:
        print(f"エラー: '{filepath}' の処理中に問題が発生しました: {e}")
# ★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★


# --- グラフの調整 ---
x_limits = ax.get_xlim3d()
y_limits = ax.get_ylim3d()
z_limits = ax.get_zlim3d()

x_center = np.mean(x_limits)
y_center = np.mean(y_limits)
z_center = np.mean(z_limits)

x_range = abs(x_limits[1] - x_limits[0])
y_range = abs(y_limits[1] - y_limits[0])
z_range = abs(z_limits[1] - z_limits[0])

plot_radius = 0.5 * max([x_range, y_range, z_range])

ax.set_xlim3d([x_center - plot_radius, x_center + plot_radius])
ax.set_ylim3d([y_center - plot_radius, y_center + plot_radius])
ax.set_zlim3d([z_center - plot_radius, z_center + plot_radius])

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')
ax.legend()


# --- グラフを表示 ---
plt.show()