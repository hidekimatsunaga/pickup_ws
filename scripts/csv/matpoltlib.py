# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from scipy.spatial.transform import Rotation as R

# # CSV読み込み（列数が多い場合でも読み取れるようにヘッダーなしで読み込む）
# df = pd.read_csv('/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_0710_185439.csv', header=None, skiprows=1)

# # 必要な列だけ抽出（12列目から -> Pythonでは11番目のインデックスから）
# position_orientation = df.iloc[:, 11:18].astype(float)
# position_orientation.columns = ['x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']

# # プロット
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# for i, row in position_orientation.iterrows():
#     x, y, z = row['x'], row['y'], row['z']
#     qx, qy, qz, qw = row['qx'], row['qy'], row['qz'], row['qw']

#     # クォータニオンから方向ベクトルを計算（x軸方向）
#     rot = R.from_quat([qx, qy, qz, qw])
#     direction = rot.apply([1, 0, 0])  # x軸ベクトルを回転

#     # 矢印で向きを描画
#     ax.quiver(x, y, z, direction[0], direction[1], direction[2], length=0.1, normalize=True)

# # ラベル
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# # 原点に矢印（x軸方向）を描く
# ax.quiver(0, 0, 0, 1, 0, 0, length=0.2, color='red', linewidth=2, normalize=True)
# ax.quiver(0, 0, 0, 0, 1, 0, length=0.2, color='green', linewidth=2, normalize=True)
# ax.quiver(0, 0, 0, 0, 0, 1, length=0.2, color='blue', linewidth=2, normalize=True)

# # 原点に大きな点を描く
# ax.scatter(0, 0, 0, color='black', s=50, label='origin')

# # 凡例追加（オプション）
# ax.legend()

# plt.show()
# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D

# # CSV読み込み（ヘッダーなし、1行目スキップ）
# df = pd.read_csv('/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_0714_001835.csv', header=None, skiprows=1)

# # 位置情報のみ抽出（x, y, z）
# position = df.iloc[:, 12:15].astype(float)
# position.columns = ['x', 'y', 'z']

# # プロット
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # 点を描画
# ax.scatter(position['x'], position['y'], position['z'], color='blue', s=10)

# # ラベル
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')

# # 原点に矢印（x, y, z軸）を描く
# ax.quiver(0, 0, 0, 1, 0, 0, length=0.2, color='red', linewidth=2, normalize=True)
# ax.quiver(0, 0, 0, 0, 1, 0, length=0.2, color='green', linewidth=2, normalize=True)
# ax.quiver(0, 0, 0, 0, 0, 1, length=0.2, color='blue', linewidth=2, normalize=True)


# # 原点に大きな点
# ax.scatter(0, 0, 0, color='black', s=50, label='origin')

# # 凡例
# ax.legend()

# plt.show()
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# --- 正解の値をここに設定してください ---
correct_x = -0.14
correct_y = -0.02
correct_z = 0.62
# ------------------------------------

# CSV読み込み（ヘッダーなし、1行目スキップ）
# ファイルパスはご自身の環境に合わせて修正してください。
df = pd.read_csv('/home/matsunaga-h/pickup_ws/angle_arucopose_csv_cleaned/aruco_motor_log_1208_related_cleaned_deduped.csv', header=None, skiprows=1)
# df = pd.read_csv('/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_1026_031137.csv', header=None, skiprows=1)
# df = pd.read_csv('/home/matsunaga-h/pickup_ws/angle_arucopose_csv_cleaned/aruco_motor_log_1108_193312_cleaned.csv', header=None, skiprows=1)
# df = pd.read_csv('/home/matsunaga-h/pickup_ws/rosbag/lookup_table/aruco_motor_log_1020_125332_cleaned.csv', header=None, skiprows=1)

# 位置情報のみ抽出（x, y, z）
position = df.iloc[:, 12:15].astype(float)
position.columns = ['x', 'y', 'z']

# プロット
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 点を描画
ax.scatter(position['x'], position['y'], position['z'], color='blue', s=10, label='Measured Data')

# ★★★ 正解の値を強調してプロット ★★★
ax.scatter(correct_x, correct_y, correct_z, color='purple', s=150, marker='*', label='Correct Value', depthshade=False)


# ラベル
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# 原点に矢印（x, y, z軸）を描く
ax.quiver(0, 0, 0, 1, 0, 0, length=0.2, color='red', linewidth=2, normalize=True)
ax.quiver(0, 0, 0, 0, 1, 0, length=0.2, color='green', linewidth=2, normalize=True)
ax.quiver(0, 0, 0, 0, 0, 1, length=0.2, color='blue', linewidth=2, normalize=True)


# 原点に大きな点
ax.scatter(0, 0, 0, color='black', s=50, label='Origin')

# 凡例
ax.legend()

plt.show()