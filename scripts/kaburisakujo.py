import pandas as pd
import numpy as np # numpyをインポート

# --- ファイルパスと設定 ---
input_csv_file = '/home/matsunaga-h/pickup_ws/angle_arucopose_csv_cleaned/aruco_motor_log_1108_175143_cleaned.csv'
output_csv_file = '/home/matsunaga-h/pickup_ws/angle_arucopose_csv_cleaned/aruco_motor_log_1108_175143_cleaneded.csv'
threshold = 0.01  # 近いと判断する距離（この値を調整できます）

# 1. CSVファイルを読み込む
df = pd.read_csv(input_csv_file)
print(f"処理前の行数: {len(df)}")

# 2. x, y, z の値を元に、どのグループに属するかを計算
df['x_group'] = np.floor(df['x'] / threshold)
df['y_group'] = np.floor(df['y'] / threshold)
df['z_group'] = np.floor(df['z'] / threshold)

# 3. グループを基準に重複を削除（各グループの最初の行だけを残す）
df_cleaned = df.drop_duplicates(subset=['x_group', 'y_group', 'z_group'], keep='first')

# 4. 処理のために追加した一時的なグループ列を削除
df_cleaned = df_cleaned.drop(columns=['x_group', 'y_group', 'z_group'])
print(f"処理後の行数: {len(df_cleaned)}")

# 5. 結果を新しいCSVファイルとして保存
df_cleaned.to_csv(output_csv_file, index=False)

print(f"近接データを削除した結果を '{output_csv_file}' として保存しました。")