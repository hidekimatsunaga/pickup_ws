import pandas as pd

# 読み込むCSVファイルのパスを指定
input_filepath = '/home/matsunaga-h/pickup_ws/angle_arucopose_csv/aruco_motor_log_1108_193312.csv'
# 保存するCSVファイルのパスを指定
output_filepath = '/home/matsunaga-h/pickup_ws/angle_arucopose_csv_cleaned/aruco_motor_log_1108_193312_cleaned.csv'

# CSVファイルを読み込む
try:
    df = pd.read_csv(input_filepath)
    print(f"元のデータ数: {len(df)} 行")

    # 条件1: 'x'列と'y'列が両方とも0の行を削除
    condition1 = (df['x'] != 0) | (df['y'] != 0)
    
    # 条件2: 'id'列が0か1の行のみを選択
    condition2 = df['marker_id'].isin([0, 1, 2])

    # 2つの条件を両方とも満たす行だけを抽出
    df_cleaned = df[condition1 & condition2]

    print(f"削除後のデータ数: {len(df_cleaned)} 行")

    # 結果を新しいCSVファイルに保存
    df_cleaned.to_csv(output_filepath, index=False)
    print(f"処理結果を {output_filepath} に保存しました。")

except FileNotFoundError:
    print(f"エラー: {input_filepath} が見つかりません。")
except KeyError:
    print("エラー: CSVファイルに 'x', 'y', 'marker_id' いずれかの列が見つかりません。")