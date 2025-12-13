#!/usr/bin/env python3
"""
既存CSVの各行について、motor1-9を固定したまま
motor10だけを掃引して先端位置を予測
"""

import pandas as pd
import numpy as np
from scipy import stats
import json
import sys
import os


def load_or_analyze_linearity(csv_path_reference):
    """
    marker_idごとの線形関係式を読み込む（キャッシュがあれば使う）
    返り値: {marker_id: {axis: {slope, intercept, r_squared, ...}}}
    """
    
    json_path = csv_path_reference.replace('.csv', '_motor10_linear_fit_all.json')
    
    # キャッシュから読み込み
    if os.path.exists(json_path):
        print(f"キャッシュから線形関係式を読み込み: {json_path}\n")
        with open(json_path, 'r') as f:
            all_fit = json.load(f)
        # キーを整数に変換
        return {int(k): v for k, v in all_fit.items()}
    
    # 古いフォーマットのキャッシュをチェック
    old_json_path = csv_path_reference.replace('.csv', '_motor10_linear_fit.json')
    if os.path.exists(old_json_path):
        print(f"古いフォーマットのキャッシュを使用: {old_json_path}\n")
        with open(old_json_path, 'r') as f:
            single_fit = json.load(f)
        # marker_id=0として使用
        return {0: single_fit}
    
    print(f"エラー: 線形関係式のキャッシュが見つかりません")
    print(f"先に analyze_motor10_linearity.py を実行してください")
    return None


def predict_tip_position(motor10_value, marker_id, all_linear_fit):
    """
    marker_idに応じた線形関係式を使ってホース先端位置を予測
    """
    if marker_id not in all_linear_fit:
        # デフォルトでID=0を使用
        marker_id = list(all_linear_fit.keys())[0]
    
    linear_fit = all_linear_fit.get(marker_id)
    if linear_fit is None:
        return None
    
    predicted = {}
    for axis in ['x', 'y', 'z']:
        slope = linear_fit[axis]['slope']
        intercept = linear_fit[axis]['intercept']
        predicted[axis] = slope * motor10_value + intercept
    return predicted


def sweep_motor10(input_csv, reference_csv, output_csv, sweep_target=-10000, motor10_step=360):
    """
    input_csv の各行について、元のmotor10値から sweep_target まで掃引して先端位置を予測
    """
    
    print(f"{'='*60}")
    print(f"Motor10掃引予測スクリプト")
    print(f"{'='*60}\n")
    
    # 線形関係式を取得
    all_linear_fit = load_or_analyze_linearity(reference_csv)
    if not all_linear_fit:
        return
    
    print(f"利用可能なmarker_id: {sorted(all_linear_fit.keys())}\n")
    
    # 入力CSVを読み込み
    print(f"入力CSVを読み込み中: {input_csv}")
    df = pd.read_csv(input_csv, header=None, skiprows=1)
    
    # ヘッダー行を取得
    with open(input_csv, 'r') as f:
        header_line = f.readline().strip()
    
    print(f"元のデータ行数: {len(df)}\n")
    print(f"Motor10掃引設定:")
    print(f"  各行の元のmotor10値から {sweep_target}° まで掃引")
    print(f"  刻み: {motor10_step}°\n")
    
    # 出力データを格納するリスト
    output_rows = []
    
    print("先端位置を予測中...")
    total_sweep_steps = 0
    processed = 0
    
    for idx, row in df.iterrows():
        # 元の行データを取得
        timestamp = row.iloc[0]
        motor1_9 = row.iloc[1:10].values  # motor1-9
        original_motor10 = float(row.iloc[10])  # 元のmotor10値
        marker_id = int(row.iloc[11])
        
        # 測定値（もしあれば）
        measured_x = row.iloc[12] if len(row) > 12 else np.nan
        measured_y = row.iloc[13] if len(row) > 13 else np.nan
        measured_z = row.iloc[14] if len(row) > 14 else np.nan
        qx = row.iloc[15] if len(row) > 15 else np.nan
        qy = row.iloc[16] if len(row) > 16 else np.nan
        qz = row.iloc[17] if len(row) > 17 else np.nan
        qw = row.iloc[18] if len(row) > 18 else np.nan
        
        # 測定値がない場合はスキップ
        if np.isnan(measured_x) or np.isnan(measured_y) or np.isnan(measured_z):
            print(f"  警告: 行{idx}に測定値がないためスキップ")
            continue
        
        # この行の掃引範囲を生成（元のmotor10から sweep_target まで）
        # より負の方向に進む
        if original_motor10 > sweep_target:
            # 例: -5000 → -10000 なら、-5000, -5360, -5720, ..., -10000
            # 元の行（差分=0）は含めない
            motor10_values = np.arange(original_motor10 - abs(motor10_step), sweep_target - motor10_step, -abs(motor10_step))
        else:
            # 既に sweep_target より小さい場合はスキップ
            continue
        
        total_sweep_steps += len(motor10_values)
        
        # motor10を掃引
        for motor10 in motor10_values:
            # motor10の差分を計算
            motor10_delta = motor10 - original_motor10
            
            # 相対的な予測（測定値 + slope × 差分）
            linear_fit = all_linear_fit.get(marker_id)
            if linear_fit is None:
                # デフォルトでID=0を使用
                linear_fit = all_linear_fit.get(list(all_linear_fit.keys())[0])
            
            if linear_fit is None:
                pred_x, pred_y, pred_z = np.nan, np.nan, np.nan
            else:
                # 切片は使わず、傾きだけで相対変化を計算
                pred_x = measured_x + linear_fit['x']['slope'] * motor10_delta
                pred_y = measured_y + linear_fit['y']['slope'] * motor10_delta
                pred_z = measured_z + linear_fit['z']['slope'] * motor10_delta
            
            # 新しい行を作成
            new_row = [
                timestamp,
                *motor1_9,  # motor1-9
                motor10,    # motor10（掃引値）
                marker_id,
                pred_x, pred_y, pred_z,  # 予測値
                qx, qy, qz, qw           # quaternion
            ]
            output_rows.append(new_row)
            
            processed += 1
            if processed % 1000 == 0:
                print(f"  進捗: {processed}/{total_sweep_steps} 掃引ステップ")
    
    print(f"  完了: {processed}/{total_sweep_steps} 掃引ステップ\n")
    
    # DataFrameに変換
    columns = [
        'timestamp',
        'motor1', 'motor2', 'motor3', 'motor4', 'motor5',
        'motor6', 'motor7', 'motor8', 'motor9', 'motor10',
        'marker_id',
        'x', 'y', 'z',
        'qx', 'qy', 'qz', 'qw'
    ]
    
    df_output = pd.DataFrame(output_rows, columns=columns)
    
    # CSVに保存
    df_output.to_csv(output_csv, index=False)
    
    print(f"{'='*60}")
    print(f"結果をCSVに保存: {output_csv}")
    print(f"{'='*60}")
    print(f"元の行数: {len(df)}")
    print(f"総掃引ステップ数: {total_sweep_steps}")
    print(f"出力行数: {len(df_output)}")
    print(f"{'='*60}\n")
    
    # 統計情報
    print("予測値の統計:")
    for axis in ['x', 'y', 'z']:
        predicted = df_output[f'{axis}'].values
        valid_mask = ~np.isnan(predicted)
        if np.sum(valid_mask) > 0:
            predicted_valid = predicted[valid_mask]
            print(f"  {axis.upper()}座標: {predicted_valid.min():.6f} ~ {predicted_valid.max():.6f} [m]")
    print()


def main():
    if len(sys.argv) < 3:
        print("使用方法:")
        print("  python3 motor10_sweep_prediction.py <input_csv> <reference_csv> [output_csv] [sweep_target] [step]")
        print("\n引数:")
        print("  input_csv     : 入力CSV（各行のmotor1-9を固定）")
        print("  reference_csv : 線形関係式の参照CSV（cleanedデータ）")
        print("  output_csv    : 出力CSV（省略時は自動生成）")
        print("  sweep_target  : motor10の掃引目標値（デフォルト: -10000）")
        print("  step          : motor10刻み幅（デフォルト: 360）")
        print("\n動作:")
        print("  各行の元のmotor10値から sweep_target まで負の方向に掃引")
        print("\n例:")
        print("  python3 motor10_sweep_prediction.py \\")
        print("    ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_203430.csv \\")
        print("    ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_205211_cleaned_id0.csv")
        return
    
    input_csv = sys.argv[1]
    reference_csv = sys.argv[2]
    
    # 出力ファイル名を自動生成
    if len(sys.argv) > 3 and not sys.argv[3].startswith('-'):
        output_csv = sys.argv[3]
        arg_offset = 4
    else:
        base, ext = os.path.splitext(input_csv)
        output_csv = base + '_motor10_sweep' + ext
        arg_offset = 3
    
    # motor10の掃引設定
    sweep_target = float(sys.argv[arg_offset]) if len(sys.argv) > arg_offset else -10000
    motor10_step = float(sys.argv[arg_offset + 1]) if len(sys.argv) > arg_offset + 1 else 360
    
    if not os.path.exists(input_csv):
        print(f"エラー: ファイルが見つかりません: {input_csv}")
        return
    
    if not os.path.exists(reference_csv):
        print(f"エラー: ファイルが見つかりません: {reference_csv}")
        return
    
    sweep_motor10(input_csv, reference_csv, output_csv, sweep_target, motor10_step)


if __name__ == '__main__':
    main()
