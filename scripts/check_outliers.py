#!/usr/bin/env python3
"""
aruco_motor_log から marker_id ごとの外れ値を検出・除去
【重要】残差（Motor10との線形関係からのズレ）ベースで検出
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import sys
import os


def analyze_data_distribution(csv_path):
    """各marker_idのデータ分布を分析"""
    
    # CSV読み込み
    df = pd.read_csv(csv_path, header=None, skiprows=1)
    
    # 必要な列を抽出
    marker_ids = df.iloc[:, 11].astype(int)
    motor10 = df.iloc[:, 10].astype(float)
    position = df.iloc[:, 12:15].astype(float)
    position.columns = ['x', 'y', 'z']
    
    unique_ids = sorted(marker_ids.unique())
    
    print(f"\n{'='*60}")
    print(f"マーカーID別のデータ分布分析")
    print(f"{'='*60}\n")
    
    # 各marker_idごとの統計
    for mid in unique_ids:
        mask = marker_ids == mid
        motor10_data = motor10[mask].values
        position_data = position[mask].values
        
        print(f"【Marker ID = {mid}】")
        print(f"  データ点数: {len(motor10_data)}")
        print(f"  Motor10範囲: {motor10_data.min():.2f}° ~ {motor10_data.max():.2f}°")
        print(f"  X範囲: {position_data[:, 0].min():.6f} ~ {position_data[:, 0].max():.6f} [m]")
        print(f"  Y範囲: {position_data[:, 1].min():.6f} ~ {position_data[:, 1].max():.6f} [m]")
        print(f"  Z範囲: {position_data[:, 2].min():.6f} ~ {position_data[:, 2].max():.6f} [m]")
        print()
    
    # プロット
    fig, axes = plt.subplots(3, len(unique_ids), figsize=(5*len(unique_ids), 12))
    fig.suptitle('Motor10 vs 各座標（marker_idごと）', fontsize=14)
    
    if len(unique_ids) == 1:
        axes = axes.reshape(3, 1)
    
    for col, mid in enumerate(unique_ids):
        mask = marker_ids == mid
        motor10_data = motor10[mask].values
        position_data = position[mask].values
        
        for row, (axis_idx, axis_name) in enumerate([(0, 'X'), (1, 'Y'), (2, 'Z')]):
            ax = axes[row, col] if len(unique_ids) > 1 else axes[row]
            
            ax.scatter(motor10_data, position_data[:, axis_idx], s=30, alpha=0.6)
            ax.set_xlabel('Motor10 [°]')
            ax.set_ylabel(f'{axis_name} [m]')
            ax.set_title(f'ID={mid}: {axis_name}座標 (n={len(motor10_data)})')
            ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_path = csv_path.replace('.csv', '_distribution_analysis.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"プロットを保存: {output_path}\n")
    plt.close()  # グラフを閉じる（ウィンドウを開かない）
    
    return unique_ids


def detect_outliers(csv_path, marker_id, method='iqr', threshold=1.5, motor10_min=None, motor10_max=None):
    """
    【残差ベースの外れ値検出】
    1. motor10の範囲チェック（motor10_min, motor10_maxで異常値を除外）
    2. motor10 vs 各座標で線形回帰 → 期待値を計算
    3. 実測値 - 期待値 = 残差
    4. 残差に対してIQR/Z-scoreを適用 → 異常な残差を外れ値とする
    
    method: 'iqr' (四分位法) または 'zscore'
    threshold: IQR法は係数（1.5=通常、3.0=厳しい）、Z-score法は閾値
    motor10_min, motor10_max: motor10の有効範囲（Noneの場合は自動判定）
    """
    
    df = pd.read_csv(csv_path, header=None, skiprows=1)
    
    marker_ids = df.iloc[:, 11].astype(int)
    motor10 = df.iloc[:, 10].astype(float)
    position = df.iloc[:, 12:15].astype(float)
    position.columns = ['x', 'y', 'z']
    
    mask = marker_ids == marker_id
    original_df = df[mask].reset_index(drop=True)
    motor10_data = motor10[mask].values
    position_data = position[mask].values
    
    # motor10の範囲で異常値を除外
    outlier_mask = np.zeros(len(original_df), dtype=bool)
    
    if motor10_min is None or motor10_max is None:
        # 自動判定：Q1-3*IQR, Q3+3*IQRを範囲とする
        Q1 = np.percentile(motor10_data, 25)
        Q3 = np.percentile(motor10_data, 75)
        IQR = Q3 - Q1
        auto_min = Q1 - 3 * IQR
        auto_max = Q3 + 3 * IQR
        motor10_min = auto_min if motor10_min is None else motor10_min
        motor10_max = auto_max if motor10_max is None else motor10_max
    
    motor10_outliers = (motor10_data < motor10_min) | (motor10_data > motor10_max)
    outlier_mask |= motor10_outliers
    
    n_motor10_outliers = np.sum(motor10_outliers)
    if n_motor10_outliers > 0:
        print(f"\n【Motor10範囲チェック】")
        print(f"  有効範囲: [{motor10_min:.2f}°, {motor10_max:.2f}°]")
        print(f"  範囲外のデータ: {n_motor10_outliers}個")
        outlier_indices = np.where(motor10_outliers)[0]
        for idx in outlier_indices[:5]:
            print(f"    Index {idx}: motor10={motor10_data[idx]:.2f}°")
        if len(outlier_indices) > 5:
            print(f"    ... 他 {len(outlier_indices) - 5}個")
        print()
    
    print(f"\n{'='*60}")
    print(f"【残差ベース外れ値検出】Marker ID={marker_id}")
    print(f"方法: {method.upper()} (threshold={threshold})")
    print(f"{'='*60}\n")
    
    outlier_mask = np.zeros(len(original_df), dtype=bool)
    residuals_by_axis = {}
    
    # 各軸について線形回帰 → 残差を計算
    for axis in ['x', 'y', 'z']:
        axis_idx = ['x', 'y', 'z'].index(axis)
        position_values = position_data[:, axis_idx]
        
        # 線形回帰
        slope, intercept, r_value, _, _ = stats.linregress(motor10_data, position_values)
        
        # 期待値
        expected_values = slope * motor10_data + intercept
        
        # 残差
        residuals = position_values - expected_values
        residuals_by_axis[axis] = residuals
        
        print(f"【{axis.upper()}座標】")
        print(f"  y = {slope:.6f} × motor10 + {intercept:.6f}")
        print(f"  R² = {r_value**2:.6f}")
        print(f"  残差 min={residuals.min():.6f}, max={residuals.max():.6f}, std={residuals.std():.6f}")
        
        # 残差に基づいて外れ値判定
        if method == 'iqr':
            Q1 = np.percentile(residuals, 25)
            Q3 = np.percentile(residuals, 75)
            IQR = Q3 - Q1
            
            lower_bound = Q1 - threshold * IQR
            upper_bound = Q3 + threshold * IQR
            
            axis_outliers = (residuals < lower_bound) | (residuals > upper_bound)
            outlier_mask |= axis_outliers
            
            n_outliers = np.sum(axis_outliers)
            print(f"  IQR法: [{lower_bound:.6f}, {upper_bound:.6f}] → {n_outliers}個の外れ値")
        
        elif method == 'zscore':
            z_scores = np.abs(stats.zscore(residuals))
            axis_outliers = z_scores > threshold
            outlier_mask |= axis_outliers
            
            n_outliers = np.sum(axis_outliers)
            print(f"  Z-score法: |z| > {threshold} → {n_outliers}個の外れ値")
        
        print()
    
    n_total_outliers = np.sum(outlier_mask)
    
    print(f"{'='*60}")
    print(f"合計: {n_total_outliers}個の外れ値を検出")
    print(f"データ点数: {len(original_df)} → {len(original_df) - n_total_outliers}")
    print(f"{'='*60}\n")
    
    # 外れ値の詳細を表示
    if n_total_outliers > 0:
        print("外れ値のあるインデックス:")
        outlier_indices = np.where(outlier_mask)[0]
        for idx in outlier_indices[:10]:
            print(f"  Index {idx}: motor10={motor10_data[idx]:.2f}°, "
                  f"x={position_data[idx, 0]:.6f}, "
                  f"y={position_data[idx, 1]:.6f}, "
                  f"z={position_data[idx, 2]:.6f}")
        if len(outlier_indices) > 10:
            print(f"  ... 他 {len(outlier_indices) - 10}個")
    
    return outlier_mask, original_df


def save_cleaned_csv(csv_path, marker_id, output_path=None, method='iqr', threshold=1.5, motor10_min=None, motor10_max=None):
    """【残差ベース】外れ値を除去したCSVを保存して、比較グラフを表示"""
    
    df = pd.read_csv(csv_path, header=None, skiprows=1)
    
    marker_ids = df.iloc[:, 11].astype(int)
    motor10 = df.iloc[:, 10].astype(float)
    position = df.iloc[:, 12:15].astype(float)
    position.columns = ['x', 'y', 'z']
    
    mask = marker_ids == marker_id
    original_df = df[mask].reset_index(drop=True)
    motor10_data = motor10[mask].values
    position_data = position[mask].values
    
    print(f"\n{'='*60}")
    print(f"外れ値除去済みCSV作成: Marker ID={marker_id}")
    print(f"【残差ベース検出】方法: {method.upper()}")
    print(f"{'='*60}\n")
    
    outlier_mask = np.zeros(len(original_df), dtype=bool)
    
    # motor10の範囲で異常値を除外
    if motor10_min is None or motor10_max is None:
        # 自動判定：Q1-3*IQR, Q3+3*IQRを範囲とする
        Q1 = np.percentile(motor10_data, 25)
        Q3 = np.percentile(motor10_data, 75)
        IQR = Q3 - Q1
        auto_min = Q1 - 3 * IQR
        auto_max = Q3 + 3 * IQR
        motor10_min = auto_min if motor10_min is None else motor10_min
        motor10_max = auto_max if motor10_max is None else motor10_max
    
    motor10_outliers = (motor10_data < motor10_min) | (motor10_data > motor10_max)
    outlier_mask |= motor10_outliers
    
    n_motor10_outliers = np.sum(motor10_outliers)
    if n_motor10_outliers > 0:
        print(f"【Motor10範囲チェック】")
        print(f"  有効範囲: [{motor10_min:.2f}°, {motor10_max:.2f}°]")
        print(f"  範囲外のデータ: {n_motor10_outliers}個")
        outlier_indices = np.where(motor10_outliers)[0]
        for idx in outlier_indices[:5]:
            print(f"    Index {idx}: motor10={motor10_data[idx]:.2f}°")
        if len(outlier_indices) > 5:
            print(f"    ... 他 {len(outlier_indices) - 5}個")
        print()
    
    linear_fits = {}  # 線形回帰の係数を保存
    
    # 各軸について線形回帰 → 残差を計算
    for axis in ['x', 'y', 'z']:
        axis_idx = ['x', 'y', 'z'].index(axis)
        position_values = position_data[:, axis_idx]
        
        # 線形回帰
        slope, intercept, r_value, _, _ = stats.linregress(motor10_data, position_values)
        linear_fits[axis] = {'slope': slope, 'intercept': intercept, 'r2': r_value**2}
        
        # 期待値と残差
        expected_values = slope * motor10_data + intercept
        residuals = position_values - expected_values
        
        print(f"【{axis.upper()}座標】")
        print(f"  y = {slope:.6f} × motor10 + {intercept:.6f}")
        print(f"  R² = {r_value**2:.6f}")
        
        # 残差に基づいて外れ値判定
        if method == 'iqr':
            Q1 = np.percentile(residuals, 25)
            Q3 = np.percentile(residuals, 75)
            IQR = Q3 - Q1
            
            lower_bound = Q1 - threshold * IQR
            upper_bound = Q3 + threshold * IQR
            
            axis_outliers = (residuals < lower_bound) | (residuals > upper_bound)
            outlier_mask |= axis_outliers
            
            n_outliers = np.sum(axis_outliers)
            print(f"  IQR法: [{lower_bound:.6f}, {upper_bound:.6f}] → {n_outliers}個の外れ値")
        
        elif method == 'zscore':
            z_scores = np.abs(stats.zscore(residuals))
            axis_outliers = z_scores > threshold
            outlier_mask |= axis_outliers
            
            n_outliers = np.sum(axis_outliers)
            print(f"  Z-score法: |z| > {threshold} → {n_outliers}個の外れ値")
        
        print()
    
    n_total_outliers = np.sum(outlier_mask)
    
    print(f"{'='*60}")
    print(f"合計: {n_total_outliers}個の外れ値を検出")
    print(f"データ点数: {len(original_df)} → {len(original_df) - n_total_outliers}")
    print(f"{'='*60}\n")
    
    # 外れ値の詳細を表示
    if n_total_outliers > 0:
        print("除去される外れ値のインデックス:")
        outlier_indices = np.where(outlier_mask)[0]
        for idx in outlier_indices[:10]:
            print(f"  Index {idx}: motor10={motor10_data[idx]:.2f}°, "
                  f"x={position_data[idx, 0]:.6f}, "
                  f"y={position_data[idx, 1]:.6f}, "
                  f"z={position_data[idx, 2]:.6f}")
        if len(outlier_indices) > 10:
            print(f"  ... 他 {len(outlier_indices) - 10}個\n")
    
    # 外れ値を除外
    cleaned_df = original_df[~outlier_mask].reset_index(drop=True)
    cleaned_motor10 = motor10_data[~outlier_mask]
    cleaned_position = position_data[~outlier_mask]
    
    if output_path is None:
        base, ext = os.path.splitext(csv_path)
        output_path = base + f'_cleaned_id{marker_id}' + ext
    
    # CSVを保存（元のフォーマットを保持）
    with open(csv_path, 'r') as f:
        header_line = f.readline().strip()
    
    with open(output_path, 'w') as f:
        f.write(header_line + '\n')
        cleaned_df.to_csv(f, index=False, header=False)
    
    print(f"✅ クリーニング済みCSVを保存: {output_path}\n")
    
    # 【比較グラフを作成】除去前後を視覚化
    print(f"除去前後のグラフを作成中...\n")
    
    fig, axes = plt.subplots(3, 3, figsize=(16, 12))
    fig.suptitle(f'残差ベース外れ値除去: 除去前後の比較 (Marker ID={marker_id})', fontsize=14)
    
    for col, (axis_idx, axis_name) in enumerate([(0, 'X'), (1, 'Y'), (2, 'Z')]):
        # 【列1】除去前：Motor10 vs 位置座標（外れ値をハイライト）
        ax1 = axes[col, 0]
        ax1.scatter(motor10_data, position_data[:, axis_idx], s=30, alpha=0.6, 
                   color='blue', label='Valid data')
        
        if n_total_outliers > 0:
            outlier_indices = np.where(outlier_mask)[0]
            ax1.scatter(motor10_data[outlier_indices], position_data[outlier_indices, axis_idx],
                       s=50, alpha=0.8, color='red', marker='x', linewidth=2, label='Outliers')
        
        slope = linear_fits[axis_name.lower()]['slope']
        intercept = linear_fits[axis_name.lower()]['intercept']
        x_line = np.array([motor10_data.min(), motor10_data.max()])
        y_line = slope * x_line + intercept
        ax1.plot(x_line, y_line, 'b-', linewidth=2, label=f'Fit line')
        
        ax1.set_xlabel('Motor10 [°]')
        ax1.set_ylabel(f'{axis_name} Position [m]')
        ax1.set_title(f'除去前: {axis_name}座標 (n={len(motor10_data)})', fontsize=11)
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # 【列2】除去前：残差プロット
        ax2 = axes[col, 1]
        expected = slope * motor10_data + intercept
        residuals = position_data[:, axis_idx] - expected
        
        ax2.scatter(motor10_data, residuals, s=30, alpha=0.6, color='blue', label='Valid data')
        if n_total_outliers > 0:
            ax2.scatter(motor10_data[outlier_indices], residuals[outlier_indices],
                       s=50, alpha=0.8, color='red', marker='x', linewidth=2, label='Outliers')
        
        ax2.axhline(y=0, color='black', linestyle='--', linewidth=1, alpha=0.5)
        ax2.set_xlabel('Motor10 [°]')
        ax2.set_ylabel(f'Residual [m]')
        ax2.set_title(f'除去前の残差: {axis_name}座標', fontsize=11)
        ax2.grid(True, alpha=0.3)
        ax2.legend()
        
        # 【列3】除去後：Motor10 vs 位置座標
        ax3 = axes[col, 2]
        ax3.scatter(cleaned_motor10, cleaned_position[:, axis_idx], s=30, alpha=0.6, color='green')
        
        # 除去後のデータで再度線形回帰
        if len(cleaned_motor10) > 1:
            slope_cleaned, intercept_cleaned, r_cleaned, _, _ = \
                stats.linregress(cleaned_motor10, cleaned_position[:, axis_idx])
            x_line_cleaned = np.array([cleaned_motor10.min(), cleaned_motor10.max()])
            y_line_cleaned = slope_cleaned * x_line_cleaned + intercept_cleaned
            ax3.plot(x_line_cleaned, y_line_cleaned, 'r-', linewidth=2,
                    label=f'Fit (R²={r_cleaned**2:.4f})')
        
        ax3.set_xlabel('Motor10 [°]')
        ax3.set_ylabel(f'{axis_name} Position [m]')
        ax3.set_title(f'除去後: {axis_name}座標 (n={len(cleaned_motor10)})', fontsize=11)
        ax3.grid(True, alpha=0.3)
        ax3.legend()
    
    plt.tight_layout()
    
    # グラフを保存
    graph_output_path = output_path.replace('.csv', '_comparison.png')
    plt.savefig(graph_output_path, dpi=150, bbox_inches='tight')
    print(f"✅ 比較グラフを保存: {graph_output_path}\n")
    plt.show()
    
    return output_path


def main():
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  1. データ分布の確認:")
        print("    python3 check_outliers.py <csv_path>")
        print("\n  2. 外れ値の検出:")
        print("    python3 check_outliers.py <csv_path> <marker_id> [method] [threshold]")
        print("\n  3. 外れ値を除いたCSVを保存:")
        print("    python3 check_outliers.py <csv_path> <marker_id> [method] [threshold] --save")
        print("\n例:")
        print("  # データ分布を確認")
        print("  python3 check_outliers.py ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_205211.csv")
        print("\n  # ID=0の外れ値を検出")
        print("  python3 check_outliers.py ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_205211.csv 0")
        print("\n  # ID=0の外れ値を除いたCSVを保存（IQR法、係数1.5）")
        print("  python3 check_outliers.py ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_205211.csv 0 iqr 1.5 --save")
        print("\n  # Z-score法で外れ値を除く")
        print("  python3 check_outliers.py ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_205211.csv 0 zscore 2.5 --save")
        return
    
    csv_path = sys.argv[1]
    
    if not os.path.exists(csv_path):
        print(f"エラー: ファイルが見つかりません: {csv_path}")
        return
    
    # ステップ1: データ分布を確認
    unique_ids = analyze_data_distribution(csv_path)
    
    # ステップ2: marker_idが指定されたら外れ値を検出・除去
    if len(sys.argv) >= 3:
        marker_id = int(sys.argv[2])
        
        if marker_id not in unique_ids:
            print(f"エラー: marker_id={marker_id} は存在しません")
            return
        
        method = sys.argv[3] if len(sys.argv) > 3 else 'iqr'
        threshold = float(sys.argv[4]) if len(sys.argv) > 4 else 1.5
        
        # motor10の範囲を指定（デフォルトは自動判定）
        motor10_min = None
        motor10_max = -50.0  # motor10は-53°より小さいはずなので、-50より大きい値は異常
        
        # --saveフラグをチェック
        save_flag = '--save' in sys.argv
        
        if save_flag:
            # 外れ値を除いたCSVを保存
            output_path = None
            if '--output' in sys.argv:
                idx = sys.argv.index('--output')
                if idx + 1 < len(sys.argv):
                    output_path = sys.argv[idx + 1]
            
            save_cleaned_csv(csv_path, marker_id, output_path, method, threshold, motor10_min, motor10_max)
        else:
            # 外れ値を検出して表示するだけ
            print(f"\n{'='*60}")
            print(f"⚠️  外れ値検出のみ（CSVは保存されません）")
            print(f"{'='*60}\n")
            detect_outliers(csv_path, marker_id, method, threshold, motor10_min, motor10_max)
            
            print(f"\n💾 外れ値を除いたCSVを保存するには、以下を実行してください:\n")
            print(f"  python3 scripts/check_outliers.py \\")
            print(f"    {csv_path} \\")
            print(f"    {marker_id} {method} {threshold} --save\n")


if __name__ == '__main__':
    main()
