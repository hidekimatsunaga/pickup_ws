#!/usr/bin/env python3
"""
aruco_motor_log_1213_205211.csv から
marker_idごとの線形関係式を導く
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import json
import sys
import os


def analyze_motor10_linearity(csv_path, marker_ids=None):
    """marker_idごとにmotor10と先端位置の線形関係を分析"""
    
    # CSV読み込み
    df = pd.read_csv(csv_path, header=None, skiprows=1)
    
    # 必要な列を抽出
    timestamp = df.iloc[:, 0].astype(float)
    marker_ids_all = df.iloc[:, 11].astype(int)
    motor10 = df.iloc[:, 10].astype(float)
    position = df.iloc[:, 12:15].astype(float)
    position.columns = ['x', 'y', 'z']
    
    # 分析対象のmarker_idを決定
    if marker_ids is None:
        marker_ids = sorted(marker_ids_all.unique())
    
    print(f"\n{'='*60}")
    print(f"Motor10 vs ホース先端位置 線形関係分析")
    print(f"{'='*60}\n")
    
    all_results = {}
    fig_count = len(marker_ids)
    
    if fig_count == 1:
        fig, axes = plt.subplots(1, 3, figsize=(15, 4))
        axes = axes.reshape(1, 3)
    else:
        fig, axes = plt.subplots(fig_count, 3, figsize=(15, 4*fig_count))
    
    fig.suptitle('Motor10 vs ホース先端位置の線形関係（marker_idごと）', fontsize=14)
    
    for row, mid in enumerate(marker_ids):
        mask = marker_ids_all == mid
        motor10_filtered = motor10[mask].values
        position_filtered = position[mask].values
        
        if len(motor10_filtered) == 0:
            print(f"【Marker ID = {mid}】")
            print(f"  データが見つかりません\n")
            continue
        
        print(f"【Marker ID = {mid}】")
        print(f"  データ点数: {len(motor10_filtered)}")
        print(f"  Motor10の範囲: {motor10_filtered.min():.2f}° ~ {motor10_filtered.max():.2f}°\n")
        
        # 各軸について線形回帰
        results = {}
        for i, axis in enumerate(['x', 'y', 'z']):
            position_axis = position_filtered[:, i]
            
            # 線形回帰
            slope, intercept, r_value, p_value, std_err = stats.linregress(motor10_filtered, position_axis)
            
            results[axis] = {
                'slope': slope,
                'intercept': intercept,
                'r_squared': r_value**2,
                'r_value': r_value,
                'p_value': p_value,
                'std_err': std_err,
                'data': position_axis
            }
            
            print(f"  【{axis.upper()}座標】")
            print(f"    線形関係式: {axis} = {slope:.8f} × motor10 + {intercept:.8f}")
            print(f"    R² = {r_value**2:.6f}  (R = {r_value:.6f})")
            print(f"    p値 = {p_value:.2e}")
            print(f"    {axis.upper()}の範囲: {position_axis.min():.6f} ~ {position_axis.max():.6f} [m]\n")
        
        all_results[mid] = results
        
        # プロット
        for col, (axis, ax_row) in enumerate(zip(['x', 'y', 'z'], [0, 1, 2])):
            if fig_count == 1:
                ax = axes[0, col]
            else:
                ax = axes[row, col]
            
            result = results[axis]
            
            # データ点
            ax.scatter(motor10_filtered, result['data'], s=30, alpha=0.6, label='Data')
            
            # 回帰直線
            x_line = np.array([motor10_filtered.min(), motor10_filtered.max()])
            y_line = result['slope'] * x_line + result['intercept']
            ax.plot(x_line, y_line, 'r-', linewidth=2, label='Linear Fit')
            
            ax.set_xlabel('Motor10 [°]', fontsize=10)
            ax.set_ylabel(f'{axis.upper()} Position [m]', fontsize=10)
            ax.set_title(f'ID={mid}: {axis.upper()} (R²={result["r_squared"]:.4f})', fontsize=11)
            ax.grid(True, alpha=0.3)
            ax.legend(fontsize=8)
    
    plt.tight_layout()
    
    # プロット保存
    output_path = csv_path.replace('.csv', '_motor10_linear_analysis_all.png')
    plt.savefig(output_path, dpi=150, bbox_inches='tight')
    print(f"プロットを保存: {output_path}\n")
    
    plt.show()
    
    # 結果をJSON形式で保存
    result_dict = {}
    for mid, data in all_results.items():
        result_dict[str(mid)] = {}
        for axis, res in data.items():
            result_dict[str(mid)][axis] = {
                'slope': float(res['slope']),
                'intercept': float(res['intercept']),
                'r_squared': float(res['r_squared']),
                'r_value': float(res['r_value']),
            }
    
    json_path = csv_path.replace('.csv', '_motor10_linear_fit_all.json')
    with open(json_path, 'w') as f:
        json.dump(result_dict, f, indent=2)
    print(f"結果をJSON保存: {json_path}\n")
    
    # サマリーテーブルを表示
    print(f"{'='*60}")
    print(f"R² 値サマリー（線形性の強さ）")
    print(f"{'='*60}")
    print(f"{'ID':>5} {'X':>10} {'Y':>10} {'Z':>10}")
    print(f"{'-'*40}")
    for mid in marker_ids:
        if mid in all_results:
            x_r2 = all_results[mid]['x']['r_squared']
            y_r2 = all_results[mid]['y']['r_squared']
            z_r2 = all_results[mid]['z']['r_squared']
            print(f"{mid:>5} {x_r2:>10.6f} {y_r2:>10.6f} {z_r2:>10.6f}")
    print(f"{'='*60}\n")
    
    return all_results


def main():
    if len(sys.argv) < 2:
        print("使用方法:")
        print("  python3 analyze_motor10_linearity.py <csv_path> [marker_id1] [marker_id2] ...")
        print("\n例:")
        print("  python3 analyze_motor10_linearity.py ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_205211.csv")
        print("  python3 analyze_motor10_linearity.py ~/pickup_ws/angle_arucopose_csv/aruco_motor_log_1213_205211.csv 0 1")
        return
    
    csv_path = sys.argv[1]
    
    if not os.path.exists(csv_path):
        print(f"エラー: ファイルが見つかりません: {csv_path}")
        return
    
    # marker_idが指定されたら、その分のみ分析
    if len(sys.argv) > 2:
        marker_ids = [int(x) for x in sys.argv[2:]]
    else:
        marker_ids = None  # すべてのIDを分析
    
    analyze_motor10_linearity(csv_path, marker_ids)


if __name__ == '__main__':
    main()
